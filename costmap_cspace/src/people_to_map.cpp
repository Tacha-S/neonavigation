/*********************************************************************
 * Copyright (c) 2020 SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <people_msgs/People.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <limits>
#include <string>

#include <costmap_cspace/pointcloud_accumulator.h>
#include <neonavigation_common/compatibility.h>

class PeopleToMapNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_map_;
  ros::Subscriber sub_scan_;

  nav_msgs::OccupancyGrid map_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  laser_geometry::LaserProjection projector_;
  ros::Time published_;
  ros::Duration publish_interval_;

  double z_min_, z_max_;
  std::string global_frame_;
  std::string robot_frame_;

  unsigned int width_;
  unsigned int height_;
  float origin_x_;
  float origin_y_;
  double cutoff_;
  double amp_;
  double cov_;
  double factor_;

  costmap_cspace::PointcloudAccumurator<people_msgs::People> accum_;

public:
  PeopleToMapNode() : nh_(), pnh_("~"), tfl_(tfbuf_)
  {
    neonavigation_common::compat::checkCompatMode();
    pnh_.param("z_min", z_min_, std::numeric_limits<double>::lowest());
    pnh_.param("z_max", z_max_, std::numeric_limits<double>::max());
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_frame", robot_frame_, std::string("base_link"));

    double accum_duration;
    pnh_.param("accum_duration", accum_duration, 1.0);
    accum_.reset(ros::Duration(accum_duration));

    pub_map_ = neonavigation_common::compat::advertise<nav_msgs::OccupancyGrid>(nh_, "map_local", pnh_, "map", 1, true);
    sub_scan_ = nh_.subscribe("people", 2, &PeopleToMapNode::cbScan, this);

    int width_param;
    pnh_.param("width", width_param, 30);
    height_ = width_ = width_param;
    map_.header.frame_id = global_frame_;

    double resolution;
    pnh_.param("resolution", resolution, 0.1);
    map_.info.resolution = resolution;
    map_.info.width = width_;
    map_.info.height = height_;
    map_.data.resize(map_.info.width * map_.info.height);

    double hz;
    pnh_.param("hz", hz, 1.0);
    publish_interval_ = ros::Duration(1.0 / hz);

    pnh_.param("cutoff", cutoff_, 10.);
    pnh_.param("amplitude", amp_, 100.);
    pnh_.param("covariance", cov_, 0.25);
    pnh_.param("factor", factor_, 5.);
  }

private:
  void cbScan(const people_msgs::People::ConstPtr& people)
  {
    people_msgs::People people_global;
    people_global.header = people->header;
    people_global.header.frame_id = global_frame_;
    try
    {
      geometry_msgs::TransformStamped trans =
          tfbuf_.lookupTransform(global_frame_, people->header.frame_id, people->header.stamp, ros::Duration(0.5));
      for (const auto& person : people->people)
      {
        people_msgs::Person person_global = person;
        tf2::doTransform(person.position, person_global.position, trans);
        geometry_msgs::TransformStamped rotate = trans;
        rotate.transform.translation = geometry_msgs::Vector3();
        tf2::doTransform(person.velocity, person_global.velocity, rotate);
        people_global.people.push_back(person_global);
      }
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
    }

    accum_.push(
        costmap_cspace::PointcloudAccumurator<people_msgs::People>::Points(people_global, people_global.header.stamp));

    ros::Time now = people->header.stamp;
    if (published_ + publish_interval_ > now)
      return;
    published_ = now;

    float robot_z;
    tf2::Vector3 pos;
    try
    {
      tf2::Stamped<tf2::Transform> trans;
      tf2::fromMsg(tfbuf_.lookupTransform(global_frame_, robot_frame_, ros::Time(0)), trans);

      pos = trans.getOrigin();
      float x = static_cast<int>(pos.x() / map_.info.resolution) * map_.info.resolution;
      float y = static_cast<int>(pos.y() / map_.info.resolution) * map_.info.resolution;
      map_.info.origin.position.x = x - map_.info.width * map_.info.resolution * 0.5;
      map_.info.origin.position.y = y - map_.info.height * map_.info.resolution * 0.5;
      map_.info.origin.position.z = 0.0;
      map_.info.origin.orientation.w = 1.0;
      origin_x_ = x - width_ * map_.info.resolution * 0.5;
      origin_y_ = y - height_ * map_.info.resolution * 0.5;
      robot_z = pos.z();
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    for (auto& cell : map_.data)
      cell = 0;

    for (auto& people : accum_)
    {
      for (auto& person : people.people)
      {
        tf2::Vector3 velocity(person.velocity.x, person.velocity.y, 0);
        double vel_angle = std::atan2(velocity.y(), velocity.x());
        double factor = 1.0 + velocity.length() * factor_;
        double base_r = getRadius(cov_);
        double spread_r = getRadius(cov_ * factor);

        int width = std::max(1, static_cast<int>((base_r + spread_r) / map_.info.resolution)),
            height = std::max(1, static_cast<int>((base_r + spread_r) / map_.info.resolution));

        tf2::Vector3 center(person.position.x, person.position.y, 0);
        double dist = (center - pos).length() - 2;
        if (dist < 0)
          dist = 0;
        tf2::Vector3 center_to_corner(map_.info.width * map_.info.resolution * 0.5,
                                      map_.info.height * map_.info.resolution * 0.5, 0);
        double max_dist = center_to_corner.length() - 2;
        if (max_dist < 0)
          max_dist = 0;

        tf2::Vector3 top_left;

        if (std::cos(vel_angle) > 0)
          top_left.setX(center.x() - base_r);
        else
          top_left.setX(center.x() + (spread_r - base_r) * std::cos(vel_angle) - base_r);
        if (std::sin(vel_angle) > 0)
          top_left.setY(center.y() - base_r);
        else
          top_left.setY(center.y() + (spread_r - base_r) * std::sin(vel_angle) - base_r);

        int left = static_cast<int>((top_left.x() - map_.info.origin.position.x) / map_.info.resolution);
        int top = static_cast<int>((top_left.y() - map_.info.origin.position.y) / map_.info.resolution);
        for (int i = 0; i < width; i++)
        {
          for (int j = 0; j < height; j++)
          {
            if (left + i >= map_.info.width || top + j >= map_.info.height || left + i < 0 || top + j < 0)
              continue;

            tf2::Vector3 calc_position((left + i) * map_.info.resolution + map_.info.origin.position.x,
                                       (top + j) * map_.info.resolution + map_.info.origin.position.y, 0);
            double diff = velocity.angle(calc_position - center);
            double cost = max_dist == 0 ? 1 : 1 - dist / max_dist;
            if (std::fabs(diff) < M_PI / 2)
              cost *= calcCost(calc_position - center, cov_ * factor, cov_, vel_angle);
            else
              cost *= calcCost(calc_position - center, cov_, cov_, 0);
            if (cost < cutoff_)
              continue;

            map_.data[left + i + (top + j) * map_.info.width] = cost;
          }
        }
      }
    }

    pub_map_.publish(map_);
  }

  double getRadius(double var)
  {
    return sqrt(-2 * var * log(cutoff_ / amp_));
  }

  double calcCost(tf2::Vector3 position, double var_x, double var_y, double base_rotation)
  {
    double norm = position.length();
    double angle = std::atan2(position.y(), position.x());
    double mx = std::cos(angle - base_rotation) * norm;
    double my = std::sin(angle - base_rotation) * norm;
    double f1 = std::pow(mx, 2.0) / (2.0 * var_x), f2 = std::pow(my, 2.0) / (2.0 * var_y);
    return amp_ * std::exp(-(f1 + f2));
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_to_map");

  PeopleToMapNode conv;
  ros::spin();

  return 0;
}
