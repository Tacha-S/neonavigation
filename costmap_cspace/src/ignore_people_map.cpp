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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <limits>
#include <string>

#include <costmap_cspace/pointcloud_accumulator.h>
#include <neonavigation_common/compatibility.h>

class IgnorePeopleMapNode
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

  costmap_cspace::PointcloudAccumurator<geometry_msgs::PointStamped> accum_;

public:
  IgnorePeopleMapNode() : nh_(), pnh_("~"), tfl_(tfbuf_)
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
    sub_scan_ = nh_.subscribe("people", 2, &IgnorePeopleMapNode::cbPeople, this);

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
  }

private:
  void cbPeople(const people_msgs::People::ConstPtr& people)
  {
    people_msgs::People people_global;
    for (auto& person : people->people)
    {
      geometry_msgs::PointStamped point, point_global;
      point.header = people->header;
      point.point = person.position;
      try
      {
        geometry_msgs::TransformStamped trans =
            tfbuf_.lookupTransform(global_frame_, people->header.frame_id, people->header.stamp, ros::Duration(0.5));
        tf2::doTransform(point, point_global, trans);
      }
      catch (tf2::TransformException& e)
      {
        ROS_WARN("%s", e.what());
      }
      accum_.push(costmap_cspace::PointcloudAccumurator<geometry_msgs::PointStamped>::Points(
          point_global, point_global.header.stamp));
    }
    ros::Time now = people->header.stamp;
    if (published_ + publish_interval_ > now)
      return;
    published_ = now;

    float robot_z;
    try
    {
      tf2::Stamped<tf2::Transform> trans;
      tf2::fromMsg(tfbuf_.lookupTransform(global_frame_, robot_frame_, ros::Time(0)), trans);

      auto pos = trans.getOrigin();
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

    for (auto& person : accum_)
    {
      unsigned int x = static_cast<int>((person.point.x - map_.info.origin.position.x) / map_.info.resolution);
      unsigned int y = static_cast<int>((person.point.y - map_.info.origin.position.y) / map_.info.resolution);
      int range = static_cast<int>(0.5 / map_.info.resolution);
      for (int i = -range; i < range; i++)
      {
        for (int j = -range; j < range; j++)
        {
          if (x + i >= map_.info.width || y + j >= map_.info.height || x + i < 0 || y + j < 0)
            continue;
          map_.data[x + i + (y + j) * map_.info.width] = 0;
        }
      }
    }

    pub_map_.publish(map_);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ignore_people_map");

  IgnorePeopleMapNode conv;
  ros::spin();

  return 0;
}
