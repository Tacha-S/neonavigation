#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Range.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_ros/point_cloud.h>

#include <string>

#include <costmap_cspace/pointcloud_accumulator.h>
#include <neonavigation_common/compatibility.h>


class RangeToMapNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_map_;
  // ros::Publisher pointcloud_pub;
  std::vector<ros::Subscriber> sub_range_;

  nav_msgs::OccupancyGrid map;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  ros::Time published_;
  ros::Duration publish_interval_;

  std::string global_frame_;
  std::string robot_frame_;

  unsigned int width_;
  unsigned int height_;
  float origin_x_;
  float origin_y_;
  double hz;

  costmap_cspace::PointcloudAccumurator<sensor_msgs::PointCloud2> accum_;

public:

  RangeToMapNode() : nh_(), pnh_("~"), tfl_(tfbuf_)
  {
    neonavigation_common::compat::checkCompatMode();
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_frame", robot_frame_, std::string("base_link"));

    double accum_duration;
    pnh_.param("accum_duration", accum_duration, 1.0);
    accum_.reset(ros::Duration(accum_duration));

    pub_map_ = neonavigation_common::compat::advertise<nav_msgs::OccupancyGrid>(nh_, "map_local", pnh_, "map", 1, true);

    int width_param;
    pnh_.param("width", width_param, 30);
    height_ = width_ = width_param;
    map.header.frame_id = global_frame_;

    double resolution;
    pnh_.param("resolution", resolution, 0.1);
    map.info.resolution = resolution;
    map.info.width = width_;
    map.info.height = height_;
    map.data.resize(map.info.width * map.info.height);

    pnh_.param("hz", hz, 1.0);

    /* get sensors from param */
    std::vector<std::string> sensor_topic_list;
    pnh_.getParam("sensor_topic_list", sensor_topic_list);
    if (sensor_topic_list.size() == 0)
      ROS_WARN("no range sensor configured");

    sub_range_.resize(sensor_topic_list.size());
    for (int i = 0; i < sensor_topic_list.size(); i++)
    {
      sub_range_.push_back(nh_.subscribe(sensor_topic_list[i], 20, &RangeToMapNode::cbRange, this));
    }

    // pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2> ("/test_point", 3);
  }

  void cbRange(const sensor_msgs::Range::ConstPtr& range)
  {
    /* convert sensor_msgs/Range to sensor_msgs/PointCloud2 */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pointCloud->header.frame_id = robot_frame_;
    pointCloud->height = 1;
    pointCloud->points.clear();
    if (!(range->range < 0 || range->range >= range->max_range)){
      geometry_msgs::TransformStamped transform;
      try
      {
        transform = tfbuf_.lookupTransform(pointCloud->header.frame_id, range->header.frame_id, ros::Time(0));
        geometry_msgs::PointStamped pt;
        pt.point.x = range->range;
        geometry_msgs::PointStamped pointOut;
        tf2::doTransform(pt, pointOut, transform);

        pcl::PointXYZ pcl_point;
        pcl_point.x = pointOut.point.x;
        pcl_point.y = pointOut.point.y;
        pcl_point.z = pointOut.point.z;
        pointCloud->points.push_back(pcl_point);
        ++(pointCloud->width);
      }
      catch (tf2::TransformException& e)
      {
        ROS_WARN("%s", e.what());
      }
    }
    // pointcloud_pub.publish(pointCloud);

    /* Load pointcloud to map */
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(*pointCloud, cloud);
    sensor_msgs::PointCloud2 cloud_global;
    geometry_msgs::TransformStamped trans;
    try
    {
      trans = tfbuf_.lookupTransform(global_frame_, cloud.header.frame_id, cloud.header.stamp, ros::Duration(0.5));
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    tf2::doTransform(cloud, cloud_global, trans);
    accum_.push(costmap_cspace::PointcloudAccumurator<sensor_msgs::PointCloud2>::Points(cloud_global,
                                                                                        cloud_global.header.stamp));

    ros::Time now = range->header.stamp;
    if(published_ + publish_interval_ > now)
        return;
    published_ = now;

    try
    {
      tf2::Stamped<tf2::Transform> trans;
      tf2::fromMsg(tfbuf_.lookupTransform(global_frame_, robot_frame_, ros::Time(0)), trans);

      auto pos = trans.getOrigin();
      float x = static_cast<int>(pos.x() / map.info.resolution) * map.info.resolution;
      float y = static_cast<int>(pos.y() / map.info.resolution) * map.info.resolution;
      map.info.origin.position.x = x - map.info.width * map.info.resolution * 0.5;
      map.info.origin.position.y = y - map.info.height * map.info.resolution * 0.5;
      map.info.origin.position.z = 0.0;
      map.info.origin.orientation.w = 1.0;
      origin_x_ = x - width_ * map.info.resolution * 0.5;
      origin_y_ = y - height_ * map.info.resolution * 0.5;
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("%s", e.what());
      return;
    }
    for (auto& cell : map.data)
      cell = 0;

    for (auto& pc : accum_)
    {
      auto itr_x = sensor_msgs::PointCloud2ConstIterator<float>(pc, "x");
      auto itr_y = sensor_msgs::PointCloud2ConstIterator<float>(pc, "y");
      for (; itr_x != itr_x.end(); ++itr_x, ++itr_y)
      {
        unsigned int x = int((*itr_x - map.info.origin.position.x) / map.info.resolution);
        unsigned int y = int((*itr_y - map.info.origin.position.y) / map.info.resolution);
        if (x >= map.info.width || y >= map.info.height)
          continue;
        map.data[x + y * map.info.width] = 100;
      }
    }
    pub_map_.publish(map);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_to_map");

  RangeToMapNode conv;
  ros::spin();

  return 0;
}
