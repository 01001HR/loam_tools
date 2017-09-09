/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "loam_tools/map_to_depth.h"
#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include <string>
#include <vector>
#include <algorithm>

namespace loam_tools {
  template <typename T>
  static T get_param(ros::NodeHandle *nh, std::string name, const T &def) {
    T val;
    if (!nh->param(name, val, def)) {
      throw (std::runtime_error("cannot find parameter " + name));
    }
    return (val);
  }
  
  MapToDepth::MapToDepth(const ros::NodeHandle& pnh) :
    nh_(pnh),
    imTrans_(pnh)
  {
  }

  MapToDepth::~MapToDepth() {
  }

  bool MapToDepth::initialize() {
    imPub_[0] = imTrans_.advertise("left/image", 1);
    imPub_[1] = imTrans_.advertise("right/image", 1);
    mapSub_   = nh_.subscribe("localmap", 1, &MapToDepth::mapCallback, this);
    numMapsToKeep_ = get_param(&nh_, "num_maps_to_keep", int(1));
    return (true);
  }

#if 0  
  static tf::Transform odom_to_tf(const OdometryConstPtr &odom) {
    const auto &q = odom->pose.pose.orientation;
    const auto &T = odom->pose.pose.position;
    tf::Quaternion qt(q.x, q.y, q.z, q.w);
    tf::Vector3    Tt(T.x, T.y, T.z);
    return (tf::Transform(qt, Tt));
  }
#endif
  
  void MapToDepth::mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    cv::Mat img(1024, 1024, CV_32FC1);
    //cv::cvtColor(im1_ptr->image, img, cv::COLOR_GRAY2BGR);
    imPub_[0].publish(cv_bridge::CvImage(msg->header, "bgr8",
                                      img).toImageMsg());
  }

}  // namespace
