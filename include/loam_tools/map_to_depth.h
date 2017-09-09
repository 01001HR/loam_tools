/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MAPTODEPTH_H
#define MAPTODEPTH_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace loam_tools {
  class MapToDepth {
  public:
    MapToDepth(const ros::NodeHandle &pnh);
    ~MapToDepth();

    MapToDepth(const MapToDepth&) = delete;
    MapToDepth& operator=(const MapToDepth&) = delete;

    bool initialize();
    void mapCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  private:
    ros::NodeHandle                 nh_;
    image_transport::ImageTransport imTrans_;
    ros::Subscriber                 mapSub_;
    image_transport::Publisher      imPub_[2];
    int                             numMapsToKeep_{1};
  };
}

#endif
