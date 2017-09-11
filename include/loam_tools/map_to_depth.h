/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef LOAMTOOLS_MAPTODEPTH_H
#define LOAMTOOLS_MAPTODEPTH_H

#include "loam_tools/camera.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <memory>

namespace loam_tools {
  using sensor_msgs::Image;
  using sensor_msgs::ImageConstPtr;
  using message_filters::TimeSynchronizer;
  using message_filters::Subscriber;
  using message_filters::Synchronizer;
  using message_filters::sync_policies::ApproximateTime;
  using sensor_msgs::PointCloud2;
  using sensor_msgs::PointCloud2ConstPtr;
  using geometry_msgs::PoseWithCovarianceStamped;
  using geometry_msgs::PoseWithCovarianceStampedConstPtr;

  typedef Eigen::Affine3f Transform;
  
  struct CloudListItem {
    CloudListItem(pcl::PointCloud<pcl::PointXYZI>::Ptr cl = pcl::PointCloud<pcl::PointXYZI>::Ptr(),
                  const Transform &p = Transform(),
                  ImageConstPtr  p0 = ImageConstPtr(),
                  ImageConstPtr  p1 = ImageConstPtr()) : cloud(cl), pose(p) {
      imagePtr[0] = p0;
      imagePtr[1] = p1;
    };
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud;
    Transform                             pose; // T_world_lidar
    ImageConstPtr                         imagePtr[2];
  };
  typedef std::list<CloudListItem> CloudList;
  typedef CloudList::const_iterator CloudListConstIt;

  class MapToDepth {
  public:
    MapToDepth(const ros::NodeHandle &pnh);
    ~MapToDepth();

    MapToDepth(const MapToDepth&) = delete;
    MapToDepth& operator=(const MapToDepth&) = delete;

    bool initialize();
    void pointCloudCallback(const PointCloud2ConstPtr &msg,
                            const PoseWithCovarianceStampedConstPtr &pose);
    void laserScanCallback(const PointCloud2ConstPtr &msg,
                           const PoseWithCovarianceStampedConstPtr &pose);
    void leftImageCallback(const sensor_msgs::ImageConstPtr &msg);
    void rightImageCallback(const sensor_msgs::ImageConstPtr &msg);

  private:
    void getLidarExtrinsics();
    void getCameraCalibration();
    void processImage(const sensor_msgs::ImageConstPtr &msg, int camId);

    cv::Affine3d::Mat4 getTransform(const std::string &field) const;

    ros::NodeHandle                                         nh_;
    image_transport::ImageTransport                         imTrans_;
    std::shared_ptr<Subscriber<PointCloud2> >               pointCloudSub_;
    std::shared_ptr<Subscriber<PointCloud2> >               laserScanSub_;
    std::shared_ptr<Subscriber<PoseWithCovarianceStamped> > poseSub_;

    typedef ApproximateTime<PointCloud2, PoseWithCovarianceStamped> ApproxPolicy;
 
    std::shared_ptr<Synchronizer<ApproxPolicy> > pointCloudSync_;
    std::shared_ptr<Synchronizer<ApproxPolicy> > laserScanSync_;

    image_transport::Subscriber     imSub_[2];
    image_transport::Publisher      imPubFromMap_[2];
    image_transport::Publisher      imPubFromScan_[2];
    image_transport::Publisher      imPubGray_[2];
    ros::Publisher                  grayCloudPub_[2];
    Camera                          cam_[2];
    ImageConstPtr                   imagePtr_[2];

    CloudList                       cloudList_;
    int                             numMapsToKeep_{1};  // TODO: not used yet!
  };
}

#endif
