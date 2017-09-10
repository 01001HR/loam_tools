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
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>
#include <algorithm>
#include <XmlRpcValue.h>

namespace loam_tools {
  template <typename T>
  static T get_param(ros::NodeHandle *nh, std::string name, const T &def) {
    T val;
    if (!nh->param(name, val, def)) {
      throw (std::runtime_error("cannot find parameter " + name));
    }
    return (val);
  }
  static inline cv::Point3d transform_pt(const cv::Point3d p, const tf::Transform &T) {
      const tf::Vector3 v3d(p.x, p.y, p.z);
      const tf::Vector3 vt(T(v3d));
      return (cv::Point3d(vt.x(), vt.y(), vt.z()));
  }
  
  MapToDepth::MapToDepth(const ros::NodeHandle& pnh) :
    nh_(pnh),
    imTrans_(pnh)
  {
  }

  MapToDepth::~MapToDepth() {
  }

  bool MapToDepth::initialize() {
    imPub_[0] = imTrans_.advertise("left/depth_image", 1);
    imPub_[1] = imTrans_.advertise("right/depth_image", 1);
    //TODO: use camera image and publish colored pointcloud
    //imSub_[0] = imTrans_.subscribe("left/image", 1, &MapToDepth::leftImageCallback, this);
    //imSub_[1] = imTrans_.subscribe("right/image", 1, &MapToDepth::rightImageCallback, this);

    poseSub_.reset(new Subscriber<PoseWithCovarianceStamped>(nh_, "pose", 1));
    pointCloudSub_.reset(new Subscriber<PointCloud2>(nh_, "pointcloud", 1));
    pointCloudSync_.reset(new Synchronizer<ApproxPolicy>(ApproxPolicy(10), *pointCloudSub_, *poseSub_));
    pointCloudSync_->registerCallback(boost::bind(&MapToDepth::pointCloudCallback, this, _1, _2));

    numMapsToKeep_ = get_param(&nh_, "num_maps_to_keep", int(1));
    getCameraCalibration();
    getLidarExtrinsics();
    
    return (true);
  }

  void MapToDepth::getCameraCalibration() {
    // cam0 is left, cam1 is right
    std::vector<std::string> cams = {"cam0", "cam1"};
    for (unsigned int camid = 0; camid < cams.size(); camid++) {
      const auto &cam = cams[camid];
      std::string dist_model;
      nh_.getParam(cam + "/distortion_model", dist_model);
      if (dist_model != "equidistant") {
        ROS_ERROR_STREAM("INVALID DISTORTION MODEL: " << dist_model << " for cam " << cam);
        continue;
      }
      std::vector<double> intrinsics, distortion;
      std::vector<int> resolution;
      nh_.getParam(cam + "/intrinsics", intrinsics);
      nh_.getParam(cam + "/distortion_coeffs", distortion);
      nh_.getParam(cam + "/resolution", resolution);
      double Kd[9] =  {intrinsics[0], 0.0, intrinsics[2],
                       0.0, intrinsics[1], intrinsics[3],
                       0.0, 0.0, 1.0};
      cam_[camid].K       = cv::Mat(3,3,CV_64FC1, Kd).clone();
      cam_[camid].dist    = cv::Mat(4,1,CV_64FC1, &distortion[0]).clone();
      cam_[camid].res[0]  = resolution[0];
      cam_[camid].res[1]  = resolution[1];
      ROS_INFO_STREAM("---- " << cam << " ------------- " << std::endl << cam_[camid]);
    }
  }

  cv::Affine3d::Mat4 MapToDepth::getTransform(const std::string &field) const {
    cv::Affine3d::Mat4 T;
    XmlRpc::XmlRpcValue lines;
    nh_.getParam(field, lines);
    if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_STREAM("invalid data in calib file transform " << field);
      return (T);
    }
    for (int i = 0; i < lines.size(); i++) {
      if (lines.size() != 4 || lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("bad line in calib file transform " << field);
        return (T);
      }
      for (int j = 0; j < lines[i].size(); j++) {
        if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
          ROS_ERROR_STREAM("bad value in calib file transform " << field);
        } else {
          T(i,j) = static_cast<double>(lines[i][j]);
        }
      }
    }
    //ROS_INFO_STREAM("read transform " << field << std::endl << T);
    return (T);
  }

  void MapToDepth::getLidarExtrinsics()  {
    cv::Affine3d::Mat4 T_cam1_lidar = getTransform("lidar/T_lidar_cam1").inv();
    cv::Affine3d::Mat4 T_cam1_cam0 = getTransform("cam1/T_cn_cnm1");
    
    // calculate transforms from lidar to camera frames
    cam_[0].T_cam_lidar = cv::Affine3d(T_cam1_cam0.inv() * T_cam1_lidar);
    cam_[1].T_cam_lidar = cv::Affine3d(T_cam1_lidar);

    ROS_INFO_STREAM("T_cam0_lidar" << std::endl << cam_[0].T_cam_lidar.matrix);
    ROS_INFO_STREAM("T_cam1_lidar" << std::endl << cam_[1].T_cam_lidar.matrix);
  }

  void MapToDepth::leftImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    processImage(msg, 0);
  }
  void MapToDepth::rightImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    processImage(msg, 1);
  }

  void MapToDepth::processImage(const sensor_msgs::ImageConstPtr &msg, int camId) {
  }
  
  void MapToDepth::pointCloudCallback(const PointCloud2ConstPtr &msg,
                                      const PoseWithCovarianceStampedConstPtr &pmsg) {
    tf::Transform tf;
    const geometry_msgs::Point &pos = pmsg->pose.pose.position;
    const geometry_msgs::Quaternion &q = pmsg->pose.pose.orientation;
    tf.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    tf.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
    tf = tf.inverse();
    
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<cv::Point3f> p3d_lidar(cloud.points.size());
    for (unsigned int i = 0; i < cloud.points.size(); i++) {
      const pcl::PointXYZ &p = cloud.points[i];
      // transform from global to lidar frame
      tf::Vector3 p_l = tf(tf::Vector3(p.x, p.y, p.z));
      p3d_lidar[i] = cv::Point3f(p_l.getX(), p_l.getY(), p_l.getZ());
    }
    ROS_INFO_STREAM("got pointcloud of size " << p3d_lidar.size());

    double NaNf = std::numeric_limits<float>::quiet_NaN();
    for (int camid = 0; camid < 2; camid++) {
      const Camera &cam = cam_[camid];
      std::vector<cv::Point2f> im(p3d_lidar.size());
      
      cv::fisheye::projectPoints(p3d_lidar, im, cam.T_cam_lidar,
                                 cam.K, cam.dist, 0.0 /*skew */);
      cv::Mat img(cam.res[1], cam.res[0], CV_32FC1, cv::Scalar(NaNf));
      for (unsigned int i = 0; i < im.size(); i++) {
        cv::Point2i ip = im[i]; // cast to integer
        if (ip.x >= 0 && ip.x < cam.res[0] &&
            ip.y >= 0 && ip.y < cam.res[1]) {
          const cv::Point3f &p = p3d_lidar[i];
          cv::Vec<float, 4> p4d(p.x, p.y, p.z, 1.0);
          cv::Vec<float, 4> ptrans = cam.T_cam_lidar.matrix * p4d;
          if (ptrans(2) > 0) {
            img.at<float>(ip.y, ip.x) = ptrans(2);
          }
        }
      }
      imPub_[camid].publish(cv_bridge::CvImage(msg->header, "32FC1",
                                               img).toImageMsg());
    }
  }

}  // namespace
