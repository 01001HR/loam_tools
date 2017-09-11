/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "loam_tools/map_to_depth.h"
#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>
#include <algorithm>
#include <math.h>
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
    imPubFromMap_[0] = imTrans_.advertise("left/depth_image_from_map", 1);
    imPubFromMap_[1] = imTrans_.advertise("right/depth_image_from_map", 1);
    imPubFromScan_[0] = imTrans_.advertise("left/depth_image_from_scan", 1);
    imPubFromScan_[1] = imTrans_.advertise("right/depth_image_from_scan", 1);
    imPubGray_[0] = imTrans_.advertise("left/gray_image", 1);
    imPubGray_[1] = imTrans_.advertise("right/gray_image", 1);

    grayCloudPub_[0]  = nh_.advertise<PointCloud2>("left/cloud_unprojected", 1);
    grayCloudPub_[1]  = nh_.advertise<PointCloud2>("right/cloud_unprojected", 1);

    
    imSub_[0] = imTrans_.subscribe("left/image", 1, &MapToDepth::leftImageCallback, this);
    imSub_[1] = imTrans_.subscribe("right/image", 1, &MapToDepth::rightImageCallback, this);

    poseSub_.reset(new Subscriber<PoseWithCovarianceStamped>(nh_, "pose", 0));
    
    //pointCloudSub_.reset(new Subscriber<PointCloud2>(nh_, "pointcloud", 0));
    //pointCloudSync_.reset(new Synchronizer<ApproxPolicy>(ApproxPolicy(10), *pointCloudSub_, *poseSub_));
    //pointCloudSync_->registerCallback(boost::bind(&MapToDepth::pointCloudCallback, this, _1, _2));
    
    laserScanSub_.reset(new Subscriber<PointCloud2>(nh_, "sweep", 0));
    laserScanSync_.reset(new Synchronizer<ApproxPolicy>(ApproxPolicy(10), *laserScanSub_, *poseSub_));
    laserScanSync_->registerCallback(boost::bind(&MapToDepth::laserScanCallback, this, _1, _2));
    

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
    assert(msg->encoding == sensor_msgs::image_encodings::MONO8);
    imagePtr_[camId] = msg;
  }

  static tf::Transform poseToTF(const PoseWithCovarianceStampedConstPtr &pmsg) {
    tf::Transform tf;
    const geometry_msgs::Point &pos = pmsg->pose.pose.position;
    const geometry_msgs::Quaternion &q = pmsg->pose.pose.orientation;
    tf.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
    tf.setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
    return (tf);
  }

  static Transform poseToTransform(const PoseWithCovarianceStampedConstPtr &pmsg) {
    const geometry_msgs::Point      &pos = pmsg->pose.pose.position;
    const geometry_msgs::Quaternion   &q = pmsg->pose.pose.orientation;
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(q.w, q.x, q.y, q.z).toRotationMatrix();
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block(0, 0, 3, 3) = mat3;
    mat4.block(0, 3, 3, 1) = Eigen::Vector3f(pos.x, pos.y, pos.z);
    Transform tf4(mat4);
    return (tf4);
  }
  
  void MapToDepth::pointCloudCallback(const PointCloud2ConstPtr &msg,
                                      const PoseWithCovarianceStampedConstPtr &pmsg) {
    //tf::Transform tf = poseToTF(pmsg).inverse();
 
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    std::vector<cv::Point3f> p3d_lidar(cloud.points.size());
#if 0    
    for (unsigned int i = 0; i < cloud.points.size(); i++) {
      const pcl::PointXYZ &p = cloud.points[i];
      // transform from global to lidar frame
      tf::Vector3 p_l = tf(tf::Vector3(p.x, p.y, p.z));
      p3d_lidar[i] = cv::Point3f(p_l.getX(), p_l.getY(), p_l.getZ());
    }
#else
    Transform T_w_lidar = poseToTransform(pmsg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(cloud, *transformedCloud, T_w_lidar.inverse());

    for (unsigned int i = 0; i < cloud.points.size(); i++) {
      const pcl::PointXYZ &p = cloud.points[i];
      // transform from global to lidar frame
      tf::Vector3 p_l(p.x, p.y, p.z);
      p3d_lidar[i] = cv::Point3f(p_l.getX(), p_l.getY(), p_l.getZ());
    }
#endif
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
      imPubFromMap_[camid].publish(cv_bridge::CvImage(msg->header, "32FC1",
                                               img).toImageMsg());
    }
  }

  static int stitchPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr stitchedCloud,
                               const CloudListConstIt &begin,
                               const CloudListConstIt &current,
                               const CloudListConstIt &end) {
    Transform currPoseInverse = current->pose.inverse();
    int numStitched(0);
    for (CloudListConstIt it = begin; it != end; ++it) {
      Transform relativePose = currPoseInverse * it->pose;
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*(it->cloud), *transformedCloud, relativePose);
      // insert transformed into stitched
      (*stitchedCloud) += *transformedCloud;
      numStitched++;
    }
    return (numStitched);
  }

  //
  // project point cloud from laser frame to camera frame
  //
  static void project3dToImg(cv::Mat *img, const Camera &cam, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
    std::vector<cv::Point3f> p3(cloud->points.size());
    // TODO: can we avoid this extra copy somehow?
    for (unsigned int i = 0; i < cloud->points.size(); i++) {
      const pcl::PointXYZI &p = cloud->points[i];
      p3[i] = cv::Point3f(p.x, p.y, p.z);
    }
    std::vector<cv::Point2f> im(cloud->size());
    cv::fisheye::projectPoints(p3, im, cam.T_cam_lidar, cam.K, cam.dist, /*skew*/0.0);
    for (unsigned int i = 0; i < im.size(); i++) {
      cv::Point2i ip = im[i]; // cast to integer
      if (ip.x >= 0 && ip.x < cam.res[0] &&
          ip.y >= 0 && ip.y < cam.res[1]) {
        // point is within camera bounds
        const cv::Point3f &p = p3[i];
        cv::Vec<float, 4> p4d(p.x, p.y, p.z, 1.0);
        cv::Vec<float, 4> ptrans = cam.T_cam_lidar.matrix * p4d;
        if (ptrans(2) > 0) {
          float oldpt = img->at<float>(ip.y, ip.x);
          if (std::isnan(oldpt) || ptrans(2) < oldpt) {
            img->at<float>(ip.y, ip.x) = ptrans(2);
          }
        }
      }
    }
  }

  void MapToDepth::laserScanCallback(const PointCloud2ConstPtr &msg,
                                     const PoseWithCovarianceStampedConstPtr &pmsg) {
    Transform T_w_lidar = poseToTransform(pmsg);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    cloudList_.push_back(CloudListItem(cloud, T_w_lidar, imagePtr_[0], imagePtr_[1]));
    if ((int)cloudList_.size() > 2 * numMapsToKeep_ + 1) {
      cloudList_.pop_front();
    }
    ROS_INFO_STREAM("got pointcloud of size " << cloud->points.size() << " list size is: " << cloudList_.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr stitchedCloud(new pcl::PointCloud<pcl::PointXYZI>());
    CloudList::const_iterator currImg = std::next(cloudList_.begin(), cloudList_.size()/2);
    int numStitched = stitchPointClouds(stitchedCloud, cloudList_.begin(), currImg, cloudList_.end());
    //ROS_INFO_STREAM("stitched: " << numStitched);
    for (int camid = 0; camid < 2; camid++) {
      const Camera &cam = cam_[camid];
      double NaNf = std::numeric_limits<float>::quiet_NaN();
      cv::Mat img(cam.res[1], cam.res[0], CV_32FC1, cv::Scalar(NaNf));
      //project3dToImg(&img, cam, cloud);
      project3dToImg(&img, cam, stitchedCloud);
      if (currImg->imagePtr[camid]) {
        const ImageConstPtr &imgMsg = currImg->imagePtr[camid];
        imPubGray_[camid].publish(imgMsg);
        imPubFromScan_[camid].publish(cv_bridge::CvImage(imgMsg->header, "32FC1",
                                                         img).toImageMsg());
      } else {
        imPubFromScan_[camid].publish(cv_bridge::CvImage(msg->header, "32FC1",
                                                         img).toImageMsg());
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr grayCloud(new pcl::PointCloud<pcl::PointXYZI>());
      //unprojectImage(&cameraImage_[camid], pcl, grayCloud);
      //grayCloudPub_[0]
    }
  }

}  // namespace
