/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef LOAMTOOLS_CAMERA_H
#define LOAMTOOLS_CAMERA_H

#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
#include <iostream>

namespace loam_tools {

  struct Camera {
    Camera() { res[0] = 0; res[1] = 0;
      T_cam_lidar = cv::Affine3d::Identity(); 
    }
    cv::Mat       K;
    cv::Mat       dist;
    cv::Affine3d  T_cam_lidar;
    int           res[2];
  };

  std::ostream &operator<<(std::ostream &os, const Camera &c);
}

#endif
