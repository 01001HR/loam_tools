/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include "loam_tools/map_to_depth.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_to_depth_node");
  ros::NodeHandle pnh("~");

  try {
    loam_tools::MapToDepth m2d(pnh);
    m2d.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}
