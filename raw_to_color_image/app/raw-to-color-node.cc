#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "raw_to_color/r2c-manager.h"
#include "raw_to_color/ros-utils.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "artifact_manager_node");
  ros::NodeHandle nh, nh_private("~");

  r2c::parseGflagsFromRosParams(argv[0], nh_private);
  r2c::R2CManager r2c_manager(nh, nh_private);

  ros::waitForShutdown();
  return 0;
}
