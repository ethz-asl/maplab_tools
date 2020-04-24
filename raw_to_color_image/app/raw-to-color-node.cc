#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "raw_to_color/r2c-manager.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "r2c_manager_node");
  ros::NodeHandle nh, nh_private("~");

  r2c::R2CManager r2c_manager(nh, nh_private);
  if (!r2c_manager.run()) {
    LOG(FATAL) << "Unable to start the r2c manager.";
  }

  ros::waitForShutdown();
  return 0;
}
