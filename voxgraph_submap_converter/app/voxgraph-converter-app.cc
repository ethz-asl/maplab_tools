#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-ros-common/gflags-interface.h>

#include "voxgraph-converter/voxgraph-converter-node.h"


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "voxgraph_submap_converter");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::VoxgraphConverterNode voxgraph_converter(nh, nh_private);

  if (!voxgraph_converter.run()) {
   ROS_FATAL("Failed to start running the converter node!");
   ros::shutdown();
   return 1;
  }

  std::atomic<bool>& end_of_days_signal_received = voxgraph_converter.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
   VLOG_EVERY_N(1, 30) << "\n" << voxgraph_converter.printStatistics();
   std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  voxgraph_converter.shutdown();
  return 0;
}
