#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-ros-common/gflags-interface.h>
#include <ros/ros.h>

#include "maplab-node/maplab-ros-node.h"

DEFINE_bool(
    map_save_on_shutdown, true,
    "Save the map on exit. If this is set to false, then the map must "
    "be saved using a service call.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "maplab_node");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::MaplabRosNode maplab_node(nh, nh_private);

  if (!maplab_node.run()) {
    ROS_FATAL("Failed to start running the maplab node!");
    ros::shutdown();
    return 0;
  }

  std::atomic<bool>& end_of_days_signal_received = maplab_node.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
    VLOG_EVERY_N(1, 10) << "\n" << maplab_node.printDeliveryQueueStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  maplab_node.shutdown();
  if (FLAGS_map_save_on_shutdown) {
    maplab_node.saveMap();
  }
  return 0;
}
