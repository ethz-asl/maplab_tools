#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-ros-common/gflags-interface.h>

#include "lidar-image-projection/lidar-image-projection-node.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "lidar_image_projection_node");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::LidarImageProjection lidar_image_projection(nh, nh_private);

  if (!lidar_image_projection.run()) {
   ROS_FATAL("Failed to start running the projection node!");
   ros::shutdown();
   return 1;
  }

  std::atomic<bool>& end_of_days_signal_received 
		= lidar_image_projection.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
    VLOG_EVERY_N(1, 10) << "\n" << lidar_image_projection.printStatistics();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  lidar_image_projection.shutdown();
  return 0;
}
