#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-ros-common/gflags-interface.h>

#include <maplab-camera-info-publisher/maplab-camera-info-publisher-node.h>

DEFINE_bool(
  map_save_on_shutdown, true,
  "Save the map on exit. If this is set to false, then the map must "
  "be saved using a service call.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "maplab_service_interface");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::MaplabCameraInfoPublisher maplab_camera_publisher(nh, nh_private);

  if (!maplab_camera_publisher.run()) {
   ROS_FATAL("Failed to start running the maplab node!");
   ros::shutdown();
   return 1;
  }

  std::atomic<bool>& end_of_days_signal_received = maplab_camera_publisher.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
   VLOG_EVERY_N(1, 10) << "\n" << maplab_camera_publisher.printStatistics();
   std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  maplab_camera_publisher.shutdown();
  return 0;
}
