#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <maplab-ros-common/gflags-interface.h>

#include <artifact_republisher/artifact-republisher.h>

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "artifact_republisher");
  ros::NodeHandle nh, nh_private("~");

  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  maplab::ArtifactRepublisher artifact_republisher(nh, nh_private);

  if (!artifact_republisher.run()) {
   ROS_FATAL("Failed to start running the maplab node!");
   ros::shutdown();
   return 1;
  }

  std::atomic<bool>& end_of_days_signal_received 
		= artifact_republisher.shouldExit();
  while (ros::ok() && !end_of_days_signal_received.load()) {
   VLOG_EVERY_N(1, 10) << "\n" << artifact_republisher.printStatistics();
   std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  artifact_republisher.shutdown();
  return 0;
}
