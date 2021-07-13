#include <atomic>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>

#include "image_debayer/image-reconstruction.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "image_debayer_node");
  ros::NodeHandle nh, nh_private("~");

  debayer::ImageReconstruction recon(nh, nh_private);
  if (!recon.run()) {
    LOG(FATAL) << "Unable to start the image reconstruction.";
  }

  ros::waitForShutdown();
  return 0;
}
