#include "artifact_republisher/artifact-republisher.h"

#include <vio-common/rostopic-settings.h>
#include <vi-map/sensor-utils.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include <boost/bind.hpp>

#include <sstream>
#include <chrono>

namespace maplab {

ArtifactRepublisher::ArtifactRepublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), 
		processed_counter_(0u) {
  if (!initializeSubscribers()) {
     LOG(FATAL) << "[ArtifactRepublisher] " 
       << "Failed initialize subscribers and services.";
  }
}


bool ArtifactRepublisher::run() {
  LOG(INFO) << "[ArtifactRepublisher] Starting...";
  spinner_.start();
  return true;
}

void ArtifactRepublisher::shutdown(){
  // noop
}

std::atomic<bool>& ArtifactRepublisher::shouldExit() {
  return should_exit_;
}

std::string ArtifactRepublisher::printStatistics() const {
  std::stringstream ss;
  ss << "[ArtifactRepublisher]  Statistics \n";
	ss << "\t processed: " << processed_counter_ << " artifacts\n";
  return ss.str();
}

bool ArtifactRepublisher::initializeSubscribers() {
  return true;
}

} // namespace maplab
