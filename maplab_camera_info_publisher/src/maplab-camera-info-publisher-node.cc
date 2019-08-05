#include <maplab-camera-info-publisher/maplab-camera-info-publisher-node.h>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>

namespace maplab {

MaplabCameraInfoPublisher::MaplabCameraInfoPublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false) {
  LOG(INFO) << "[MaplabCameraInfoPublisher] Initializing camera info publisher...";
  
}

bool MaplabCameraInfoPublisher::run() {
  LOG(INFO) << "[MaplabCameraInfoPublisher] Starting...";
  spinner_.start();
  return true;
}

void MaplabCameraInfoPublisher::shutdown(){

}

std::atomic<bool>& MaplabCameraInfoPublisher::shouldExit() {
  return should_exit_;
}

std::string MaplabCameraInfoPublisher::printStatistics() const {
  std::stringstream ss;
  ss << "[MaplabCameraInfoPublisher]  Statistics \n";
  return ss.str();
}

} // namespace maplab
