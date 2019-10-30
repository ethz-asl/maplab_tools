#include "ply-publisher/ply-publisher-node.h"
#include <map-resources/resource-conversion.h>

#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/bind.hpp>
#include <sstream>


DEFINE_string(publish_topic, "/ply",
    "Defines the topic used for republishing the converted submaps.");

DEFINE_string(PLY_directory, "",
    "If set will store the extracted submap as PLY to the given path.");

namespace maplab {

PlyPublisher::PlyPublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), processed_submaps_(0) {
  LOG(INFO) << "[PlyPublisher] Initializing publisher...";

  // Initialize the submap republish.
  ply_pub_ = nh.advertise<sensor_msgs::PointCloud2>(FLAGS_publish_topic, 1);
}

bool PlyPublisher::run() {
  LOG(INFO) << "[PlyPublisher] Starting...";
  spinner_.start();
  return true;
}

void PlyPublisher::shutdown(){

}

std::atomic<bool>& PlyPublisher::shouldExit() {
  return should_exit_;
}

std::string PlyPublisher::printStatistics() const {
  std::stringstream ss;
  ss << "[PlyPublisher] \n";
  ss << " \t Processed " << processed_submaps_ << " so far. \n";
  return ss.str();
}

} // namespace maplab
