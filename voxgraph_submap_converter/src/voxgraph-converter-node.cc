#include "voxgraph-converter/voxgraph-converter-node.h"
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>


namespace maplab {

VoxgraphConverterNode::VoxgraphConverterNode(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false) {
  LOG(INFO) << "[VoxgraphConverter] Initializing converter...";
}

bool VoxgraphConverterNode::run() {
  LOG(INFO) << "[VoxgraphConverter] Starting...";
  spinner_.start();
  return true;
}

void VoxgraphConverterNode::shutdown(){

}

std::atomic<bool>& VoxgraphConverterNode::shouldExit() {
  return should_exit_;
}

std::string VoxgraphConverterNode::printStatistics() const {
  std::stringstream ss;
  ss << "[VoxgraphConverter] \n";
  return ss.str();
}

} // namespace maplab
