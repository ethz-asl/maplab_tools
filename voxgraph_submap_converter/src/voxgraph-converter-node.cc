#include "voxgraph-converter/voxgraph-converter-node.h"
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>

DEFINE_string(submap_topic, "/voxgraph_mapper/submap_surface_pointclouds",
    "Defines the topic for the voxgraph submaps.");

namespace maplab {

VoxgraphConverterNode::VoxgraphConverterNode(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false) {
  LOG(INFO) << "[VoxgraphConverter] Initializing converter...";
  
  // Initialize submap callback.
  boost::function<void(const voxgraph_msgs::MapSurfaceConstPtr&)>
    submap_callback = boost::bind(&VoxgraphConverterNode::submapCallback,
        this, _1);
  submap_sub_ = nh.subscribe<voxgraph_msgs::MapSurface>(
      FLAGS_submap_topic, 1, submap_callback);
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

void VoxgraphConverterNode::submapCallback(
    const voxgraph_msgs::MapSurfaceConstPtr& msg){
  VLOG(1) << "received submap";  
}

} // namespace maplab
