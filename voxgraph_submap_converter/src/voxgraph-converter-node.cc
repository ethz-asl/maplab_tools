#include "voxgraph-converter/voxgraph-converter-node.h"
#include <map-resources/resource-conversion.h>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>

DEFINE_string(submap_topic, "/voxgraph_mapper/submap_surface_pointclouds",
    "Defines the topic for the voxgraph submaps.");

DEFINE_string(republish_topic, "/submap",
    "Defines the topic used for republishing the converted submaps.");

DEFINE_string(PLY_directory, "",
    "If set will store the extracted submap as PLY to the given path.");

namespace maplab {

VoxgraphConverterNode::VoxgraphConverterNode(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), processed_submaps_(0) {
  LOG(INFO) << "[VoxgraphConverter] Initializing converter...";
  
  // Initialize submap callback.
  boost::function<void(const voxgraph_msgs::MapSurfaceConstPtr&)>
    submap_callback = boost::bind(&VoxgraphConverterNode::submapCallback,
        this, _1);
  submap_sub_ = nh.subscribe<voxgraph_msgs::MapSurface>(
      FLAGS_submap_topic, 1, submap_callback);

  // Initialize the submap republish.
  pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>(FLAGS_republish_topic, 1);
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
  ss << " \t Processed " << processed_submaps_ << " so far. \n";
  ss << " \t Export: " << std::boolalpha 
    << !FLAGS_PLY_directory.empty() << "\n";
  return ss.str();
}

void VoxgraphConverterNode::submapCallback(
    const voxgraph_msgs::MapSurfaceConstPtr& msg){
  VLOG(3) << "Received new submap.";
  sensor_msgs::PointCloud2 pc = msg->pointcloud;
  pc.header.frame_id = "map";
  pcl_pub_.publish(pc);

  if (!FLAGS_PLY_directory.empty()) {
    saveSubmapAsPLY(pc);
  }
  ++processed_submaps_;
}

void VoxgraphConverterNode::saveSubmapAsPLY(
    const sensor_msgs::PointCloud2& msg) {
  resources::PointCloud maplab_pointcloud;
  backend::convertPointCloudType(msg, &maplab_pointcloud);
  VLOG(3) << "Converted maplab_pcl points: " << maplab_pointcloud.xyz.size();
  VLOG(3) << "Converted maplab_pcl points: " << maplab_pointcloud.scalars.size();

  VLOG(2) << "Exporting to PLY to: " 
    << normalizedDirectoryPath(processed_submaps_);
  maplab_pointcloud.writeToFile(normalizedDirectoryPath(processed_submaps_));
}

std::string VoxgraphConverterNode::normalizedDirectoryPath(const uint32_t nr) {
  std::string directory = FLAGS_PLY_directory;
  directory.erase(directory.find_last_not_of("/") + 1);

  return directory + "/" + std::to_string(nr) + ".ply";
}

} // namespace maplab
