#include <maplab-service-interface/maplab-service-interface-node.h>
#include <maplab-service-interface/service-handler.h>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>


namespace maplab {

MaplabServiceInterfaceNode::MaplabServiceInterfaceNode(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false) {
  LOG(INFO) << "[MaplabSeriviceInterface] Initializing service interface...";
  
  setupServiceCalls(kSanityServiceTopic, 
      &MaplabServiceInterfaceNode::onSanityServiceCall);
  setupServiceCalls(kRunMaplabConsoleCall, 
      &MaplabServiceInterfaceNode::onRunMaplabConsoleCall);
  setupServiceCalls(kFetchAllMaps, 
      &MaplabServiceInterfaceNode::onFetchAllMaps);
  setupServiceCalls(kOptimizeMapsAlone, 
      &MaplabServiceInterfaceNode::onOptimizeMapsAlone);
  setupServiceCalls(kOptimizeMapsTogether, 
      &MaplabServiceInterfaceNode::onOptimizeMapsTogether);
}

bool MaplabServiceInterfaceNode::run() {
  LOG(INFO) << "[MaplabSeriviceInterface] Starting...";
  spinner_.start();
  return true;
}

void MaplabServiceInterfaceNode::shutdown(){

}

std::atomic<bool>& MaplabServiceInterfaceNode::shouldExit() {
  return should_exit_;
}

std::string MaplabServiceInterfaceNode::printStatistics() const {
  std::stringstream ss;
  ss << "[MaplabServiceStatistics] \n";
  ss << kSanityServiceTopic << " \t \t \t " << " called " << sanity_service_count_ << " times.\n";
  ss << kRunMaplabConsoleCall << " \t " << " called " << run_maplab_console_call_count_ << " times.\n";
  ss << kFetchAllMaps << " \t \t " << " called " << fetch_all_maps_count_ << " times.\n";
  ss << kOptimizeMapsAlone << " \t " << " called " << optimize_maps_alone_count_ << " times.\n";
  ss << kOptimizeMapsTogether << " \t " << " called " << optimize_maps_together_count_ << " times.\n";
  return ss.str();
}

bool MaplabServiceInterfaceNode::onSanityServiceCall(
    std_srvs::Empty::Request&,
    std_srvs::Empty::Response&) {
  VLOG(3) << "[MaplabServiceInterfaceNode] Received service call for"
          << " the sanity service.";
  ++sanity_service_count_;
  return true;
}

bool MaplabServiceInterfaceNode::onRunMaplabConsoleCall
    (std_srvs::Empty::Request&,
     std_srvs::Empty::Response&) {
  VLOG(3) << "[MaplabServiceInterfaceNode] Received service call for"
          << " the run maplab service.";
  return true;
}

bool MaplabServiceInterfaceNode::onFetchAllMaps
    (std_srvs::Empty::Request&,
     std_srvs::Empty::Response&) {
  VLOG(3) << "[MaplabServiceInterfaceNode] Received service call for"
          << " the fetch all maps service.";
  return true;
}

bool MaplabServiceInterfaceNode::onOptimizeMapsAlone
    (std_srvs::Empty::Request&,
     std_srvs::Empty::Response&) {
  VLOG(3) << "[MaplabServiceInterfaceNode] Received service call for"
          << " the optimize maps alone service.";

  return true;
}

bool MaplabServiceInterfaceNode::onOptimizeMapsTogether
    (std_srvs::Empty::Request&,
     std_srvs::Empty::Response&) {
  VLOG(3) << "[MaplabServiceInterfaceNode] Received service call for"
          << " the optimize maps together service.";

  return true;
}

} // namespace maplab
