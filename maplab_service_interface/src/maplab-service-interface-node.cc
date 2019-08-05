#include <maplab-service-interface/maplab-service-interface-node.h>
#include <maplab-service-interface/service-handler.h>
#include <glog/logging.h>
#include <boost/bind.hpp>


namespace maplab {

MaplabServiceInterfaceNode::MaplabServiceInterfaceNode(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false) {
  LOG(INFO) << "[MaplabSeriviceInterface] Initializing service interface...";
  
  // Register the service callback.
  boost::function<bool(
    maplab_service_interface::maplab_service::Request&,
     maplab_service_interface::maplab_service::Response&)> callback2 =
    boost::bind(&MaplabServiceInterfaceNode::onServiceCall, this, _1, _2);

  pipeline_service_ = nh.advertiseService("maplab_pipeline_service", callback2);
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
  return "bla bla";
}

bool MaplabServiceInterfaceNode::onServiceCall
    (maplab_service_interface::maplab_service::Request &req,
     maplab_service_interface::maplab_service::Response &resp) {
  const ServiceType service_type = static_cast<ServiceType>(req.type);
  resp.success = ServiceHandler::handleServiceCall(service_type);
  return resp.success;
}

} // namespace maplab
