#include <maplab-service-interface/maplab-service-interface-node.h>
#include <maplab-service-interface/service-handler.h>
#include <glog/logging.h>
#include <boost/bind.hpp>


namespace maplab {

MaplabServiceInterfaceNode::MaplabServiceInterfaceNode(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false) {
  LOG(INFO) << "[MaplabSeriviceInterface] Initializing service interface...";
  

//  pipeline_service_ = nh.advertiseService("maplab_pipeline_service", callback2);
  setupServiceCalls(kSanityServiceTopic, 
      &MaplabServiceInterfaceNode::onSanityServiceCall);
}

/*
void MaplabServiceInterfaceNode::setupServiceCalls(const std::string &topic,
        std::function<bool(std_srvs::Empty::Request&, 
                           std_srvs::Empty::Response&)> func) {
  // Register the service callback.
  boost::function<bool(
    std_srvs::Empty::Request&, std_srvs::Empty::Response&)> callback =
    boost::bind(func, this, _1, _2);
  services_.emplace_back(nh_.advertiseService(topic, callback));
}
    */

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

/*
bool MaplabServiceInterfaceNode::onServiceCall
    (maplab_service_interface::maplab_service::Request &req,
     maplab_service_interface::maplab_service::Response &resp) {
  const ServiceType service_type = static_cast<ServiceType>(req.type);
  resp.success = ServiceHandler::handleServiceCall(service_type);
  return resp.success;
}
*/

bool MaplabServiceInterfaceNode::onSanityServiceCall(
    std_srvs::Empty::Request&,
    std_srvs::Empty::Response&) {
  VLOG(1) << "sanity called";
  return true;
}

} // namespace maplab
