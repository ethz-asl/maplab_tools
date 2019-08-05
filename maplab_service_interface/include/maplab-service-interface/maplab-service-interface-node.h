#ifndef MAPLAB_SERVICE_INTERFACE_NODE_H_
#define MAPLAB_SERVICE_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>
#include <std_srvs/Empty.h>

#include <maplab-service-interface/service-type.h>
#include <maplab-service-interface/service-handler.h>

#include <vector>
#include <atomic>
#include <functional>

namespace maplab {

class MaplabServiceInterfaceNode {
  public:
    explicit MaplabServiceInterfaceNode(ros::NodeHandle& nh, 
        const ros::NodeHandle& nh_private);

    bool run();
    void shutdown();
    std::atomic<bool>& shouldExit();
    std::string printStatistics() const;
  private:
    template <typename FuncType>
    void setupServiceCalls(const std::string& service_topic, 
                           FuncType func);

    bool onSanityServiceCall
        (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& resp);

    bool onRunMaplabConsoleCall
        (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& resp);

    bool onFetchAllMaps
        (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& resp);

    bool onOptimizeMapsAlone
        (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& resp);

    bool onOptimizeMapsTogether
        (std_srvs::Empty::Request& req,
         std_srvs::Empty::Response& resp);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;
    std::vector<ros::ServiceServer> services_;

    std::atomic<bool> should_exit_;

    const std::string kSanityServiceTopic = "maplab_service_sanity";
    const std::string kRunMaplabConsoleCall = "maplab_service_run_maplab_console";
    const std::string kFetchAllMaps = "maplab_service_fetch_all_maps";
    const std::string kOptimizeMapsAlone = "maplab_service_optimize_maps_alone";
    const std::string kOptimizeMapsTogether = "maplab_service_optimize_maps_together";

    std::size_t sanity_service_count_ = 0u;
    std::size_t run_maplab_console_call_count_ = 0u;
    std::size_t fetch_all_maps_count_ = 0u;
    std::size_t optimize_maps_alone_count_ = 0u;
    std::size_t optimize_maps_together_count_ = 0u;

    ServiceHandler service_handler_;
};

template <typename FuncType>
void MaplabServiceInterfaceNode::setupServiceCalls(
    const std::string& service_topic, FuncType func) {
  VLOG(1) << "Registering service: " << service_topic;
  // Register the service callback.
  boost::function<bool(
    std_srvs::Empty::Request&, std_srvs::Empty::Response&)> callback =
    boost::bind(func, this, _1, _2);
  services_.emplace_back(nh_.advertiseService(service_topic, callback));
}

} // namespace maplab

#endif
