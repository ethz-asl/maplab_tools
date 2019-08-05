#ifndef MAPLAB_SERVICE_INTERFACE_NODE_H_
#define MAPLAB_SERVICE_INTERFACE_NODE_H_

#include <ros/ros.h>
#include <maplab_service_interface/maplab_service.h>
#include <maplab-service-interface/service-type.h>

#include <atomic>

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
    bool onServiceCall
        (maplab_service_interface::maplab_service::Request &req,
         maplab_service_interface::maplab_service::Response &resp);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;
    ros::ServiceServer pipeline_service_;

    std::atomic<bool> should_exit_;
};

} // namespace maplab

#endif
