#ifndef VOXGRAPH_CONVERTER_NODE_H_
#define VOXGRAPH_CONVERTER_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>
#include <std_srvs/Empty.h>

#include <vector>
#include <atomic>
#include <functional>

namespace maplab {

class VoxgraphConverterNode {
  public:
    explicit VoxgraphConverterNode(ros::NodeHandle& nh, 
        const ros::NodeHandle& nh_private);

    bool run();
    void shutdown();
    std::atomic<bool>& shouldExit();
    std::string printStatistics() const;
  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;

    std::atomic<bool> should_exit_;
};

} // namespace maplab

#endif
