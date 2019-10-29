#ifndef VOXGRAPH_CONVERTER_NODE_H_
#define VOXGRAPH_CONVERTER_NODE_H_

#include <voxgraph_msgs/MapSurface.h>

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
    void submapCallback(const voxgraph_msgs::MapSurfaceConstPtr& msg);
    //void republishSubmap(const )

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;
    ros::Subscriber submap_sub_;
    ros::Publisher pcl_pub_;

    std::atomic<bool> should_exit_;
};

} // namespace maplab

#endif
