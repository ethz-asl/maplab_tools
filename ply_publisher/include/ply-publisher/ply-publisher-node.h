#ifndef PLY_PUBLISHER_NODE_H_
#define PLY_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>

#include <vector>
#include <atomic>
#include <functional>

namespace maplab {

class PlyPublisher {
  public:
    explicit PlyPublisher(ros::NodeHandle& nh, 
        const ros::NodeHandle& nh_private);

    bool run();
    void shutdown();
    std::atomic<bool>& shouldExit();
    std::string printStatistics() const;
  private:
    void readPointclouds(const std::string& dir);
    void readDirectory(const std::string& directory,
      std::vector<std::string>* files);
    //void publishPointcloud()

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;
    ros::Publisher ply_pub_;

    std::atomic<bool> should_exit_;
    uint32_t processed_plys_;
    std::string ply_directory_;
    std::string publish_topic_;
};

} // namespace maplab

#endif
