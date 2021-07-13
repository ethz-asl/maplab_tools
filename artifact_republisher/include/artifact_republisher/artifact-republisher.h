#ifndef ARTIFACT_REPUBLISHER_NODE_H_
#define ARTIFACT_REPUBLISHER_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>
#include <vi-map/sensor-manager.h>
#include <maplab_msgs/filtered_art.h>

#include <vector>
#include <atomic>
#include <map>
#include <functional>
#include <memory>

namespace maplab {

class ArtifactRepublisher {
  public:
    explicit ArtifactRepublisher(ros::NodeHandle& nh, 
        const ros::NodeHandle& nh_private);

    bool run();
    void shutdown();
    std::atomic<bool>& shouldExit();
    std::string printStatistics() const;

  private:
    bool initializeSubscribers();
    void artifact_callback(const maplab_msgs::filtered_artConstPtr& artifact, 
         const std::string& robot_name);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::atomic<bool> should_exit_;
    ros::AsyncSpinner spinner_;
		double processed_counter_;

    ros::Subscriber art_sub_;
    ros::Publisher art_pub_;
};

} // namespace maplab

#endif // ARTIFACT_REPUBLISHER_NODE_H_
