#ifndef ARTIFACT_REPUBLISHER_NODE_H_
#define ARTIFACT_REPUBLISHER_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>
#include <vi-map/sensor-manager.h>

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

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::atomic<bool> should_exit_;
    ros::AsyncSpinner spinner_;
		double processed_counter_;
};

} // namespace maplab

#endif // ARTIFACT_REPUBLISHER_NODE_H_
