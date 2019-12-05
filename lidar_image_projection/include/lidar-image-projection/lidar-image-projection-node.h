#ifndef MAPLAB_LIDAR_IMAGE_PROJECTION_NODE_H_
#define MAPLAB_LIDAR_IMAGE_PROJECTION_NODE_H_

#include <ros/ros.h>
#include <glog/logging.h>
#include <std_srvs/Empty.h>
#include <vi-map/sensor-manager.h>
#include <image_transport/image_transport.h>

#include <vector>
#include <atomic>
#include <map>
#include <functional>
#include <memory>

namespace maplab {

class LidarImageProjection {
  public:
    explicit LidarImageProjection(ros::NodeHandle& nh, 
        const ros::NodeHandle& nh_private);

    bool run();
    void shutdown();
    std::atomic<bool>& shouldExit();
    std::string printStatistics() const;

  private:
    bool initializeServicesAndSubscribers();
    bool initializeNCamera();

    void imageCallback(const sensor_msgs::ImageConstPtr &image, 
        std::size_t camera_idx);

    void lidarMeasurementCallback(
       const sensor_msgs::PointCloud2ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::AsyncSpinner spinner_;
    std::atomic<bool> should_exit_;

    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber sub_images_;
    ros::Subscriber ros_subs_;

    std::unique_ptr<vi_map::SensorManager> sensor_manager_;
    aslam::NCamera::Ptr ncamera_rig_;
		double processed_counter_;
		double total_processing_time_ms_;
};

} // namespace maplab

#endif // MAPLAB_LIDAR_IMAGE_PROJECTION_NODE_H_
