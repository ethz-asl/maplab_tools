#ifndef MAPLAB_NODE_ROS_HELPERS_H_
#define MAPLAB_NODE_ROS_HELPERS_H_

#include <atomic>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#pragma GCC diagnostic pop

#include <eigen_conversions/eigen_msg.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <bayesian_artifact_filter_msgs/filtered_art.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensors/lidar.h>
#include <sensors/artifact-detection.h>
#include <vio-common/vio-types.h>

#include "maplab-node/odometry-estimate.h"

namespace maplab {

constexpr int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return aslam::time::seconds(static_cast<int64_t>(rostime.sec)) +
         static_cast<int64_t>(rostime.nsec);
}

vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
    const sensor_msgs::ImuConstPtr& imu_msg);

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx);

vi_map::RosLidarMeasurement::Ptr convertRosCloudToMaplabCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const aslam::SensorId& sensor_id);

maplab::OdometryEstimate::Ptr convertRosOdometryMsgToOdometryEstimate(
    const maplab_msgs::OdometryWithImuBiasesConstPtr& msg);
vi_map::ArtifactMeasurement::Ptr convertToArtifactMeasurement(
    const bayesian_artifact_filter_msgs::filtered_artConstPtr& msg,
    const aslam::SensorId& sensor_id);

}  // namespace maplab

#endif  // MAPLAB_NODE_ROS_HELPERS_H_
