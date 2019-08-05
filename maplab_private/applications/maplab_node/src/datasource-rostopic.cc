#include "maplab-node/datasource-rostopic.h"

#include <string>

#include <aslam/common/time.h>
#include <boost/bind.hpp>
#include <map-resources/resource-conversion.h>
#include <maplab-common/accessors.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensors/lidar.h>
#include <vio-common/rostopic-settings.h>

#include "maplab-node/ros-helpers.h"

DECLARE_bool(zero_initial_timestamps);

namespace maplab {

DataSourceRostopic::DataSourceRostopic(
    const vio_common::RosTopicSettings& ros_topics)
    : shutdown_requested_(false),
      ros_topics_(ros_topics),
      image_transport_(node_handle_) {}

DataSourceRostopic::~DataSourceRostopic() {}

void DataSourceRostopic::startStreaming() {
  registerSubscribers(ros_topics_);
}

void DataSourceRostopic::shutdown() {
  shutdown_requested_ = true;
}

void DataSourceRostopic::registerSubscribers(
    const vio_common::RosTopicSettings& ros_topics) {
  // Camera subscriber.
  const size_t num_cameras = ros_topics.camera_topic_cam_index_map.size();
  sub_images_.reserve(num_cameras);

  for (const std::pair<std::string, size_t>& topic_camidx :
       ros_topics.camera_topic_cam_index_map) {
    CHECK(!topic_camidx.first.empty()) << "Camera " << topic_camidx.second
                                       << " is subscribed to an empty topic!";

    boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
        boost::bind(
            &DataSourceRostopic::imageCallback, this, _1, topic_camidx.second);

    constexpr size_t kRosSubscriberQueueSizeImage = 20u;
    image_transport::Subscriber image_sub = image_transport_.subscribe(
        topic_camidx.first, kRosSubscriberQueueSizeImage, image_callback);
    sub_images_.push_back(image_sub);

    VLOG(1) << "[MaplabNode-DataSource] Camera " << topic_camidx.second
            << " is subscribed to topic: '" << topic_camidx.first << "'";
  }

  // IMU subscriber.
  CHECK(!ros_topics.imu_topic.empty())
      << "IMU is subscribed to an empty topic!";
  constexpr size_t kRosSubscriberQueueSizeImu = 1000u;
  boost::function<void(const sensor_msgs::ImuConstPtr&)> imu_callback =
      boost::bind(&DataSourceRostopic::imuMeasurementCallback, this, _1);
  sub_imu_ = node_handle_.subscribe(
      ros_topics.imu_topic, kRosSubscriberQueueSizeImu, imu_callback);

  VLOG(1) << "[MaplabNode-DataSource] IMU is subscribed to topic: '"
          << ros_topics.imu_topic << "'";

  // Lidar subscriber.
  for (const std::pair<std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.lidar_topic_sensor_id_map) {
    CHECK(topic_sensorid.second.isValid())
        << "The ROS-topic to Lidar sensor id association contains an invalid "
        << "sensor id! topic: " << topic_sensorid.first;
    CHECK(!topic_sensorid.first.empty())
        << "Lidar(" << topic_sensorid.second
        << ") is subscribed to an empty topic!";
    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>
        lidar_callback = boost::bind(
            &DataSourceRostopic::lidarMeasurementCallback, this, _1,
            topic_sensorid.second);
    constexpr size_t kRosSubscriberQueueSizeLidar = 20u;

    sub_lidars_.emplace_back(node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeLidar, lidar_callback));

    VLOG(1) << "[MaplabNode-DataSource] Lidar(" << topic_sensorid.second
            << ") is subscribed to topic: '" << topic_sensorid.first << "'";
  }

  // Odometry subscriber.
  if (!ros_topics.odometry_6dof_topic.empty()) {
    constexpr size_t kRosSubscriberQueueSizeOdometry = 1000u;
    boost::function<void(const maplab_msgs::OdometryWithImuBiasesConstPtr&)>
        odometry_callback = boost::bind(
            &DataSourceRostopic::odometryEstimateCallback, this, _1);
    sub_odom_ = node_handle_.subscribe(
        ros_topics.odometry_6dof_topic, kRosSubscriberQueueSizeOdometry,
        odometry_callback);

    VLOG(1)
        << "[MaplabNode-DataSource] External odometry is subscribed to topic: '"
        << ros_topics.odometry_6dof_topic << "'";
  }
 
  // Artifact subscriber.
  for (const std::pair<std::string, aslam::SensorId>& topic_sensorid :
       ros_topics.artifact_topic_sensor_id_map) {
    CHECK(topic_sensorid.second.isValid())
        << "The ROS-topic to artifact sensor id association contains an invalid"
        << " sensor id! topic: " << topic_sensorid.first;
    CHECK(!topic_sensorid.first.empty())
        << "Artifact sensor(" << topic_sensorid.second
        << ") is subscribed to an empty topic!";
    boost::function<void(const 
        bayesian_artifact_filter_msgs::filtered_artConstPtr&)>
        artifact_callback = boost::bind(
            &DataSourceRostopic::artifactMeasurementCallback, this, _1,
            topic_sensorid.second);
    constexpr size_t kRosSubscriberQueueSizeArtifact = 10u;
    sub_artifact_.emplace_back(node_handle_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeArtifact,
        artifact_callback));

    VLOG(1)
        << "[MaplabNode-DataSource] Artifact detection is subscribed to topic: '"
        << topic_sensorid.first << "'";
  }
}

void DataSourceRostopic::imageCallback(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
  if (shutdown_requested_) {
    return;
  }

  vio::ImageMeasurement::Ptr image_measurement =
      convertRosImageToMaplabImage(image_message, camera_idx);

  // Apply the IMU to camera time shift.
  if (FLAGS_imu_to_camera_time_offset_ns != 0) {
    image_measurement->timestamp += FLAGS_imu_to_camera_time_offset_ns;
  }

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(image_measurement->timestamp))) {
    invokeImageCallbacks(image_measurement);
  }
}

void DataSourceRostopic::imuMeasurementCallback(
    const sensor_msgs::ImuConstPtr& msg) {
  if (shutdown_requested_) {
    return;
  }

  vio::ImuMeasurement::Ptr imu_measurement = convertRosImuToMaplabImu(msg);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(imu_measurement->timestamp))) {
    invokeImuCallbacks(imu_measurement);
  }
}

void DataSourceRostopic::lidarMeasurementCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  vi_map::RosLidarMeasurement::Ptr lidar_measurement =
      convertRosCloudToMaplabCloud(msg, sensor_id);
  CHECK(lidar_measurement);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(
          lidar_measurement->getTimestampNanosecondsMutable())) {
    invokeLidarCallbacks(lidar_measurement);
  }
}

void DataSourceRostopic::artifactMeasurementCallback(
    const bayesian_artifact_filter_msgs::filtered_artConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  vi_map::ArtifactMeasurement::Ptr detection = 
    convertToArtifactMeasurement(msg, sensor_id);
  CHECK(detection);
  invokeArtifactCallbacks(detection);
}

void DataSourceRostopic::odometryEstimateCallback(
    const maplab_msgs::OdometryWithImuBiasesConstPtr& msg) {
  CHECK(msg);
  if (shutdown_requested_) {
    return;
  }

  maplab::OdometryEstimate::Ptr odometry_measurement =
      convertRosOdometryMsgToOdometryEstimate(msg);
  CHECK(odometry_measurement);

  // Shift timestamps to start at 0.
  if (!FLAGS_zero_initial_timestamps ||
      shiftByFirstTimestamp(&(odometry_measurement->timestamp_ns))) {
    invokeOdometryCallbacks(odometry_measurement);
  }
}  // namespace maplab

}  // namespace maplab