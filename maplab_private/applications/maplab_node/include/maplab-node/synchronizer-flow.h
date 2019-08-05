#ifndef MAPLAB_NODE_SYNCHRONIZER_FLOW_H_
#define MAPLAB_NODE_SYNCHRONIZER_FLOW_H_

#include <aslam/cameras/ncamera.h>
#include <message-flow/message-flow.h>
#include <sensors/imu.h>
#include <string>
#include <vi-map/sensor-utils.h>
#include <vio-common/vio-types.h>
#include "maplab-node/flow-topics.h"
#include "maplab-node/synchronizer.h"

static constexpr char kSubscriberNodeName[] = "Synchronizer";

namespace maplab {

class SynchronizerFlow {
 public:
  explicit SynchronizerFlow(const vi_map::SensorManager& sensor_manager)
      : sensor_manager_(sensor_manager), synchronizer_(sensor_manager) {
    delivery_method_.exclusivity_group_id =
        kExclusivityGroupIdRawSensorDataSubscribers;
  }

  ~SynchronizerFlow() {
    shutdown();
  }

  void start() {
    synchronizer_.start();
  }

  void initializeInertialData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectImuData();

    message_flow_->registerSubscriber<message_flow_topics::IMU_MEASUREMENTS>(
        kSubscriberNodeName, delivery_method_,
        [this](const vio::ImuMeasurement::ConstPtr& imu) {
          CHECK(imu);
          // TODO(schneith): This seems inefficient. Should we batch IMU
          // measurements on the datasource side?
          this->synchronizer_.processImuMeasurements(
              (Eigen::Matrix<int64_t, 1, 1>() << imu->timestamp).finished(),
              imu->imu_data);
        });
  }

  void initializeOdometryData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectOdometryData();

    message_flow_->registerSubscriber<message_flow_topics::ODOMETRY_ESTIMATES>(
        kSubscriberNodeName, delivery_method_,
        [this](const OdometryEstimate::ConstPtr& odometry_estimate) {
          CHECK(odometry_estimate);
          this->synchronizer_.processOdometryMeasurement(
              odometry_estimate->vinode);
        });
  }

  void initializeLidarData() {
    CHECK_NOTNULL(message_flow_);

    synchronizer_.expectLidarData();

    message_flow_->registerSubscriber<message_flow_topics::LIDAR_MEASUREMENTS>(
        kSubscriberNodeName, delivery_method_,
        [this](const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
          CHECK(lidar_measurement);
          this->synchronizer_.processLidarMeasurement(lidar_measurement);
        });
    synchronizer_.registerLidarMeasurementCallback(
        message_flow_->registerPublisher<
            message_flow_topics::SYNCED_LIDAR_MEASUREMENTS>());
  }

  void initializeArtifactDetectionData() {
    CHECK_NOTNULL(message_flow_);
    synchronizer_.expectArtifactDetection();

    message_flow_->registerSubscriber<message_flow_topics::ARTIFACT_DETECTION>(
        kSubscriberNodeName, delivery_method_,
        [this](const vi_map::ArtifactMeasurement::ConstPtr& detection) {
          CHECK(detection);
          this->synchronizer_.processArtifactMeasurement(detection);
        });

    synchronizer_.registerArtifactMeasurementCallback(
        message_flow_->registerPublisher<
            message_flow_topics::SYNCED_ARTIFACT_DETECTION>());
  }

  void initializeVisualData(aslam::NCamera::Ptr mapping_ncamera) {
    CHECK_NOTNULL(message_flow_);
    CHECK(mapping_ncamera);

    synchronizer_.initializeNCameraSynchronization(mapping_ncamera);
    synchronizer_.expectVisualData();

    message_flow_->registerSubscriber<message_flow_topics::IMAGE_MEASUREMENTS>(
        kSubscriberNodeName, delivery_method_,
        [this](const vio::ImageMeasurement::ConstPtr& image) {
          CHECK(image);
          this->synchronizer_.processCameraImage(
              image->camera_index, image->image, image->timestamp);
        });
    synchronizer_.registerSynchronizedNFrameCallback(
        message_flow_
            ->registerPublisher<message_flow_topics::SYNCED_NFRAMES>());
  }

  void attachToMessageFlow(message_flow::MessageFlow* const flow) {
    CHECK_NOTNULL(flow);
    message_flow_ = flow;
  }

  void shutdown() {
    synchronizer_.shutdown();
  }

  const vio_common::PoseLookupBuffer& T_M_B_buffer() {
    return this->synchronizer_.T_M_B_buffer();
  }

 private:
  const vi_map::SensorManager& sensor_manager_;

  message_flow::DeliveryOptions delivery_method_;
  message_flow::MessageFlow* message_flow_;

  Synchronizer synchronizer_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_SYNCHRONIZER_FLOW_H_
