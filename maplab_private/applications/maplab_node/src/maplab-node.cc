#include "maplab-node/maplab-node.h"

#include <atomic>
#include <memory>
#include <signal.h>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <aslam/cameras/ncamera.h>
#include <localization-summary-map/localization-summary-map-creation.h>
#include <localization-summary-map/localization-summary-map.h>
#include <maplab-common/sigint-breaker.h>
#include <maplab-common/threading-helpers.h>
#include <message-flow/message-dispatcher-fifo.h>
#include <message-flow/message-flow.h>
#include <message-flow/message-topic-registration.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <vi-map/sensor-utils.h>

#include <vi-map/vi-map-serialization.h>
#include <vio-common/vio-types.h>

#include "maplab-node/data-publisher-flow.h"
#include "maplab-node/datasource-flow.h"
#include "maplab-node/feature-tracking-flow.h"
#include "maplab-node/synchronizer-flow.h"
#include "maplab-node/visual-localizer-flow.h"

DEFINE_bool(
    run_map_builder, true,
    "When set to false, the map builder will be deactivated and no map will be "
    "built.");

DEFINE_int64(
    visual_localization_enable_visualization, true,
    "Enable visualization of the visual localization observations and inliers "
    "as ROS message.");

namespace maplab {
MaplabNode::MaplabNode(
    const std::string& sensor_calibration_file,
    const std::string& save_map_folder, message_flow::MessageFlow* const flow)
    : message_flow_(CHECK_NOTNULL(flow)),
      is_datasource_exhausted_(false),
      is_running_(false),
      sensor_manager_(new vi_map::SensorManager()) {
  // === SENSORS ===
  if (!sensor_manager_->deserializeFromFile(sensor_calibration_file)) {
    LOG(FATAL) << "[MaplabNode] Failed to read the sensor calibration from '"
               << sensor_calibration_file << "'!";
  }
  CHECK(sensor_manager_);

  // === SYNCHRONIZER ===
  synchronizer_flow_.reset(new SynchronizerFlow(*sensor_manager_));
  synchronizer_flow_->attachToMessageFlow(message_flow_);

  // === DATA SOURCE ===
  datasource_flow_.reset(new DataSourceFlow(*sensor_manager_));
  datasource_flow_->attachToMessageFlow(message_flow_);
  // Subscribe to end of days signal from the datasource.
  datasource_flow_->registerEndOfDataCallback(
      [&]() { is_datasource_exhausted_.store(true); });

  // === DATA PUBLISHER ====
  data_publisher_flow_.reset(new DataPublisherFlow(*sensor_manager_));
  data_publisher_flow_->attachToMessageFlow(message_flow_);

  // === MAP BUILDER ====
  if (FLAGS_run_map_builder) {
    map_builder_flow_.reset(new MapBuilderFlow(
        *sensor_manager_, save_map_folder, synchronizer_flow_->T_M_B_buffer()));
    map_builder_flow_->attachToMessageFlow(message_flow_);
  }

  // === ENABLE MAPPING COMPONENTS ===
  initializeOdometrySource();
  initializeInertialMapping();
  initializeLidarMapping();
  initializeVisualMapping();

  initializeArtifactDetections();
}

MaplabNode::~MaplabNode() {
  shutdown();
}

void MaplabNode::initializeOdometrySource() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Odometry6DoF::Ptr external_odometry_sensor =
      vi_map::getSelectedOdometry6DoFSensor(*sensor_manager_);
  if (external_odometry_sensor) {
    LOG(INFO) << "[MaplabNode] External 6DoF Odometry sensor is ENABLED!";
    synchronizer_flow_->initializeOdometryData();
  } else {
    LOG(FATAL)
        << "[MaplabNode] This node currently relies on an external "
        << "odometry source and an IMU (for interpolation and forward "
        << "propagation) to build the map! Please ensure that all "
        << "sensors (including odometry) are registered in the yaml file.";
  }
}

void MaplabNode::initializeInertialMapping() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Imu::Ptr mapping_imu = vi_map::getSelectedImu(*sensor_manager_);
  if (mapping_imu) {
    synchronizer_flow_->initializeInertialData();
    LOG(INFO) << "[MaplabNode] Inertial sensor is ENABLED!";
  } else {
    LOG(FATAL)
        << "[MaplabNode] This node currently relies on an external "
        << "odometry source and an IMU (for interpolation and forward "
        << "propagation) to build the map! Please ensure that all "
        << "sensors (including odometry) are registered in the yaml file!";
  }
}

void MaplabNode::initializeVisualMapping() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  aslam::NCamera::Ptr mapping_ncamera =
      vi_map::getSelectedNCamera(*sensor_manager_);
  if (mapping_ncamera) {
    synchronizer_flow_->initializeVisualData(mapping_ncamera);
    tracker_flow_.reset(new FeatureTrackingFlow(
        mapping_ncamera, synchronizer_flow_->T_M_B_buffer()));
    tracker_flow_->attachToMessageFlow(message_flow_);
    LOG(INFO) << "[MaplabNode] Visual-inertial mapping is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] Visual-inertial mapping is DISABLED!";
  }
}

void MaplabNode::initializeLidarMapping() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";

  vi_map::Lidar::Ptr mapping_lidar = vi_map::getSelectedLidar(*sensor_manager_);
  if (mapping_lidar) {
    synchronizer_flow_->initializeLidarData();

    // TODO(LBern): initialize lidar mapping blocks here, rm imu if no imu
    // constraints are needed.

    // TODO(LBern): Currently only one lidar is supported, but there is no
    // good reason not to support N-Lidars.

    LOG(INFO) << "[MaplabNode] Lidar-inertial mapping is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] Lidar-inertial mapping is DISABLED!";
  }
}

void MaplabNode::initializeArtifactDetections() {
  CHECK(sensor_manager_);
  CHECK(synchronizer_flow_)
      << "[MaplabNode] Initialize the Synchronizer first!";
  vi_map::ArtifactDetection::Ptr detection_sensor 
    = vi_map::getSelectedArtifactDetectionSensor(*sensor_manager_);
  if (detection_sensor) {
    synchronizer_flow_->initializeArtifactDetectionData();
    LOG(INFO) << "[MaplabNode] Artifact detection is ENABLED!";
  } else {
    LOG(WARNING) << "[MaplabNode] Artifact detection is DISABLED!";
  }
}

void MaplabNode::initializeLocalizationHandler() {
  if (!localization_handler_flow) {
    localization_handler_flow.reset(new LocalizationHandlerFlow(
        *sensor_manager_, synchronizer_flow_->T_M_B_buffer()));
    localization_handler_flow->attachToMessageFlow(message_flow_);
  }
}

void MaplabNode::enableVisualLocalization(
    std::unique_ptr<summary_map::LocalizationSummaryMap> localization_map) {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Enable visual localization with prior map...";
  CHECK(!is_running_) << "Cannot configure node after it started running!";

  localizer_flow_.reset(new VisualLocalizerFlow(
      *sensor_manager_, synchronizer_flow_->T_M_B_buffer(),
      FLAGS_visual_localization_enable_visualization));
  localizer_flow_->setLocalizationMap(std::move(localization_map));
  localizer_flow_->attachToMessageFlow(message_flow_);

  initializeLocalizationHandler();
}

void MaplabNode::enableVisualLocalization() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Enable visual localization without prior map...";
  CHECK(!is_running_) << "Cannot configure node after it started running!";

  localizer_flow_.reset(new VisualLocalizerFlow(
      *sensor_manager_, synchronizer_flow_->T_M_B_buffer(),
      FLAGS_visual_localization_enable_visualization));
  localizer_flow_->attachToMessageFlow(message_flow_);

  initializeLocalizationHandler();
}

void MaplabNode::enableLidarLocalization() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Enable lidar localization without prior map...";
  CHECK(!is_running_) << "Cannot configure node after it started running!";

  // TODO(LBern): Add lidar localization initialization here, this includes
  // loading the localization map and setting up the blocks.

  LOG(FATAL) << "NOT IMPLEMENTED!";

  initializeLocalizationHandler();
}

void MaplabNode::enableOnlineMapping() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Enable online mapping...";
  CHECK(!is_running_) << "Cannot configure node after it started running!";

  LOG(FATAL) << "NOT IMPLEMENTED!";
}

void MaplabNode::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Starting...";

  CHECK(synchronizer_flow_);
  synchronizer_flow_->start();

  CHECK(!is_datasource_exhausted_.load())
      << "Cannot start the MaplabNode after the "
      << "end-of-data signal was received!";

  CHECK(datasource_flow_);
  datasource_flow_->startStreaming();
  VLOG(1) << "Starting data source...";

  is_running_ = true;
}
void MaplabNode::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Shutting down...";
  datasource_flow_->shutdown();
  VLOG(1) << "Closing data source...";
  is_running_ = false;
}

void MaplabNode::saveMapAndOptionallyOptimize(
    const std::string& path, const bool overwrite_existing_map,
    const bool process_to_localization_map) {
  std::lock_guard<std::mutex> lock(mutex_);
  LOG(INFO) << "[MaplabNode] Saving map to '" << path << "'.";

  if (map_builder_flow_) {
    map_builder_flow_->saveMapAndOptionallyOptimize(
        path, overwrite_existing_map, process_to_localization_map);
  } else {
    LOG(ERROR) << "Cannot save map, because map building has been disabled "
               << "using --run_map_builder=false";
  }
}

std::atomic<bool>& MaplabNode::isDataSourceExhausted() {
  return is_datasource_exhausted_;
}

}  // namespace maplab
