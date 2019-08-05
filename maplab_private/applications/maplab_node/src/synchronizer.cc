#include "maplab-node/synchronizer.h"

#include <aslam/pipeline/visual-pipeline-null.h>
#include <maplab-common/conversions.h>

DEFINE_int64(
    vio_nframe_sync_tolerance_ns, 500000,
    "Tolerance of the timestamps of two images to consider them as "
    "part of a single n-frame [ns].");
DEFINE_double(
    vio_nframe_sync_max_output_frequency_hz, 10.0,
    "Maximum output frequency of the synchronized NFrame structures "
    "from the synchronizer.");
DEFINE_int32(
    vio_nframe_sync_max_queue_size, 50,
    "Maximum queue size of the synchronization pipeline trying to match images "
    "into NFrames.");

DEFINE_int64(
    odometry_buffer_history_ns, aslam::time::seconds(30u),
    "History length of the buffered external 6DOF odometry measurements.");
DEFINE_int64(
    odometry_buffer_max_forward_propagation_ns, aslam::time::milliseconds(500),
    "Determines the maximum duration the odometry buffer can "
    "forward-propagate using the IMU.");

namespace maplab {

Synchronizer::Synchronizer(const vi_map::SensorManager& sensor_manager)
    : sensor_manager_(sensor_manager),
      T_M_B_buffer_(
          FLAGS_odometry_buffer_history_ns,
          FLAGS_odometry_buffer_max_forward_propagation_ns),
      frame_skip_counter_(0u),
      previous_nframe_timestamp_ns_(aslam::time::getInvalidTime()),
      min_nframe_timestamp_diff_ns_(
          kSecondsToNanoSeconds /
          FLAGS_vio_nframe_sync_max_output_frequency_hz),
      lidar_skip_counter_(0u),
      shutdown_(false),
      time_last_imu_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_camera_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_lidar_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()),
      time_last_odometry_message_received_or_checked_ns_(
          aslam::time::getInvalidTime()) {
  CHECK_GT(FLAGS_vio_nframe_sync_max_output_frequency_hz, 0.);
}

Synchronizer::~Synchronizer() {
  if (!shutdown_.load()) {
    shutdown();
  }
}

void Synchronizer::start() {
  check_if_messages_are_incoming_thread_ =
      std::thread(&Synchronizer::checkIfMessagesAreIncomingWorker, this);
}

void Synchronizer::initializeNCameraSynchronization(
    const aslam::NCamera::Ptr& camera_system) {
  CHECK(camera_system);
  CHECK(!visual_pipeline_) << "[MaplabNode-Synchronizer] NCamera "
                              "synchronization already initialized!";

  // Initialize the pipeline.
  static constexpr bool kCopyImages = false;
  std::vector<aslam::VisualPipeline::Ptr> mono_pipelines;
  for (size_t camera_idx = 0u; camera_idx < camera_system->getNumCameras();
       ++camera_idx) {
    mono_pipelines.emplace_back(new aslam::NullVisualPipeline(
        camera_system->getCameraShared(camera_idx), kCopyImages));
  }

  const int kNFrameToleranceNs = FLAGS_vio_nframe_sync_tolerance_ns;
  constexpr size_t kNumThreads = 1u;
  visual_pipeline_.reset(new aslam::VisualNPipeline(
      kNumThreads, mono_pipelines, camera_system, camera_system,
      kNFrameToleranceNs));
}

void Synchronizer::processCameraImage(
    const size_t camera_index, const cv::Mat& image, const int64_t timestamp) {
  CHECK(visual_pipeline_) << "[MaplabNode-Synchronizer] The visual pipeline, "
                             "which turns individual images "
                          << "into NFrames, has not been initialized yet!";

  time_last_camera_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  if (!visual_pipeline_->processImageBlockingIfFull(
          camera_index, image, timestamp,
          FLAGS_vio_nframe_sync_max_queue_size)) {
    LOG(ERROR)
        << "[MaplabNode-Synchronizer] Failed to process an image of camera "
        << camera_index << " into an NFrame at time " << timestamp << "ns!";
    shutdown();
  }

  // Put all visual frames that are ready into the buffer.
  {
    std::lock_guard<std::mutex> lock(nframe_buffer_mutex_);
    aslam::VisualNFrame::Ptr next_nframe = visual_pipeline_->getNext();
    while (next_nframe) {
      nframe_buffer_.addValue(
          next_nframe->getMinTimestampNanoseconds(), next_nframe);
      next_nframe = visual_pipeline_->getNext();
    }
  }
}

void Synchronizer::processLidarMeasurement(
    const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement) {
  CHECK(lidar_measurement);
  time_last_lidar_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  {
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex_);
    lidar_buffer_.addValue(
        lidar_measurement->getTimestampNanoseconds(), lidar_measurement);
  }
}

void Synchronizer::processArtifactMeasurement(
    const vi_map::ArtifactMeasurement::ConstPtr& detection) {
  VLOG(1) << "[Synchronizer] << artifact arrived for processing";
  CHECK(detection);
  time_last_artifact_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  {
    std::lock_guard<std::mutex> lock(artifact_buffer_mutex_);
    artifact_buffer_.addValue(
        detection->getTimestampNanoseconds(), detection);
  }
}

void Synchronizer::processImuMeasurements(
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements) {
  time_last_imu_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  T_M_B_buffer_.imu_buffer_mutable().addMeasurements(
      timestamps_nanoseconds, imu_measurements);
}

void Synchronizer::processOdometryMeasurement(
    const vio::ViNodeState& odometry) {
  time_last_odometry_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());

  T_M_B_buffer_.bufferOdometryEstimate(odometry);

  releaseData();
}

void Synchronizer::releaseNFrameData(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  std::vector<aslam::VisualNFrame::Ptr> nframes;
  {
    std::lock_guard<std::mutex> lock(nframe_buffer_mutex_);

    // Drop these frames, since there is no odometry data anymore for them.
    const size_t dropped_nframes =
        nframe_buffer_.removeItemsBefore(oldest_timestamp_ns);
    LOG_IF(WARNING, dropped_nframes != 0u)
        << "[MaplabNode-Synchronizer] Could not find an odometry "
        << "transformation for " << dropped_nframes << " nframes "
        << "because it was already dropped from the buffer! "
        << "This might be okay during initialization.";

    frame_skip_counter_ += dropped_nframes;

    nframe_buffer_.extractItemsBeforeIncluding(newest_timestamp_ns, &nframes);
  }

  for (const aslam::VisualNFrame::Ptr nframe : nframes) {
    CHECK(nframe);
    vio::SynchronizedNFrame::Ptr new_nframe_measurement(
        new vio::SynchronizedNFrame);
    new_nframe_measurement->nframe = nframe;
    const int64_t current_frame_timestamp_ns =
        nframe->getMinTimestampNanoseconds();

    // Throttle the output rate of VisualNFrames to reduce the rate of which
    // the following nodes are running (e.g. tracker).
    if (aslam::time::isValidTime(previous_nframe_timestamp_ns_)) {
      if (current_frame_timestamp_ns - previous_nframe_timestamp_ns_ <
          min_nframe_timestamp_diff_ns_) {
        ++frame_skip_counter_;
        continue;
      }
    }
    previous_nframe_timestamp_ns_ = current_frame_timestamp_ns;

    {
      std::lock_guard<std::mutex> callback_lock(nframe_callback_mutex_);
      for (const std::function<void(const vio::SynchronizedNFrame::Ptr&)>&
               callback : nframe_callbacks_) {
        callback(new_nframe_measurement);
      }
    }
  }
}

void Synchronizer::releaseLidarData(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  std::vector<vi_map::RosLidarMeasurement::ConstPtr> lidar_measurements;
  {
    std::lock_guard<std::mutex> lock(lidar_buffer_mutex_);

    // Drop these lidar measurements, since there is no odometry data anymore
    // for them.
    const size_t dropped_lidar_measurements =
        lidar_buffer_.removeItemsBefore(oldest_timestamp_ns);
    LOG_IF(WARNING, dropped_lidar_measurements != 0u)
        << "[MaplabNode-Synchronizer] Could not find an odometry "
        << "transformation for " << dropped_lidar_measurements
        << " lidar measurements because it was already dropped from the "
        << "buffer! This might be okay during initialization.";

    lidar_skip_counter_ += dropped_lidar_measurements;

    lidar_buffer_.extractItemsBeforeIncluding(
        newest_timestamp_ns, &lidar_measurements);
  }

  for (const vi_map::RosLidarMeasurement::ConstPtr lidar_measurement :
       lidar_measurements) {
    CHECK(lidar_measurement);
    std::lock_guard<std::mutex> callback_lock(lidar_callback_mutex_);
    for (const std::function<void(
             const vi_map::RosLidarMeasurement::ConstPtr&)>& callback :
         lidar_callbacks_) {
      callback(lidar_measurement);
    }
  }
}

void Synchronizer::releaseArtifactData(
    const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns) {
  CHECK_GE(newest_timestamp_ns, oldest_timestamp_ns);
  std::vector<vi_map::ArtifactMeasurement::ConstPtr> detection_measurements;
  {
    std::lock_guard<std::mutex> lock(artifact_buffer_mutex_);

    // Drop these artifact detection measurements, since there 
    // is no odometry data anymore for them.
    const size_t dropped_artifact_measurements =
        artifact_buffer_.removeItemsBefore(oldest_timestamp_ns);
    LOG_IF(WARNING, dropped_artifact_measurements != 0u)
        << "[MaplabNode-Synchronizer] Could not find an odometry "
        << "transformation for " << dropped_artifact_measurements
        << " detection measurements because it was already dropped from the "
        << "buffer! This might be okay during initialization.";

    artifact_skip_counter_ += dropped_artifact_measurements;

    artifact_buffer_.extractItemsBeforeIncluding(
        newest_timestamp_ns, &detection_measurements);
  }
  for (const vi_map::ArtifactMeasurement::ConstPtr detection_measurement :
       detection_measurements) {
    CHECK(detection_measurement);
    std::lock_guard<std::mutex> callback_lock(artifact_callback_mutex_);
    for (const std::function<void(
             const vi_map::ArtifactMeasurement::ConstPtr&)>& callback :
         artifact_callbacks_) {
      callback(detection_measurement);
    }
  }
}

void Synchronizer::releaseData() {
  int64_t oldest_timestamp_ns;
  if (!T_M_B_buffer_.getOldestTimestampOfAvailablePose(&oldest_timestamp_ns)) {
    return;
  }

  int64_t newest_timestamp_ns;
  if (!T_M_B_buffer_.getNewestTimestampOfAvailablePose(&newest_timestamp_ns)) {
    return;
  }

  // Release (or drop) NFrames.
  if (aslam::time::isValidTime(
          time_last_camera_message_received_or_checked_ns_.load())) {
    releaseNFrameData(oldest_timestamp_ns, newest_timestamp_ns);
  }

  // Release (or drop) lidar measurements.
  if (aslam::time::isValidTime(
          time_last_lidar_message_received_or_checked_ns_.load())) {
    releaseLidarData(oldest_timestamp_ns, newest_timestamp_ns);
  }
  
  // Release (or drop) artifact detections.
  if (aslam::time::isValidTime(
          time_last_artifact_message_received_or_checked_ns_.load())) {
    releaseArtifactData(oldest_timestamp_ns, newest_timestamp_ns);
  }
}

void Synchronizer::registerSynchronizedNFrameCallback(
    const std::function<void(const vio::SynchronizedNFrame::Ptr&)>& callback) {
  std::lock_guard<std::mutex> lock(nframe_callback_mutex_);
  CHECK(callback);
  nframe_callbacks_.emplace_back(callback);
}

void Synchronizer::registerLidarMeasurementCallback(
    const std::function<void(const vi_map::RosLidarMeasurement::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(lidar_callback_mutex_);
  CHECK(callback);
  lidar_callbacks_.emplace_back(callback);
}

void Synchronizer::registerArtifactMeasurementCallback(
    const std::function<void(const vi_map::ArtifactMeasurement::ConstPtr&)>&
        callback) {
  std::lock_guard<std::mutex> lock(artifact_callback_mutex_);
  CHECK(callback);
  artifact_callbacks_.emplace_back(callback);
}

void Synchronizer::shutdown() {
  shutdown_.store(true);

  if (visual_pipeline_) {
    visual_pipeline_->shutdown();
  }

  T_M_B_buffer_.imu_buffer_mutable().shutdown();

  cv_shutdown_.notify_all();
  if (check_if_messages_are_incoming_thread_.joinable()) {
    check_if_messages_are_incoming_thread_.join();
  }

  LOG(INFO)
      << "[MaplabNode-Synchronizer] Shutting down. Skipped visual frames: "
      << frame_skip_counter_
      << " Skipped lidar measurements: " << lidar_skip_counter_;
}

void Synchronizer::expectOdometryData() {
  time_last_odometry_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectLidarData() {
  time_last_lidar_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectVisualData() {
  time_last_camera_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectImuData() {
  time_last_imu_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::expectArtifactDetection() {
  time_last_artifact_message_received_or_checked_ns_.store(
      aslam::time::nanoSecondsSinceEpoch());
}

void Synchronizer::checkIfMessagesAreIncomingWorker() {
  constexpr int kMaxTimeBeforeWarningS = 5;
  const int64_t kMaxTimeBeforeWarningNs =
      aslam::time::secondsToNanoSeconds(kMaxTimeBeforeWarningS);
  while (true) {
    std::unique_lock<std::mutex> lock(mutex_check_if_messages_are_incoming_);
    const bool shutdown_requested = cv_shutdown_.wait_for(
        lock, std::chrono::seconds(kMaxTimeBeforeWarningS),
        [this]() { return shutdown_.load(); });
    if (shutdown_requested) {
      return;
    }

    const int64_t current_time_ns = aslam::time::nanoSecondsSinceEpoch();

    if (time_last_imu_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_imu_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No IMU messages have been received in "
          << "the last " << kMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    if (time_last_camera_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_camera_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No camera messages have been received "
          << "in the last " << kMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    if (time_last_odometry_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_odometry_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No odometry messages have been "
          << "received in the last " << kMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }

    if (time_last_lidar_message_received_or_checked_ns_.load() !=
        aslam::time::getInvalidTime()) {
      const int64_t last_time_ns =
          time_last_lidar_message_received_or_checked_ns_.exchange(
              current_time_ns);
      LOG_IF(WARNING, current_time_ns - last_time_ns > kMaxTimeBeforeWarningNs)
          << "[MaplabNode-Synchronizer] No lidar messages have been "
          << "received in the last " << kMaxTimeBeforeWarningS
          << " seconds. Check for measurement drops or if the topic is "
          << "properly set in the sensor calibration file";
    }
  }
}
}  // namespace maplab
