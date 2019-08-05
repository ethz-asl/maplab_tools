#ifndef MAPLAB_NODE_SYNCHRONIZER_H_
#define MAPLAB_NODE_SYNCHRONIZER_H_

#include <atomic>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <aslam/cameras/ncamera.h>
#include <aslam/pipeline/visual-npipeline.h>
#include <opencv2/core/core.hpp>
#include <vi-map/sensor-manager.h>
#include <vio-common/imu-measurements-buffer.h>
#include <vio-common/pose-lookup-buffer.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include <sensors/artifact-detection.h>

namespace maplab {

class Synchronizer {
 public:
  MAPLAB_POINTER_TYPEDEFS(Synchronizer);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Synchronizer() = delete;

  explicit Synchronizer(const vi_map::SensorManager& sensor_manager);

  ~Synchronizer();

  void initializeNCameraSynchronization(
      const aslam::NCamera::Ptr& camera_system);

  void processImuMeasurements(
      const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& timestamps_nanoseconds,
      const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements);
  void processCameraImage(
      const size_t camera_index, const cv::Mat& image, const int64_t timestamp);
  void processLidarMeasurement(
      const vi_map::RosLidarMeasurement::ConstPtr& lidar_measurement);
  void processOdometryMeasurement(const vio::ViNodeState& odometry);
  void processArtifactMeasurement(
      const vi_map::ArtifactMeasurement::ConstPtr& detection);

  // These functions make sure the synchronizer is aware that data of this type
  // is incomming and it will raise a warning if no data is received for more
  // than 5s.
  void expectOdometryData();
  void expectLidarData();
  void expectVisualData();
  void expectImuData();
  void expectArtifactDetection();

  // Release the buffered data based on the availability of the odometry pose.
  void releaseData();
  void releaseNFrameData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releaseLidarData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);
  void releaseArtifactData(
      const int64_t oldest_timestamp_ns, const int64_t newest_timestamp_ns);

  void registerSynchronizedNFrameCallback(
      const std::function<void(const vio::SynchronizedNFrame::Ptr&)>& callback);

  void registerLidarMeasurementCallback(
      const std::function<void(const vi_map::RosLidarMeasurement::ConstPtr&)>&
          callback);

  void registerArtifactMeasurementCallback(
      const std::function<void(const vi_map::ArtifactMeasurement::ConstPtr&)>&
          callback);

  void start();
  void shutdown();

  vio_common::PoseLookupBuffer& T_M_B_buffer() {
    return T_M_B_buffer_;
  }

 private:
  void checkIfMessagesAreIncomingWorker();

  const vi_map::SensorManager& sensor_manager_;

  // Buffer to store incomming odometry estimates, imu biases and raw imu
  // measurements. This Buffer is directly linked and modified in several other
  // components of the maplab node.
  vio_common::PoseLookupBuffer T_M_B_buffer_;

  // Pipeline to combine images into NFrames.
  aslam::VisualNPipeline::UniquePtr visual_pipeline_;
  // Buffer to store the assembled NFrames before they are released.
  mutable std::mutex nframe_buffer_mutex_;
  common::TemporalBuffer<aslam::VisualNFrame::Ptr> nframe_buffer_;

  // Buffer to store the lidar measurements before they are released.
  mutable std::mutex lidar_buffer_mutex_;
  common::TemporalBuffer<vi_map::RosLidarMeasurement::ConstPtr> lidar_buffer_;
  
  // Buffer to store the artifact measurements before they are released.
  mutable std::mutex artifact_buffer_mutex_;
  common::TemporalBuffer<
    vi_map::ArtifactMeasurement::ConstPtr> artifact_buffer_;

  // Number of already skipped frames.
  size_t frame_skip_counter_;
  // Timestamp of previously released NFrame, used to throttle the NFrames.
  int64_t previous_nframe_timestamp_ns_;
  // Threshold used to throttle the consecutively published NFrames.
  const int64_t min_nframe_timestamp_diff_ns_;

  // Number of already skipped lidar measurements.
  size_t lidar_skip_counter_;

  // Number of already skipped artifact detection.
  size_t artifact_skip_counter_;

  std::atomic<bool> shutdown_;
  std::condition_variable cv_shutdown_;
  std::thread check_if_messages_are_incoming_thread_;
  std::mutex mutex_check_if_messages_are_incoming_;

  // Indicates the timestamp when either the last message was received or the
  // check that messages are (still) incoming was performed last (whichever
  // happend most recently). The timers are initialized to -1, in case we do not
  // subscribe to that topic, no warnings will be shown.
  std::atomic<int64_t> time_last_imu_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_camera_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_lidar_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_odometry_message_received_or_checked_ns_;
  std::atomic<int64_t> time_last_artifact_message_received_or_checked_ns_;

  std::vector<std::function<void(const vio::SynchronizedNFrame::Ptr&)>>
      nframe_callbacks_;
  std::mutex nframe_callback_mutex_;

  std::vector<std::function<void(const vi_map::RosLidarMeasurement::ConstPtr&)>>
      lidar_callbacks_;
  std::mutex lidar_callback_mutex_;

  std::vector<std::function<void(const vi_map::ArtifactMeasurement::ConstPtr&)>>
      artifact_callbacks_;
  std::mutex artifact_callback_mutex_;
};

}  // namespace maplab

#endif  // MAPLAB_NODE_SYNCHRONIZER_H_
