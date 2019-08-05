#include "maplab-node/datasource-rosbag.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <aslam/common/time.h>
#include <boost/bind.hpp>
#include <maplab-common/accessors.h>
#include <maplab-common/file-system-tools.h>
#include <maplab_msgs/OdometryWithImuBiases.h>
#include <sensor_msgs/PointCloud2.h>
#include <vio-common/rostopic-settings.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#pragma GCC diagnostic pop

#include <sensors/lidar.h>
#include "maplab-node/odometry-estimate.h"
#include "maplab-node/ros-helpers.h"

DEFINE_double(vio_rosbag_start_s, 0.0, "Start of the rosbag in seconds.");
DEFINE_double(vio_rosbag_end_s, 0.0, "End of the rosbag in seconds.");
DEFINE_double(
    vio_rosbag_realtime_playback_rate, 1.0,
    "Playback rate of the ROSBAG. Real-time corresponds to 1.0. "
    "This only makes sense when using offline data sources.");
DEFINE_bool(
    zero_initial_timestamps, false,
    "If set to true, the timestamps outputted by the estimator start with 0. "
    "Not zeroing the timestamps may lead to less accurate results due to "
    "rounding errors.");

namespace maplab {

DataSourceRosbag::DataSourceRosbag(
    const std::string& rosbag_path_filename,
    const vio_common::RosTopicSettings& ros_topics)
    : shutdown_requested_(false),
      all_data_streamed_(false),
      rosbag_path_filename_(rosbag_path_filename),
      ros_topics_(ros_topics) {
  initialize();
}

DataSourceRosbag::~DataSourceRosbag() {
  CHECK(bag_);
  shutdown();
  bag_->close();
}

void DataSourceRosbag::startStreaming() {
  streaming_thread_.reset(
      new std::thread(std::bind(&DataSourceRosbag::streamingWorker, this)));
}

void DataSourceRosbag::shutdown() {
  shutdown_requested_ = true;
  if (streaming_thread_ != nullptr && streaming_thread_->joinable()) {
    streaming_thread_->join();
  }
}

std::string DataSourceRosbag::getDatasetName() const {
  std::string path, filename;
  common::splitPathAndFilename(rosbag_path_filename_, &path, &filename);
  return filename;
}

void DataSourceRosbag::initialize() {
  try {
    bag_.reset(new rosbag::Bag);
    LOG(INFO) << "Start streaming data from rosbag at '"
              << rosbag_path_filename_ << "'.";
    bag_->open(rosbag_path_filename_, rosbag::bagmode::Read);
  } catch (const std::exception& ex) {  // NOLINT
    LOG(FATAL) << "Could not open the rosbag " << rosbag_path_filename_ << ": "
               << ex.what();
  }
  CHECK(bag_);

  // Open a view on all topics.
  ros_topics_.makeAbsoluteTopics();

  std::vector<std::string> all_topics;
  for (const vio_common::RosTopicSettings::CameraTopicIdxMap::value_type&
           topic_idx_pair : ros_topics_.camera_topic_cam_index_map) {
    all_topics.push_back(topic_idx_pair.first);
  }
  all_topics.push_back(ros_topics_.imu_topic);
  all_topics.push_back(ros_topics_.relative_6dof_topic);
  all_topics.push_back(ros_topics_.gps_wgs_topic);
  all_topics.push_back(ros_topics_.gps_utm_topic);

  for (const vio_common::RosTopicSettings::TopicSensorIdMap::value_type&
           topic_sensor_id : ros_topics_.lidar_topic_sensor_id_map) {
    all_topics.push_back(topic_sensor_id.first);
  }

  all_topics.push_back(ros_topics_.odometry_6dof_topic);

  for (const vio_common::RosTopicSettings::TopicSensorIdMap::value_type&
           topic_sensor_id : ros_topics_.artifact_topic_sensor_id_map) {
    all_topics.push_back(topic_sensor_id.first);
  }

  try {
    CHECK(bag_);
    if (FLAGS_vio_rosbag_end_s != 0.0 || FLAGS_vio_rosbag_start_s != 0.0) {
      // Get the start offset from the unconstrained view.
      bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(all_topics)));

      const double bag_start_time = bag_view_->getBeginTime().toSec();
      CHECK_GE(FLAGS_vio_rosbag_start_s, 0);
      const double absolute_start_time_s =
          bag_start_time + FLAGS_vio_rosbag_start_s;

      // Auto-set the end-time to the bag-length if not provided.
      double vio_rosbag_end_s = FLAGS_vio_rosbag_end_s;
      if (vio_rosbag_end_s <= 0.0) {
        vio_rosbag_end_s = bag_view_->getEndTime().toSec() - bag_start_time;
      }
      CHECK_GT(vio_rosbag_end_s, FLAGS_vio_rosbag_start_s);
      const double absolute_end_time_s = bag_start_time + vio_rosbag_end_s;

      bag_view_.reset(new rosbag::View(
          *bag_, rosbag::TopicQuery(all_topics),
          ros::Time(absolute_start_time_s), ros::Time(absolute_end_time_s)));
    } else {
      bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(all_topics)));
    }
  } catch (const std::exception& ex) {  // NOLINT
    LOG(FATAL) << "Could not open a rosbag view: " << ex.what();
  }
  CHECK(bag_view_);
}

void DataSourceRosbag::streamingWorker() {
  // Play all messages.
  CHECK(bag_view_);
  CHECK_GT(FLAGS_vio_rosbag_realtime_playback_rate, 0.0);
  const double message_wait_time_scaler =
      1.0 / FLAGS_vio_rosbag_realtime_playback_rate;

  // NOTE: the playback order corresponds to the message timestamp (=host
  // received) and not the actual timestamp in the message's header field.
  rosbag::View::iterator it_message = bag_view_->begin();
  while (it_message != bag_view_->end()) {
    if (shutdown_requested_) {
      return;
    }

    const std::chrono::high_resolution_clock::time_point time_publishing_start =
        std::chrono::high_resolution_clock::now();

    const rosbag::MessageInstance& message = *it_message;
    const std::string& topic = message.getTopic();
    CHECK(!topic.empty());

    // Enqueue image messages.
    sensor_msgs::ImageConstPtr image_message =
        message.instantiate<sensor_msgs::Image>();
    if (image_message) {
      const size_t camera_idx =
          common::getChecked(ros_topics_.camera_topic_cam_index_map, topic);
      vio::ImageMeasurement::Ptr image_measurement =
          convertRosImageToMaplabImage(image_message, camera_idx);
      CHECK(image_measurement);

      // Apply the IMU to camera time shift.
      if (FLAGS_imu_to_camera_time_offset_ns != 0) {
        image_measurement->timestamp += FLAGS_imu_to_camera_time_offset_ns;
      }

      // Shift timestamps to start at 0.
      if (!FLAGS_zero_initial_timestamps ||
          shiftByFirstTimestamp(&(image_measurement->timestamp))) {
        VLOG(3) << "Publish Image measurement...";
        invokeImageCallbacks(image_measurement);
      }
    }

    // Enqueue IMU messages.
    if (topic == ros_topics_.imu_topic) {
      sensor_msgs::ImuConstPtr imu_msg =
          message.instantiate<sensor_msgs::Imu>();
      CHECK(imu_msg);

      vio::ImuMeasurement::Ptr imu_measurement =
          convertRosImuToMaplabImu(imu_msg);
      CHECK(imu_measurement);

      // Shift timestamps to start at 0.
      if (!FLAGS_zero_initial_timestamps ||
          shiftByFirstTimestamp(&(imu_measurement->timestamp))) {
        VLOG(3) << "Publish IMU measurement...";
        invokeImuCallbacks(imu_measurement);
      }
    }

    if (topic == ros_topics_.odometry_6dof_topic) {
      maplab_msgs::OdometryWithImuBiasesConstPtr odometry_msg =
          message.instantiate<maplab_msgs::OdometryWithImuBiases>();
      CHECK(odometry_msg);

      maplab::OdometryEstimate::Ptr odometry_estimate =
          convertRosOdometryMsgToOdometryEstimate(odometry_msg);
      CHECK(odometry_estimate);

      // Shift timestamps to start at 0.
      if (!FLAGS_zero_initial_timestamps ||
          shiftByFirstTimestamp(&(odometry_estimate->timestamp_ns))) {
        VLOG(3) << "Publish Odometry estimate...";
        invokeOdometryCallbacks(odometry_estimate);
      }
    }

    // Enqueue lidar messages.
    sensor_msgs::PointCloud2ConstPtr lidar_msgs =
        message.instantiate<sensor_msgs::PointCloud2>();
    if (lidar_msgs) {
      const aslam::SensorId sensor_id =
          common::getChecked(ros_topics_.lidar_topic_sensor_id_map, topic);
      vi_map::RosLidarMeasurement::Ptr lidar_measurement =
          convertRosCloudToMaplabCloud(lidar_msgs, sensor_id);

      // Shift timestamps to start at 0.
      if (!FLAGS_zero_initial_timestamps ||
          shiftByFirstTimestamp(
              lidar_measurement->getTimestampNanosecondsMutable())) {
        VLOG(3) << "Publish Lidar measurement...";
        invokeLidarCallbacks(lidar_measurement);
      }
    }

    // Wait for the time between messages.
    rosbag::View::iterator it_next_message = it_message;
    ++it_next_message;
    if (it_next_message != bag_view_->end()) {
      std::chrono::duration<double> delta_t_k_kp1(
          it_next_message->getTime().toSec() - message.getTime().toSec());
      delta_t_k_kp1 *= message_wait_time_scaler;
      std::this_thread::sleep_until(time_publishing_start + delta_t_k_kp1);
    }
    ++it_message;
  }
  LOG(INFO) << "Rosbag playback finished!";
  all_data_streamed_ = true;
  invokeEndOfDataCallbacks();
  return;
}

}  // namespace maplab
