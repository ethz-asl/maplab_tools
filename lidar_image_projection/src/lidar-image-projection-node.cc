#include "lidar-image-projection/lidar-image-projection-node.h"

#include <vio-common/rostopic-settings.h>
#include <vi-map/sensor-utils.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include <boost/bind.hpp>

#include <sstream>
#include <chrono>

DEFINE_string(
   sensor_calibration_file, "",
   "Yaml file with all the sensor calibrations. Determines which sensors are "
   "used by camera info publisher.");
DEFINE_string(
   selected_camera_id, "",
   "Defines the used camera id.");

namespace maplab {

LidarImageProjection::LidarImageProjection(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), 
    image_transport_(nh),                                                       
    sensor_manager_(new vi_map::SensorManager()),
		processed_counter_(0u) {
  if (FLAGS_sensor_calibration_file.empty()) {
    LOG(FATAL) << "[LidarImageProjection] No sensors YAML provided!";
  }
  LOG(INFO) 
		<< "[LidarImageProjection] Initializing camera info publisher...";
  if (!sensor_manager_->deserializeFromFile(FLAGS_sensor_calibration_file)) {
     LOG(FATAL) 
			 << "[LidarImageProjection] " 
			 << "Failed to read the sensor calibration from '"
			 << FLAGS_sensor_calibration_file << "'!";
  }
  CHECK(sensor_manager_);
  if (!initializeServicesAndSubscribers()) {
     LOG(FATAL) << "[MaplabCameraInfoPublisher] " 
       << "Failed initialize subscribers and services.";
  }
}

bool LidarImageProjection::initializeServicesAndSubscribers() {
  // First initialize the ncamera.
  if (!initializeNCamera()) {
    LOG(FATAL) << "[LidarImageProjection] " 
      << "Failed to initialize the ncamera pointer.";
  }
  CHECK(ncamera_rig_);

  // Setup image subscribers and info publishers.
  vio_common::RosTopicSettings ros_topics(*sensor_manager_);
  const uint8_t num_cameras = ros_topics.camera_topic_cam_index_map.size();
  if (num_cameras == 0) {
    return false;
  }

  aslam::SensorId id;
  id.fromHexString(FLAGS_selected_camera_id);
  CHECK(id.isValid());

  for (const std::pair<std::string, uint8_t>& topic_camidx :
        ros_topics.camera_topic_cam_index_map) {
    // Setup subscriber.
    CHECK(!topic_camidx.first.empty()) << "Camera " << topic_camidx.second
                                       << " is subscribed to an empty topic!";

    const aslam::Camera& camera = ncamera_rig_->getCamera(topic_camidx.second);
    if (id == camera.getId()) continue;

    boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
      boost::bind(&LidarImageProjection::imageCallback,
      this, _1, topic_camidx.second);

    constexpr size_t kRosSubscriberQueueSizeImage = 20u;
    sub_images_ = image_transport_.subscribe(
        topic_camidx.first, kRosSubscriberQueueSizeImage, image_callback);
    VLOG(1) << "[LidarImageProjection] Camera " << topic_camidx.second
            << " is subscribed to topic: '" << topic_camidx.first << "'";
  }

  return true;
}

bool LidarImageProjection::initializeNCamera() {
  CHECK(sensor_manager_);
  ncamera_rig_ =
      vi_map::getSelectedNCamera(*sensor_manager_);
  return ncamera_rig_ != nullptr;
}


bool LidarImageProjection::run() {
  LOG(INFO) << "[MaplabCameraInfoPublisher] Starting...";
  spinner_.start();
  return true;
}

void LidarImageProjection::shutdown(){}

std::atomic<bool>& LidarImageProjection::shouldExit() {
  return should_exit_;
}

std::string LidarImageProjection::printStatistics() const {
  std::stringstream ss;
  ss << "[LidarImageProjection]  Statistics \n";
	ss << "\t processed: " << processed_counter_ << " images\n";
	if (processed_counter_ > 0)
		ss << "\t Average processing time: " 
			<< total_processing_time_ms_ / processed_counter_ << "ms \n";
  return ss.str();
}

void LidarImageProjection::imageCallback(
    const sensor_msgs::ImageConstPtr &image, 
    std::size_t camera_idx) {
  VLOG(1) << "received image";
}

} // namespace maplab
