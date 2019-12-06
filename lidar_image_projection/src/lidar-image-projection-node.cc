#include "lidar-image-projection/lidar-image-projection-node.h"

#include <vio-common/rostopic-settings.h>
#include <vi-map/sensor-utils.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>

#include <pcl/point_cloud.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/passthrough.h>


#include <sstream>
#include <chrono>

DEFINE_string(
   sensor_calibration_file, "",
   "Yaml file with all the sensor calibrations. Determines which sensors are "
   "used by camera info publisher.");
DEFINE_string(
   selected_camera_id, "",
   "Defines the used camera id.");
DEFINE_string(
   selected_lidar_id, "",
   "Defines the used lidar id.");

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
		<< "[LidarImageProjection] Initializing...";
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

  // Setup image subscriber.
  vio_common::RosTopicSettings ros_topics(*sensor_manager_);
  const uint8_t num_cameras = ros_topics.camera_topic_cam_index_map.size();
  if (num_cameras == 0) {
    return false;
  }

  aslam::SensorId camera_id, lidar_id;
  camera_id.fromHexString(FLAGS_selected_camera_id);
  lidar_id.fromHexString(FLAGS_selected_lidar_id);

  CHECK(camera_id.isValid());
  CHECK(lidar_id.isValid());

  bool camera_found = false;
  for (const std::pair<std::string, uint8_t>& topic_camidx :
        ros_topics.camera_topic_cam_index_map) {
    // Setup subscriber.
    CHECK(!topic_camidx.first.empty()) << "Camera " << topic_camidx.second
                                       << " is subscribed to an empty topic!";

    const aslam::Camera& camera = ncamera_rig_->getCamera(topic_camidx.second);
    if (camera_id != camera.getId()) continue;

    boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
      boost::bind(&LidarImageProjection::imageCallback, this, _1);

    constexpr size_t kRosSubscriberQueueSizeImage = 20u;
    sub_images_ = image_transport_.subscribe(
        topic_camidx.first, kRosSubscriberQueueSizeImage, image_callback);
    VLOG(1) << "[LidarImageProjection] Camera " << topic_camidx.second
            << " is subscribed to topic: '" << topic_camidx.first << "'";
    T_B_C_ = ncamera_rig_->get_T_C_B(topic_camidx.second).inverse();

    camera_found = true;
    break;
  }
  CHECK(camera_found) << "Unable to retrieve camera using the provided id.";

  // Setup LiDAR subscriber
  bool lidar_found = false;
  for (const std::pair<const std::string, aslam::SensorId>& topic_sensorid :    
          ros_topics.lidar_topic_sensor_id_map) {     
    CHECK(topic_sensorid.second.isValid())
      << "The ROS-topic to Lidar sensor id association contains an invalid "
      << "sensor id! topic: " << topic_sensorid.first;
    CHECK(!topic_sensorid.first.empty())
      << "Lidar(" << topic_sensorid.second
      << ") is subscribed to an empty topic!";

    if (lidar_id != topic_sensorid.second) continue;
     
    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>
          lidar_callback = boost::bind(
          &LidarImageProjection::lidarMeasurementCallback, this, _1);
    constexpr size_t kRosSubscriberQueueSizeLidar = 20u;
    ros_subs_ = nh_.subscribe(topic_sensorid.first,
        kRosSubscriberQueueSizeLidar, lidar_callback);
    T_B_L_ = sensor_manager_->getSensor_T_B_S(lidar_id);
    lidar_found = true;
    break;
  }
  CHECK(lidar_found) << "Unable to retrieve LiDAR using the provided id.";

  message_sync_.registerCallback([this] (
    const sensor_msgs::ImageConstPtr& imageMsg, 
    const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
      syncedCallback(imageMsg, cloudMsg);
    });
  T_C_L_ = T_B_C_.inverse() * T_B_L_;

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
    const sensor_msgs::ImageConstPtr& image) {
  message_sync_.callback1(image);
}

void LidarImageProjection::lidarMeasurementCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  message_sync_.callback2(cloudMsg);
}

void LidarImageProjection::syncedCallback(
    const sensor_msgs::ImageConstPtr& imageMsg, 
    const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  cv::Mat image;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (
      new pcl::PointCloud<pcl::PointXYZ>());
  try {
    image = cv_bridge::toCvShare(imageMsg)->image;

    pcl::fromROSMsg (*cloudMsg, *cloud);
  } catch(std::exception& e) {
       LOG(FATAL) << "conversion exception: " << e.what();                          
  }

  VLOG(1) << "received synced callback!";
  pcl::transformPointCloud(*cloud, *cloud, T_C_L_.getTransformationMatrix());
  
  VLOG(1) << "points received before pass: " << cloud->size();
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 60.0);
  pass.filter(*cloud);

  VLOG(1) << "points received after pass: " << cloud->size();
  
}

} // namespace maplab
