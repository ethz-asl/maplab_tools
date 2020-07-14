#include "lidar-image-projection/lidar-image-projection-node.h"
#include "lidar-image-projection/rotation-utils.h"

#include <aslam/cameras/camera-pinhole.h>
#include <vi-map/sensor-utils.h>
#include <vio-common/rostopic-settings.h>

#include <Eigen/Dense>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <sensor_msgs/CameraInfo.h>

#include <chrono>
#include <sstream>

DEFINE_string(
    sensor_calibration_file, "",
    "Yaml file with all the sensor calibrations. Determines which sensors are "
    "used by camera info publisher.");
DEFINE_string(selected_camera_id, "", "Defines the used camera id.");
DEFINE_string(selected_lidar_id, "", "Defines the used lidar id.");
DEFINE_double(
    correction_alpha, 0.0, "Defines the correction used for the alpha angle");
DEFINE_double(
    correction_beta, 0.0, "Defines the correction used for the beta angle");
DEFINE_double(
    correction_gamma, 0.0, "Defines the correction used for the gamma angle");

namespace maplab {

LidarImageProjection::LidarImageProjection(
    ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      spinner_(1),
      should_exit_(false),
      image_transport_(nh),
      sensor_manager_(new vi_map::SensorManager()),
      processed_counter_(0u),
      message_sync_(5e7) {
  if (FLAGS_sensor_calibration_file.empty()) {
    LOG(FATAL) << "[LidarImageProjection] No sensors YAML provided!";
  }
  LOG(INFO) << "[LidarImageProjection] Initializing...";
  if (!sensor_manager_->deserializeFromFile(FLAGS_sensor_calibration_file)) {
    LOG(FATAL) << "[LidarImageProjection] "
               << "Failed to read the sensor calibration from '"
               << FLAGS_sensor_calibration_file << "'!";
  }
  CHECK(sensor_manager_);
  if (!initializeServicesAndSubscribers()) {
    LOG(FATAL) << "[MaplabCameraInfoPublisher] "
               << "Failed initialize subscribers and services.";
  }
  cv::namedWindow("Projection", cv::WINDOW_AUTOSIZE);
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
    camera_idx_ = topic_camidx.second;
    const aslam::Camera& camera = ncamera_rig_->getCamera(camera_idx_);
    if (camera_id != camera.getId())
      continue;

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

    if (lidar_id != topic_sensorid.second)
      continue;

    boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>
        lidar_callback = boost::bind(
            &LidarImageProjection::lidarMeasurementCallback, this, _1);
    constexpr size_t kRosSubscriberQueueSizeLidar = 20u;
    ros_subs_ = nh_.subscribe(
        topic_sensorid.first, kRosSubscriberQueueSizeLidar, lidar_callback);
    VLOG(1) << "[LidarImageProjection] LiDAR " << topic_sensorid.second
            << " is subscribed to topic: '" << topic_sensorid.first << "'";
    T_B_L_ = sensor_manager_->getSensor_T_B_S(lidar_id);
    lidar_found = true;
    break;
  }
  CHECK(lidar_found) << "Unable to retrieve LiDAR using the provided id.";

  message_sync_.registerCallback(
      [this](
          const sensor_msgs::ImageConstPtr& imageMsg,
          const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
        syncedCallback(imageMsg, cloudMsg);
      });
  aslam::Transformation correction(RotationUtils::CreateTransformation(
      FLAGS_correction_alpha, FLAGS_correction_beta, FLAGS_correction_gamma));
  T_C_L_ = T_B_C_.inverse() * (correction * T_B_L_);

  return true;
}

bool LidarImageProjection::initializeNCamera() {
  CHECK(sensor_manager_);
  ncamera_rig_ = vi_map::getSelectedNCamera(*sensor_manager_);
  return ncamera_rig_ != nullptr;
}

bool LidarImageProjection::run() {
  LOG(INFO) << "[MaplabCameraInfoPublisher] Starting...";
  spinner_.start();
  return true;
}

void LidarImageProjection::shutdown() {}

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
  aslam::Transformation T_B_L = T_B_C_ * T_C_L_;
  ss << "Current transformation from lidar to imu is: \n" << T_B_L << "\n";
  ss << "Inverse transformation: \n" << T_B_L.inverse() << "\n";
  ss << "Quaternion (w,x,y,z): \n"
     << T_B_L.getRotation() << "\n"
     << "inverse Quaternion (w,x,y,z): \n"
     << T_B_L.getRotation().inverse() << "\n";

  return ss.str();
}

void LidarImageProjection::imageCallback(
    const sensor_msgs::ImageConstPtr& image) {
  // VLOG(7) << "Received image measurement at " << image->header.stamp;
  message_sync_.callback1(image);
}

void LidarImageProjection::lidarMeasurementCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  // VLOG(7) << "Received LiDAR measurement at " << cloudMsg->header.stamp;
  message_sync_.callback2(cloudMsg);
}

void LidarImageProjection::syncedCallback(
    const sensor_msgs::ImageConstPtr& imageMsg,
    const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
  cv::Mat image;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  try {
    image = cv_bridge::toCvShare(imageMsg)->image;
    // CHECK TYPE
    cv::cvtColor(image, image, CV_GRAY2RGB);
    pcl::fromROSMsg(*cloudMsg, *cloud);
  } catch (std::exception& e) {
    LOG(ERROR) << "conversion exception: " << e.what();
    return;
  }

  // Project the cloud.
  auto start = std::chrono::high_resolution_clock::now();
  cv::Vec3b pcl_color(100, 100, 100);
  pcl::transformPointCloud(*cloud, *cloud, T_C_L_.getTransformationMatrix());
  const aslam::Camera& camera = ncamera_rig_->getCamera(camera_idx_);
  const uint32_t n_points = cloud->size();
  for (uint32_t i = 0u; i < n_points; ++i) {
    Eigen::Vector2d keypoint;
    pcl::PointXYZI cur_point = cloud->points[i];
    Eigen::Vector3d point_3d(cur_point.x, cur_point.y, cur_point.z);
    aslam::ProjectionResult ret = camera.project3(point_3d, &keypoint);
    if (ret.getDetailedStatus() ==
        aslam::ProjectionResult::Status::KEYPOINT_VISIBLE) {
      float dist = std::sqrt(
          cur_point.x * cur_point.x + cur_point.y * cur_point.y +
          cur_point.z * cur_point.z);
      // const float gray = std::fmod(cur_point.intensity, 80.0f) / 80.0f;
      const float gray = std::fmod(dist, 10.0f) / 10.0f;
      image.at<cv::Vec3b>(keypoint(1), keypoint(0)) = intensityToRGB(gray);
    }
  }
  auto end = std::chrono::high_resolution_clock::now();
  total_processing_time_ms_ +=
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();

  // Show the projection.
  cv::imshow("Projection", image);
  cv::waitKey(10);
  ++processed_counter_;
}

double LidarImageProjection::interpolate(
    double val, double y0, double x0, double y1, double x1) const {
  return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

double LidarImageProjection::base(double val) const {
  if (val <= -0.75)
    return 0;
  else if (val <= -0.25)
    return interpolate(val, 0.0, -0.75, 1.0, -0.25);
  else if (val <= 0.25)
    return 1.0;
  else if (val <= 0.75)
    return interpolate(val, 1.0, 0.25, 0.0, 0.75);
  else
    return 0.0;
}

cv::Vec3f LidarImageProjection::intensityToRGB(float intensity) const {
  return cv::Vec3f(
      base(intensity - 0.5) * 255.0f, base(intensity) * 255.0f,
      base(intensity + 0.5) * 255.0f);
}

}  // namespace maplab
