#include "maplab-node/ros-helpers.h"

#include <atomic>
#include <memory>
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
#include <minkindr_conversions/kindr_msg.h>

#include <vio-common/vio-types.h>

DEFINE_bool(
    image_apply_clahe_histogram_equalization, false,
    "Apply CLAHE histogram equalization to image.");
DEFINE_int32(
    image_clahe_clip_limit, 2,
    "CLAHE histogram equalization parameter: clip limit.");
DEFINE_int32(
    image_clahe_grid_size, 8,
    "CLAHE histogram equalization parameter: grid size.");

DEFINE_double(
    image_16_bit_to_8_bit_scale_factor, 1,
    "Scale factor applied to 16bit images when converting them to 8bit "
    "images.");
DEFINE_double(
    image_16_bit_to_8_bit_shift, 0,
    "Shift applied to the scaled values when converting 16bit images to 8bit "
    "images.");

namespace maplab {

vio::ImuMeasurement::Ptr convertRosImuToMaplabImu(
    const sensor_msgs::ImuConstPtr& imu_msg) {
  CHECK(imu_msg);
  vio::ImuMeasurement::Ptr imu_measurement(new vio::ImuMeasurement);
  imu_measurement->timestamp = rosTimeToNanoseconds(imu_msg->header.stamp);
  imu_measurement->imu_data << imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
      imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z;
  return imu_measurement;
}

void applyHistogramEqualization(
    const cv::Mat& input_image, cv::Mat* output_image) {
  CHECK_NOTNULL(output_image);
  CHECK(!input_image.empty());

  static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
      FLAGS_image_clahe_clip_limit,
      cv::Size(FLAGS_image_clahe_grid_size, FLAGS_image_clahe_grid_size));
  clahe->apply(input_image, *output_image);
}

vio::ImageMeasurement::Ptr convertRosImageToMaplabImage(
    const sensor_msgs::ImageConstPtr& image_message, size_t camera_idx) {
  CHECK(image_message);
  cv_bridge::CvImageConstPtr cv_ptr;
  vio::ImageMeasurement::Ptr image_measurement(new vio::ImageMeasurement);
  try {
    // 16bit images are treated differently, we will apply histogram
    // equalization first and then convert them to MONO8;
    if (image_message->encoding == sensor_msgs::image_encodings::MONO16) {
      cv_ptr = cv_bridge::toCvShare(
          image_message, sensor_msgs::image_encodings::MONO16);
      CHECK(cv_ptr);

      cv::Mat processed_image;
      if (FLAGS_image_apply_clahe_histogram_equalization) {
        applyHistogramEqualization(cv_ptr->image, &processed_image);
      } else {
        processed_image = cv_ptr->image;
      }
      processed_image.convertTo(
          image_measurement->image, CV_8U,
          FLAGS_image_16_bit_to_8_bit_scale_factor,
          FLAGS_image_16_bit_to_8_bit_shift);
    } else {
      if (image_message->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
        // NOTE: we assume all 8UC1 type images are monochrome images.
        cv_ptr = cv_bridge::toCvShare(
            image_message, sensor_msgs::image_encodings::TYPE_8UC1);
      } else {
        cv_ptr = cv_bridge::toCvShare(
            image_message, sensor_msgs::image_encodings::MONO8);
      }
      CHECK(cv_ptr);

      if (FLAGS_image_apply_clahe_histogram_equalization) {
        cv::Mat processed_image;
        applyHistogramEqualization(cv_ptr->image, &processed_image);
        image_measurement->image = processed_image;
      } else {
        image_measurement->image = cv_ptr->image.clone();
      }
    }
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

  image_measurement->timestamp =
      rosTimeToNanoseconds(image_message->header.stamp);
  image_measurement->camera_index = camera_idx;
  return image_measurement;
}

vi_map::RosLidarMeasurement::Ptr convertRosCloudToMaplabCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
    const aslam::SensorId& sensor_id) {
  CHECK(cloud_msg);

  vi_map::RosLidarMeasurement::Ptr lidar_measurement(
      new vi_map::RosLidarMeasurement(
          sensor_id, static_cast<int64_t>(cloud_msg->header.stamp.toNSec())));
  *(lidar_measurement->getPointCloudMutable()) = *cloud_msg;
  // pcl::fromROSMsg(*cloud_msg, *(lidar_measurement->getPointCloudMutable()));
  return lidar_measurement;
}

OdometryEstimate::Ptr convertRosOdometryMsgToOdometryEstimate(
    const maplab_msgs::OdometryWithImuBiasesConstPtr& msg) {
  CHECK(msg);

  OdometryEstimate::Ptr odometry_estimate(new OdometryEstimate);

  // Header.
  odometry_estimate->timestamp_ns = msg->header.stamp.toNSec();

  CHECK_GE(odometry_estimate->timestamp_ns, 0);

  // Odometry Pose.
  Eigen::Affine3d eigen_T_M_B;
  tf::poseMsgToEigen(msg->pose.pose, eigen_T_M_B);
  aslam::Transformation T_M_B(eigen_T_M_B.matrix());
  odometry_estimate->vinode.set_T_M_I(T_M_B);

  odometry_estimate->vinode.setTimestamp(odometry_estimate->timestamp_ns);

  // Velocity.
  Eigen::Vector3d v_M_I;
  tf::vectorMsgToEigen(msg->twist.twist.linear, v_M_I);
  odometry_estimate->vinode.set_v_M_I(v_M_I);

  // IMU Biases.
  Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
  tf::vectorMsgToEigen(msg->accel_bias, acc_bias);
  odometry_estimate->vinode.setAccBias(acc_bias);

  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  tf::vectorMsgToEigen(msg->gyro_bias, gyro_bias);
  odometry_estimate->vinode.setGyroBias(gyro_bias);

  // TODO(mfehr): We currently don't support this.
  odometry_estimate->has_T_G_M = false;

  return odometry_estimate;
}

vi_map::ArtifactMeasurement::Ptr convertToArtifactMeasurement(
    const bayesian_artifact_filter_msgs::filtered_artConstPtr& msg,
    const aslam::SensorId& sensor_id) {
  uint64_t ts = rosTimeToNanoseconds(msg->image.header.stamp);

  vi_map::ArtifactMeasurement::Ptr artifact_measurement(
      new vi_map::ArtifactMeasurement(sensor_id, ts));

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    sensor_msgs::Image::ConstPtr rosimg 
      = boost::make_shared<sensor_msgs::Image>(msg->image);
    cv_ptr = cv_bridge::toCvShare(
      rosimg, sensor_msgs::image_encodings::MONO8);
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception for artifacts: " << e.what();
  }
  CHECK(cv_ptr);
  artifact_measurement->getDetectionImage() = cv_ptr->image.clone();
  artifact_measurement->getDetectionProbability() = msg->probability;
  artifact_measurement->getDetectedClassLabel() = msg->class_label.data;
  artifact_measurement->getTimestampNs() = rosTimeToNanoseconds(msg->transform.header.stamp);

  aslam::Transformation T_S_artifact;
  tf::transformMsgToKindr(msg->transform.transform, &T_S_artifact);
  artifact_measurement->getArtifactLocation() = T_S_artifact;
  

  return artifact_measurement;
}

}  // namespace maplab
