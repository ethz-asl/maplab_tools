#include "raw_to_color/r2c-manager.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

DEFINE_string(cam_topic, "", "Defines the topic for the input images.");
DEFINE_string(color_topic, "", "Defines the topic for the processed images.");

namespace r2c {

R2CManager::R2CManager(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), spinner_(1), image_transport_(nh) {
  CHECK(!FLAGS_cam_topic.empty());
  CHECK(!FLAGS_color_topic.empty());
  setupSubAndPub();
}

void R2CManager::setupSubAndPub() {
  // Set up the raw image subscriber.
  boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
      boost::bind(&R2CManager::imageCallback, this, _1);
  constexpr size_t kRosQueueSize = 20u;
  sub_raw_image_ = image_transport_.subscribe(
      FLAGS_cam_topic, kRosQueueSize, image_callback);

  // Set up the processed image publisher.
  pub_color_image_ =
      image_transport_.advertise(FLAGS_color_topic, kRosQueueSize);
}

bool R2CManager::run() {
  LOG(INFO) << "[R2CManager] Starting...";
  spinner_.start();
  return true;
}

void R2CManager::imageCallback(const sensor_msgs::ImageConstPtr& image) {
  VLOG(1) << "received image";
  cv_bridge::CvImagePtr cv_ptr;
  try {
    if (image->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    } else {
      LOG(ERROR) << "Unknown image encoding: " << image->encoding;
    }
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);
  // cv::Mat new_img = cv_ptr->image.clone();

  publishColorImage(cv_ptr, image);
}

void R2CManager::publishColorImage(
    const cv_bridge::CvImagePtr& cv_ptr,
    const sensor_msgs::ImageConstPtr& orig_image) {
  sensor_msgs::ImagePtr color_img_msg = cv_ptr->toImageMsg();
  color_img_msg->encoding = sensor_msgs::image_encodings::BGR8;
  color_img_msg->header.stamp = orig_image->header.stamp;
  pub_color_image_.publish(color_img_msg);
}

}  // namespace r2c
