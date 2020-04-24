#include "raw_to_color/r2c-manager.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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
  // wb_ = cv::xphoto::createGrayworldWB();
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
  CHECK_NOTNULL(image);
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
  CHECK_NOTNULL(cv_ptr);
  debayer(&cv_ptr->image);
  adjustWhiteBalance(&cv_ptr->image);

  publishColorImage(cv_ptr, image);
}

void R2CManager::publishColorImage(
    const cv_bridge::CvImagePtr& cv_ptr,
    const sensor_msgs::ImageConstPtr& orig_image) {
  CHECK_NOTNULL(cv_ptr);
  CHECK_NOTNULL(orig_image);
  const sensor_msgs::ImagePtr color_img_msg = cv_ptr->toImageMsg();
  CHECK_NOTNULL(color_img_msg);

  color_img_msg->encoding = sensor_msgs::image_encodings::RGB8;
  color_img_msg->header.stamp = orig_image->header.stamp;
  pub_color_image_.publish(color_img_msg);
}

void R2CManager::debayer(cv::Mat* raw_image) {
  CHECK_NOTNULL(raw_image);
  cv::cvtColor(*raw_image, *raw_image, CV_BayerGR2RGB);
}

void R2CManager::adjustWhiteBalance(cv::Mat* rgb_image) {
  CHECK_NOTNULL(rgb_image);
  // wb_->balanceWhite(*rgb_image, *rgb_image);
  // credits to http://web.stanford.edu/~sujason/ColorBalancing/simplestcb.html
  constexpr float percent = 30;
  constexpr float half_percent = percent / 200.0f;
  std::vector<cv::Mat> tmpsplit(3);
  cv::split(*rgb_image, tmpsplit);
  for (std::size_t i = 0u; i < 3; ++i) {
    cv::Mat flat;
    tmpsplit[i].reshape(1, 1).copyTo(flat);
    cv::sort(flat, flat, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    const uchar lowval =
        flat.at<uchar>(cvFloor((static_cast<float>(flat.cols)) * half_percent));
    const uchar highval =
        flat.at<uchar>(cvCeil(((float)flat.cols) * (1.0 - half_percent)));
    tmpsplit[i].setTo(lowval, tmpsplit[i] < lowval);
    tmpsplit[i].setTo(highval, tmpsplit[i] > highval);

    cv::normalize(tmpsplit[i], tmpsplit[i], 0, 255, cv::NORM_MINMAX);
  }
  cv::merge(tmpsplit, *rgb_image);
}

}  // namespace r2c
