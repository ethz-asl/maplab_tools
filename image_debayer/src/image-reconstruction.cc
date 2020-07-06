#include "image_debayer/image-reconstruction.h"

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
DEFINE_string(
    bayer_pattern, "GR2RGB", "Defines the Bayer pattern used in debayering.");
DEFINE_double(
    simple_wb_saturation_percentage, 10.0,
    "Defines the percentage of the bright/dark pixel saturation.");

namespace debayer {
ImageReconstruction::ImageReconstruction(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), spinner_(1), image_transport_(nh) {
  CHECK(!FLAGS_cam_topic.empty());
  CHECK(!FLAGS_color_topic.empty());
  setupSubAndPub();
}

void ImageReconstruction::setupSubAndPub() {
  // Set up the raw image subscriber.
  boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
      boost::bind(&ImageReconstruction::imageCallback, this, _1);
  constexpr size_t kRosQueueSize = 20u;
  sub_raw_image_ = image_transport_.subscribe(
      FLAGS_cam_topic, kRosQueueSize, image_callback);

  // Set up the processed image publisher.
  pub_color_image_ =
      image_transport_.advertise(FLAGS_color_topic, kRosQueueSize);
}

bool ImageReconstruction::run() {
  LOG(INFO) << "[ImageReconstruction] Starting...";
  spinner_.start();
  return true;
}

void ImageReconstruction::imageCallback(
    const sensor_msgs::ImageConstPtr& image) {
  CHECK_NOTNULL(image);
  cv_bridge::CvImagePtr cv_ptr;
  try {
    // Currently the incomplete color image output is published as a mono8.
    if (image->encoding == sensor_msgs::image_encodings::MONO8) {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    } else {
      LOG(ERROR) << "Unknown image encoding: " << image->encoding;
    }
  } catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK_NOTNULL(cv_ptr);
  debayer(&cv_ptr->image, FLAGS_bayer_pattern);
  adjustWhiteBalance(&cv_ptr->image);

  publishColorImage(cv_ptr, image);
}

void ImageReconstruction::publishColorImage(
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

void ImageReconstruction::debayer(
    cv::Mat* raw_image, const std::string& pattern) {
  CHECK_NOTNULL(raw_image);
  if (pattern == "GR2RGB")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerGR2RGB_EA);
  else if (pattern == "RG2RGB")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerRG2RGB_EA);
  else if (pattern == "GB2RGB")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerGB2RGB_EA);
  else if (pattern == "BG2RGB")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerBG2RGB_EA);
  else if (pattern == "GR2BGR")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerGR2BGR_EA);
  else if (pattern == "RG2BGR")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerRG2BGR_EA);
  else if (pattern == "GB2BGR")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerGB2BGR_EA);
  else if (pattern == "BG2BGR")
    cv::cvtColor(*raw_image, *raw_image, cv::COLOR_BayerBG2BGR_EA);
  else
    LOG(ERROR) << "Unknown pattern given: " << pattern;
}

void ImageReconstruction::adjustWhiteBalance(cv::Mat* rgb_image) {
  CHECK_NOTNULL(rgb_image);
  // credits to
  // http://web.stanford.edu/~sujason/ColorBalancing/simplestcb.html
  const float percent = FLAGS_simple_wb_saturation_percentage;
  const float half_percent = percent / 200.0f;
  std::vector<cv::Mat> tmpsplit(3);
  cv::split(*rgb_image, tmpsplit);
  for (std::size_t i = 0u; i < 3; ++i) {
    cv::Mat flat;
    tmpsplit[i].reshape(1, 1).copyTo(flat);
    cv::sort(flat, flat, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
    const float f_cols = static_cast<float>(flat.cols);
    const uchar lowval = flat.at<uchar>(cvFloor(f_cols * half_percent));
    const uchar highval = flat.at<uchar>(cvCeil(f_cols * (1.0 - half_percent)));
    tmpsplit[i].setTo(lowval, tmpsplit[i] < lowval);
    tmpsplit[i].setTo(highval, tmpsplit[i] > highval);

    cv::normalize(tmpsplit[i], tmpsplit[i], 0, 255, cv::NORM_MINMAX);
  }
  cv::merge(tmpsplit, *rgb_image);
}

}  // namespace debayer
