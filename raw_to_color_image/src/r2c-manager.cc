#include "raw_to_color/r2c-manager.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

DEFINE_string(cam_topic, "", "Defines the topic for the images.");

namespace r2c {

R2CManager::R2CManager(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), spinner_(1), image_transport_(nh) {
  CHECK(!FLAGS_cam_topic.empty());
  setupSubscriber();
}

void R2CManager::setupSubscriber() {
  boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
      boost::bind(&R2CManager::imageCallback, this, _1);
  constexpr size_t kRosSubscriberQueueSizeImage = 20u;
  sub_image_ = image_transport_.subscribe(
      FLAGS_cam_topic, kRosSubscriberQueueSizeImage, image_callback);
}

bool R2CManager::run() {
  LOG(INFO) << "[R2CManager] Starting...";
  spinner_.start();
  return true;
}

void R2CManager::imageCallback(const sensor_msgs::ImageConstPtr& image) {
  VLOG(1) << "received image";
}

}  // namespace r2c
