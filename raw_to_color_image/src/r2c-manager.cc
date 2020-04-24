#include "raw_to_color/r2c-manager.h"

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

namespace r2c {

R2CManager::R2CManager(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), spinner_(1) {}

}  // namespace r2c
