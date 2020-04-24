#ifndef RAW_TO_COLOR_ROS_UTILS_H_
#define RAW_TO_COLOR_ROS_UTILS_H_

#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace r2c {

void parseGflagsFromRosParams(
    const char* program_name, const ros::NodeHandle& nh_private);

constexpr int64_t rosTimeToNanoseconds(const ros::Time& rostime) {
  return static_cast<int64_t>(rostime.sec * 1e9) +
         static_cast<int64_t>(rostime.nsec);
}

}  // namespace r2c

#endif  // RAW_TO_COLOR_ROS_UTILS_H_
