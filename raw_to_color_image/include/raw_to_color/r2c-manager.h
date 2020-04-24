#ifndef RAW_TO_COLOR_R2C_MANAGER_H_
#define RAW_TO_COLOR_R2C_MANAGER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace r2c {

class R2CManager {
 public:
  R2CManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool run();
  void imageCallback(const sensor_msgs::ImageConstPtr& image);

 private:
  void setupSubscriber();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::AsyncSpinner spinner_;
  image_transport::ImageTransport image_transport_;
  image_transport::Subscriber sub_image_;
};

}  // namespace r2c

#endif  // RAW_TO_COLOR_R2C_MANAGER_H_
