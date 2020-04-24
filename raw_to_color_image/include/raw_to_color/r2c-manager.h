#ifndef RAW_TO_COLOR_R2C_MANAGER_H_
#define RAW_TO_COLOR_R2C_MANAGER_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <sensor_msgs/Image.h>

namespace r2c {

class R2CManager {
 public:
  R2CManager(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool run();
  void imageCallback(const sensor_msgs::ImageConstPtr& image);

 private:
  void setupSubAndPub();
  void publishColorImage(
      const cv_bridge::CvImagePtr& cv_ptr,
      const sensor_msgs::ImageConstPtr& orig_image);
  void debayer(cv::Mat* raw_image);
  void adjustWhiteBalance(cv::Mat* rgb_image);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::AsyncSpinner spinner_;
  image_transport::ImageTransport image_transport_;
  cv::Ptr<cv::xphoto::WhiteBalancer> wb_;

  image_transport::Subscriber sub_raw_image_;
  image_transport::Publisher pub_color_image_;
};

}  // namespace r2c

#endif  // RAW_TO_COLOR_R2C_MANAGER_H_
