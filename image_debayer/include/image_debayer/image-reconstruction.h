#ifndef IMAGE_DEBAYER_IMAGE_RECONSTRUCTION_H_
#define IMAGE_DEBAYER_IMAGE_RECONSTRUCTION_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <sensor_msgs/Image.h>

namespace debayer {

class ImageReconstruction {
 public:
  ImageReconstruction(
      const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool run();
  void imageCallback(const sensor_msgs::ImageConstPtr& image);

 private:
  void setupSubAndPub();
  void publishColorImage(
      const cv_bridge::CvImagePtr& cv_ptr,
      const sensor_msgs::ImageConstPtr& orig_image);
  void debayer(cv::Mat* raw_image, const std::string& pattern);
  void adjustWhiteBalance(cv::Mat* rgb_image);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::AsyncSpinner spinner_;
  image_transport::ImageTransport image_transport_;

  image_transport::Subscriber sub_raw_image_;
  image_transport::Publisher pub_color_image_;
  std::size_t image_counter_ = 0u;
};

}  // namespace debayer

#endif  // IMAGE_DEBAYER_IMAGE_RECONSTRUCTION_H_
