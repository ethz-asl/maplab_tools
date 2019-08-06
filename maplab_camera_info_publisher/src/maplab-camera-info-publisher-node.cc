#include <maplab-camera-info-publisher/maplab-camera-info-publisher-node.h>
#include <vio-common/rostopic-settings.h>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>


DEFINE_string(
   sensor_calibration_file, "",
   "Yaml file with all the sensor calibrations. Determines which sensors are "
   "used by camera info publisher.");

DEFINE_string(
   front_facing_cam_topic, "",
   "Defines the camera used for the camera info publisher.");


namespace maplab {

MaplabCameraInfoPublisher::MaplabCameraInfoPublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), 
    image_transport_(nh),
    sensor_manager_(new vi_map::SensorManager()) {
  if (FLAGS_sensor_calibration_file.empty()) {
    LOG(FATAL) << "No sensors YAML provided!";
  }
  if (FLAGS_front_facing_cam_topic.empty()) {
    //LOG(FATAL) << "No front facing camera topic provided!";
  }
  LOG(INFO) << "[MaplabCameraInfoPublisher] Initializing camera info publisher...";
  if (!sensor_manager_->deserializeFromFile(FLAGS_sensor_calibration_file)) {
     LOG(FATAL) << "[MaplabNode] Failed to read the sensor calibration from '"
                << FLAGS_sensor_calibration_file << "'!";
  }
  CHECK(sensor_manager_);
  if (!initialize_services_and_subscribers()) {
     LOG(FATAL) << "[MaplabNode] Failed initialize subscribers and services";
  }
}

bool MaplabCameraInfoPublisher::initialize_services_and_subscribers() {
  vio_common::RosTopicSettings ros_topics(*sensor_manager_);
  const size_t num_cameras = ros_topics.camera_topic_cam_index_map.size();
  if (num_cameras == 0) {
    return false;
  }
  sub_images_.reserve(num_cameras);

  for (const std::pair<std::string, size_t>& topic_camidx :
        ros_topics.camera_topic_cam_index_map) {
    CHECK(!topic_camidx.first.empty()) << "Camera " << topic_camidx.second
                                        << " is subscribed to an empty topic!";

    boost::function<void(const sensor_msgs::ImageConstPtr&)> image_callback =
        boost::bind(&MaplabCameraInfoPublisher::imageCallback,
            this, _1, topic_camidx.second);

       constexpr size_t kRosSubscriberQueueSizeImage = 20u;
    image_transport::Subscriber image_sub = image_transport_.subscribe(
        topic_camidx.first, kRosSubscriberQueueSizeImage, image_callback);
    sub_images_.emplace_back(image_sub);

    VLOG(1) << "[MaplabNode-DataSource] Camera " << topic_camidx.second
            << " is subscribed to topic: '" << topic_camidx.first << "'";
  }

  return true;
}

bool MaplabCameraInfoPublisher::run() {
  LOG(INFO) << "[MaplabCameraInfoPublisher] Starting...";
  spinner_.start();
  return true;
}

void MaplabCameraInfoPublisher::shutdown(){

}

std::atomic<bool>& MaplabCameraInfoPublisher::shouldExit() {
  return should_exit_;
}

std::string MaplabCameraInfoPublisher::printStatistics() const {
  std::stringstream ss;
  ss << "[MaplabCameraInfoPublisher]  Statistics \n";
  return ss.str();
}

void MaplabCameraInfoPublisher::imageCallback(
    const sensor_msgs::ImageConstPtr &image, 
    std::size_t camera_idx) {
  VLOG(1) << "got image from " << camera_idx;
}

} // namespace maplab
