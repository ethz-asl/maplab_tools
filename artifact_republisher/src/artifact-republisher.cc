#include "artifact_republisher/artifact-republisher.h"

#include <vio-common/rostopic-settings.h>
#include <vi-map/sensor-utils.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>
#include <maplab_msgs/Artifact.h>

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>
#include <boost/bind.hpp>

#include <sstream>
#include <chrono>

DEFINE_string(
    subscriber_topic, "", 
    "Sets the topic to subscribe to.");
DEFINE_string(
    publisher_topic, "", 
    "Sets the topic to publish to.");
DEFINE_string(
    robot_name, "", 
    "Defines the robot name.");

namespace maplab {

ArtifactRepublisher::ArtifactRepublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), 
		processed_counter_(0u) {
  CHECK(!FLAGS_subscriber_topic.empty());
  CHECK(!FLAGS_publisher_topic.empty());
  CHECK(!FLAGS_robot_name.empty());

  if (!initializeSubscribers()) {
     LOG(FATAL) << "[ArtifactRepublisher] " 
       << "Failed initialize subscribers and services.";
  }
}


bool ArtifactRepublisher::run() {
  LOG(INFO) << "[ArtifactRepublisher] Starting...";
  spinner_.start();
  return true;
}

void ArtifactRepublisher::shutdown(){
  // noop
}

std::atomic<bool>& ArtifactRepublisher::shouldExit() {
  return should_exit_;
}

std::string ArtifactRepublisher::printStatistics() const {
  std::stringstream ss;
  ss << "[ArtifactRepublisher]  Statistics \n";
	ss << "\t processed: " << processed_counter_ << " artifacts\n";
  return ss.str();
}

bool ArtifactRepublisher::initializeSubscribers() {
  // Handle normal images.
  boost::function<void(const maplab_msgs::filtered_artConstPtr&)> art_callback =
    boost::bind(&ArtifactRepublisher::artifact_callback, this, _1, 
        FLAGS_robot_name);

  constexpr size_t kRosSubscriberQueueSizeImage = 20u;
  art_sub_ = nh_.subscribe(FLAGS_subscriber_topic,
      kRosSubscriberQueueSizeImage, art_callback);

  art_pub_ = nh_.advertise<maplab_msgs::Artifact>(FLAGS_publisher_topic, 1);
  return true;
}
void ArtifactRepublisher::artifact_callback(
    const maplab_msgs::filtered_artConstPtr& filtered_art, 
    const std::string& robot_name) {
  VLOG(1) << "Received artifact message";
  
  maplab_msgs::Artifact artifact;
  artifact.robot_name.data = robot_name;
  artifact.point = filtered_art->point;
  artifact.image = filtered_art->image;
  artifact.compressed_image = filtered_art->compressed_image;
  artifact.class_label = filtered_art->class_label;
  artifact.probability = filtered_art->probability;
  artifact.transform = filtered_art->transform;

  art_pub_.publish(artifact);
}

} // namespace maplab
