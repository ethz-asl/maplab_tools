#include <maplab-camera-info-publisher/maplab-camera-info-publisher-node.h>
#include <vio-common/rostopic-settings.h>
#include <vi-map/sensor-utils.h>
#include <aslam/cameras/camera.h>
#include <aslam/cameras/camera-pinhole.h>

#include <sensor_msgs/CameraInfo.h>
#include <glog/logging.h>
#include <boost/bind.hpp>
#include <sstream>

DEFINE_string(
   sensor_calibration_file, "",
   "Yaml file with all the sensor calibrations. Determines which sensors are "
   "used by camera info publisher.");

DEFINE_string(
   cam_info_topic_suffix, "/camera_info",
   "Defines the topic suffix used to publish the camera info.");

DEFINE_string(
   cam_scale_topic_suffix, "/scaled_image",
   "Defines the topic suffix used to publish the scaled image.");

DEFINE_double(
		image_scale_factor, 1.0, 
		"Defines the scale of republished image.");

namespace maplab {

MaplabCameraInfoPublisher::MaplabCameraInfoPublisher(ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), 
    spinner_(1), should_exit_(false), 
    image_transport_(nh),
    sensor_manager_(new vi_map::SensorManager()) {
  if (FLAGS_sensor_calibration_file.empty()) {
    LOG(FATAL) << "No sensors YAML provided!";
  }
  LOG(INFO) 
		<< "[MaplabCameraInfoPublisher] Initializing camera info publisher...";
  if (!sensor_manager_->deserializeFromFile(FLAGS_sensor_calibration_file)) {
     LOG(FATAL) 
			 << "[MaplabCameraInfoPublisher] " 
			 << "Failed to read the sensor calibration from '"
			 << FLAGS_sensor_calibration_file << "'!";
  }
  CHECK(sensor_manager_);
  if (!initializeServicesAndSubscribers()) {
     LOG(FATAL) << "[MaplabCameraInfoPublisher] " 
       << "Failed initialize subscribers and services.";
  }
}

bool MaplabCameraInfoPublisher::initializeServicesAndSubscribers() {
  // First initialize the ncamera.
  if (!initializeNCamera()) {
    LOG(FATAL) << "[MaplabCameraInfoPublisher] " 
      << "Failed to initialize the ncamera pointer.";
  }
  CHECK(ncamera_rig_);

  // Setup service calls.
  boost::function<bool(
     std_srvs::Empty::Request&, std_srvs::Empty::Response&)> callback_start =
     boost::bind(&MaplabCameraInfoPublisher::startPublishing, this, _1, _2);
  services_.emplace_back(nh_.advertiseService(
        kStartServiceTopic, callback_start));

  boost::function<bool(
     std_srvs::Empty::Request&, std_srvs::Empty::Response&)> callback_stop =
     boost::bind(&MaplabCameraInfoPublisher::stopPublishing, this, _1, _2);
  services_.emplace_back(nh_.advertiseService(
        kStopServiceTopic, callback_stop));

  // Setup image subscribers and info publishers.
  vio_common::RosTopicSettings ros_topics(*sensor_manager_);
  const size_t num_cameras = ros_topics.camera_topic_cam_index_map.size();
  if (num_cameras == 0) {
    return false;
  }
  sub_images_.reserve(num_cameras);
  info_pubs_.reserve(num_cameras);
	if (FLAGS_image_scale_factor != 1.0) {
		scaled_pubs_.reserve(num_cameras);
	}

  for (const std::pair<std::string, size_t>& topic_camidx :
        ros_topics.camera_topic_cam_index_map) {
    // Setup subscriber.
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

    // Setup publisher.
    const aslam::Camera& camera = ncamera_rig_->getCamera(topic_camidx.second);
    constexpr size_t kRosPublisherQueueSize = 100u;
    const std::string info_topic = camera.getTopic() 
			+ FLAGS_cam_info_topic_suffix;
    info_pubs_.emplace_back(nh_.advertise<sensor_msgs::CameraInfo>(
        info_topic, kRosPublisherQueueSize));

		if (FLAGS_image_scale_factor != 1.0) {
			const std::string scale_topic = camera.getTopic() 
				+ FLAGS_cam_scale_topic_suffix;
			scaled_pubs_.emplace_back(image_transport_.advertise(scale_topic, 1));
		}
  }


  return true;
}

bool MaplabCameraInfoPublisher::initializeNCamera() {
  CHECK(sensor_manager_);
  ncamera_rig_ =
      vi_map::getSelectedNCamera(*sensor_manager_);
  return ncamera_rig_ != nullptr;
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
  if (!should_publish_) {
    return;
  }
	if (FLAGS_image_scale_factor != 1.0) {
		const cv::Mat scaled = rescaleImage(FLAGS_image_scale_factor, 
				image);
		publishRescaled(scaled, camera_idx, image);
	}
  createAndPublishCameraInfo(camera_idx, image);
}

bool MaplabCameraInfoPublisher::startPublishing(
    std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  LOG(INFO) << "Publisher started...";
  should_publish_ = true;
  return true;
}

bool MaplabCameraInfoPublisher::stopPublishing(
    std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  LOG(INFO) << "Publisher stopped...";
  should_publish_ = false;
  return true;
}

bool MaplabCameraInfoPublisher::retrieveCameraIntrinsics(
    const aslam::Camera& camera, 
    double* fu, double* fv, double* cu, double *cv) const {
  CHECK(fu);
  CHECK(fv);
  CHECK(cu);
  CHECK(cv);
  
  switch (camera.getType()) {
    case aslam::Camera::Type::kPinhole: {
      Eigen::VectorXd parameters = camera.getParameters();
      *fu = parameters(aslam::PinholeCamera::Parameters::kFu);
      *fv = parameters(aslam::PinholeCamera::Parameters::kFv);
      *cu = parameters(aslam::PinholeCamera::Parameters::kCu);
      *cv = parameters(aslam::PinholeCamera::Parameters::kCv);
      break;
    }
    default:
      LOG(FATAL) << "Unknown camera type. The given camera has to be of type "
          "Pinhole.";

  }
  return true;
}

bool MaplabCameraInfoPublisher::retrieveDistortionParameters(
    const aslam::Camera& camera, double* k1, double* k2, 
    double* k3, double *k4, std::string* type) const {
  CHECK(k1);
  CHECK(k2);
  CHECK(k3);
  CHECK(k4);

  const aslam::Distortion& distortion = camera.getDistortion();
  switch (distortion.getType()) {
    case aslam::Distortion::Type::kEquidistant: {
      Eigen::VectorXd parameters = distortion.getParameters();
      *k1 = parameters(0);
      *k2 = parameters(1);
      *k3 = parameters(2);
      *k4 = parameters(3);
      *type = "equidistant";
      break;
    }
    default:
      LOG(FATAL) << "Unknown distortion type." 
        " The given distortion has to be of type equidistant.";
  }
  return true;
}

void MaplabCameraInfoPublisher::createAndPublishCameraInfo(
    const std::size_t camera_idx,
    const sensor_msgs::ImageConstPtr &image) const {
  CHECK(ncamera_rig_);
  CHECK(image);
  const aslam::Camera& camera = ncamera_rig_->getCamera(camera_idx);

  // Retrieve camera parameters.
  double fu, fv, cu, cv; 
  retrieveCameraIntrinsics(camera, &fu, &fv, &cu, &cv);
  double k1, k2, k3, k4;
  std::string distortion_type;
  retrieveDistortionParameters(camera, &k1, &k2, &k3, &k4, &distortion_type);

  // Create camera info ros message.
  sensor_msgs::CameraInfo cam_info_msg;
  cam_info_msg.header.stamp = image->header.stamp;
  cam_info_msg.header.frame_id = image->header.frame_id;
  cam_info_msg.width = camera.imageWidth();
  cam_info_msg.height = camera.imageHeight();

  // We assume monocular cameras, thus the rectification 
  // matrix is a 3x3 identity matrix. 
  cam_info_msg.R[0] = 1.0;
  cam_info_msg.R[4] = 1.0;
  cam_info_msg.R[8] = 1.0;

  // Set the camera intrinsics for distored images. 
  cam_info_msg.K[0] = fu;
  cam_info_msg.K[2] = cu;
  cam_info_msg.K[4] = fv;
  cam_info_msg.K[5] = cv;
  cam_info_msg.K[8] = 1.0;

  // Set the projection matrix with Tx = Ty = 0. 
  cam_info_msg.P[0] = cam_info_msg.K[0];
  cam_info_msg.P[2] = cam_info_msg.K[2];
  cam_info_msg.P[4] = cam_info_msg.K[4];
  cam_info_msg.P[5] = cam_info_msg.K[5];
  
  // Set the distortion model. 
  cam_info_msg.distortion_model = distortion_type;
  cam_info_msg.D = {k1, k2, k3, k4};

  CHECK_LT(camera_idx, info_pubs_.size());
  info_pubs_[camera_idx].publish(cam_info_msg);
}

cv::Mat MaplabCameraInfoPublisher::rescaleImage(const double scale,
		const sensor_msgs::ImageConstPtr &image) const {
  cv_bridge::CvImageConstPtr cv_ptr;
	try {
	  cv_ptr = cv_bridge::toCvShare(
			 image, sensor_msgs::image_encodings::BGR8);
	} catch (const cv_bridge::Exception& e) {  // NOLINT
    LOG(FATAL) << "cv_bridge exception: " << e.what();
  }
  CHECK(cv_ptr);

	const cv::Mat& old_img = cv_ptr->image.clone();
	const int new_cols = round(old_img.cols * scale);
	const int new_rows = round(old_img.rows * scale);

	cv::Mat new_img = cv::Mat(new_cols, new_rows, old_img.type());
	cv::Size size(new_cols, new_rows);
	cv::resize(old_img, new_img, size);
	return new_img;
}

void MaplabCameraInfoPublisher::publishRescaled(const cv::Mat& img, 
    const std::size_t camera_idx,
		const sensor_msgs::ImageConstPtr &orig_img) const {
	sensor_msgs::ImagePtr img_msg 
		= cv_bridge::CvImage(
				orig_img->header, "mono8", img).toImageMsg();

  CHECK_LT(camera_idx, scaled_pubs_.size());
  scaled_pubs_[camera_idx].publish(img_msg);
}

} // namespace maplab
