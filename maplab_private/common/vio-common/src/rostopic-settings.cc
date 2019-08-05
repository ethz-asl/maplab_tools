#include "vio-common/rostopic-settings.h"

#include <string>
#include <unordered_map>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <sensors/gps-utm.h>
#include <sensors/gps-wgs.h>
#include <sensors/lidar.h>
#include <sensors/relative-6dof-pose.h>
#include <vi-map/sensor-utils.h>

DEFINE_string(
    vio_camera_topic_suffix, "/image_raw",
    "Image name appended to camera topic.");
DEFINE_string(
    vio_hardware_features_topic, "hw_features",
    "Name of the hardware features topic.");

namespace vio_common {

RosTopicSettings::RosTopicSettings(
    const aslam::NCamera& camera_system, const vi_map::Imu& imu)
    : imu_topic(imu.getTopic()),
      camera_topic_suffix(FLAGS_vio_camera_topic_suffix),
      relative_6dof_topic(""),
      hardware_features_topic(FLAGS_vio_hardware_features_topic),
      gps_wgs_topic(""),
      gps_utm_topic(""),
      odometry_6dof_topic("") {
  CHECK(!imu_topic.empty());
  for (size_t cam_idx = 0u; cam_idx < camera_system.getNumCameras();
       ++cam_idx) {
    camera_topic_cam_index_map.emplace(std::make_pair(
        camera_system.getCameraShared(cam_idx)->getTopic() +
            camera_topic_suffix,
        cam_idx));
  }
}

RosTopicSettings::RosTopicSettings(const vi_map::SensorManager& sensor_manager)
    : imu_topic(""),
      camera_topic_suffix(FLAGS_vio_camera_topic_suffix),
      relative_6dof_topic(""),
      hardware_features_topic(FLAGS_vio_hardware_features_topic),
      gps_wgs_topic(""),
      gps_utm_topic(""),
      odometry_6dof_topic(""){
  aslam::NCamera::Ptr ncamera = getSelectedNCamera(sensor_manager);
  if (ncamera) {
    const size_t num_cameras = ncamera->numCameras();
    for (size_t cam_idx = 0u; cam_idx < num_cameras; ++cam_idx) {
      const std::string camera_topic =
          ncamera->getCamera(cam_idx).getTopic() + camera_topic_suffix;
      CHECK(!camera_topic.empty())
          << "Camera " << cam_idx << " of NCamera ('" << ncamera->getId()
          << "') has an empty ROS topic!";
      CHECK(camera_topic_cam_index_map.emplace(camera_topic, cam_idx).second)
          << "Topic '" << camera_topic
          << "' already exists in the ROS topic settings, meaning another "
             "camera already subscribes to this topic!";
    }
  }

  vi_map::Imu::Ptr imu = getSelectedImu(sensor_manager);
  if (imu) {
    imu_topic = imu->getTopic();
    CHECK(!imu_topic.empty()) << "The selected IMU ('" << imu->getId()
                              << "') has an empty ROS topic!";
  }

  vi_map::Lidar::Ptr lidar = getSelectedLidar(sensor_manager);
  if (lidar) {
    const std::string lidar_topic = lidar->getTopic();
    CHECK(!lidar_topic.empty()) << "The selected LIDAR ('" << lidar->getId()
                                << "') has an empty ROS topic!";
    CHECK(
        lidar_topic_sensor_id_map.emplace(lidar_topic, lidar->getId()).second);
  }

  // TODO(mfehr): reenable multi-lidar support.
  // Set lidar_sensor_ids;
  // sensor_manager->getAllSensorIdsOfType(
  //     vi_map::SensorType::kLidar, &lidar_sensor_ids);
  //
  // for (const & lidar_sensor_id : lidar_sensor_ids) {
  //   CHECK(lidar_sensor_id.isValid());
  //   const std::string& hardware_id =
  //       sensor_manager->getSensor<vi_map::Lidar>(lidar_sensor_id).getTopic();
  //   CHECK(!hardware_id.empty());
  //   CHECK(
  //       lidar_topic_sensor_id_map.emplace(hardware_id,
  //       lidar_sensor_id).second);
  // }
  // CHECK_EQ(lidar_topic_sensor_id_map.size(), lidar_sensor_ids.size());

  vi_map::Relative6DoF::Ptr relative_6dof =
      getSelectedRelative6DoFSensor(sensor_manager);
  if (relative_6dof) {
    relative_6dof_topic = relative_6dof->getTopic();
    CHECK(!relative_6dof_topic.empty())
        << "The selected Relative 6DOF sensor ('" << relative_6dof->getId()
        << "') has an empty ROS topic!";
  }

  vi_map::Odometry6DoF::Ptr odometry_6dof_sensor =
      getSelectedOdometry6DoFSensor(sensor_manager);
  if (odometry_6dof_sensor) {
    odometry_6dof_topic = odometry_6dof_sensor->getTopic();
    CHECK(!odometry_6dof_topic.empty())
        << "The selected Odometry 6DOF sensor ('"
        << odometry_6dof_sensor->getId() << "') has an empty ROS topic!";
  }

  vi_map::ArtifactDetection::Ptr artifact_detection_sensor =
      getSelectedArtifactDetectionSensor(sensor_manager);
  if (artifact_detection_sensor) {
    const std::string artifact_topic = artifact_detection_sensor->getTopic();
    CHECK(!artifact_topic.empty())
        << "The selected artifact detection sensor ('"
        << artifact_detection_sensor->getId() << "') has an empty ROS topic!";
    CHECK(
        artifact_topic_sensor_id_map.emplace(artifact_topic, 
          artifact_detection_sensor->getId()).second);
  }

  vi_map::GpsUtm::Ptr gps_utm = getSelectedGpsUtmSensor(sensor_manager);
  if (gps_utm) {
    gps_utm_topic = gps_utm->getTopic();
    CHECK(!gps_utm_topic.empty())
        << "The selected GPS UTM sensor ('" << gps_utm->getId()
        << "') has an empty ROS topic!";
  }

  vi_map::GpsWgs::Ptr gps_wgs = getSelectedGpsWgsSensor(sensor_manager);
  if (gps_wgs) {
    gps_wgs_topic = gps_wgs->getTopic();
    CHECK(!gps_wgs_topic.empty())
        << "The selected GPS WGS sensor ('" << gps_wgs->getId()
        << "') has an empty ROS topic!";
  }
}

void RosTopicSettings::getAllValidTopics(std::vector<std::string>* topics) {
  CHECK_NOTNULL(topics)->clear();
  makeAbsoluteTopics();

  for (const CameraTopicIdxMap::value_type& camera_topic_idx_pair :
       camera_topic_cam_index_map) {
    topics->emplace_back(camera_topic_idx_pair.first);
  }

  if (!imu_topic.empty()) {
    topics->emplace_back(imu_topic);
  }

  if (!relative_6dof_topic.empty()) {
    topics->emplace_back(relative_6dof_topic);
  }

  if (!gps_wgs_topic.empty()) {
    topics->emplace_back(gps_wgs_topic);
  }

  if (!gps_utm_topic.empty()) {
    topics->emplace_back(gps_utm_topic);
  }

  for (const TopicSensorIdMap::value_type& lidar_topic_sensor_id_pair :
       lidar_topic_sensor_id_map) {
    topics->emplace_back(lidar_topic_sensor_id_pair.first);
  }

  if (!odometry_6dof_topic.empty()) {
    topics->emplace_back(odometry_6dof_topic);
  }

  
  for (const TopicSensorIdMap::value_type& artifact_topic_sensor_id_pair :
       artifact_topic_sensor_id_map) {
    topics->emplace_back(artifact_topic_sensor_id_pair.first);
  }
}

void RosTopicSettings::makeAbsoluteTopics() {
  CameraTopicIdxMap camera_topics_tmp_map;
  for (const CameraTopicIdxMap::value_type& topic_idx :
       camera_topic_cam_index_map) {
    CHECK(!topic_idx.first.empty());
    topic_idx.first[0] == '/'
        ? camera_topics_tmp_map.emplace(topic_idx.first, topic_idx.second)
        : camera_topics_tmp_map.emplace(
              '/' + topic_idx.first, topic_idx.second);
  }
  camera_topic_cam_index_map.swap(camera_topics_tmp_map);

  if (!imu_topic.empty() && imu_topic[0] != '/') {
    imu_topic = '/' + imu_topic;
  }

  if (!relative_6dof_topic.empty() && relative_6dof_topic[0] != '/') {
    relative_6dof_topic = '/' + relative_6dof_topic;
  }

  if (!gps_wgs_topic.empty() && gps_wgs_topic[0] != '/') {
    gps_wgs_topic = '/' + gps_wgs_topic;
  }

  if (!gps_utm_topic.empty() && gps_utm_topic[0] != '/') {
    gps_utm_topic = '/' + gps_utm_topic;
  }

  TopicSensorIdMap lidar_topics_tmp_map;
  for (const TopicSensorIdMap::value_type& topic_sensor_id :
       lidar_topic_sensor_id_map) {
    CHECK(!topic_sensor_id.first.empty());
    topic_sensor_id.first[0] == '/'
        ? lidar_topics_tmp_map.emplace(
              topic_sensor_id.first, topic_sensor_id.second)
        : lidar_topics_tmp_map.emplace(
              '/' + topic_sensor_id.first, topic_sensor_id.second);
  }
  lidar_topic_sensor_id_map.swap(lidar_topics_tmp_map);

  if (!odometry_6dof_topic.empty() && odometry_6dof_topic[0] != '/') {
    odometry_6dof_topic = '/' + odometry_6dof_topic;
  }

  TopicSensorIdMap artifact_topics_tmp_map;
  for (const TopicSensorIdMap::value_type& topic_sensor_id :
       artifact_topic_sensor_id_map) {
    CHECK(!topic_sensor_id.first.empty());
    topic_sensor_id.first[0] == '/'
        ? artifact_topics_tmp_map.emplace(
              topic_sensor_id.first, topic_sensor_id.second)
        : artifact_topics_tmp_map.emplace(
              '/' + topic_sensor_id.first, topic_sensor_id.second);
  }
  artifact_topic_sensor_id_map.swap(artifact_topics_tmp_map);
}
}  // namespace vio_common
