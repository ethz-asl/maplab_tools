#include <sensors/artifact-detection.h>

#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>


namespace vi_map {

ArtifactDetection::ArtifactDetection() : Sensor() {}

ArtifactDetection::ArtifactDetection(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool ArtifactDetection::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}

void ArtifactDetection::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}
  // noop

bool ArtifactMeasurement::isValidImpl() const {
  return true;
}

void ArtifactMeasurement::setRandomImpl() {
  // TODO(lbern): implement or remove. We could use the
  // general point cloud conversion tools to make one
  // function to fill all point cloud types randomly.
  LOG(FATAL) << "NOT IMPLEMENTED!";
}

cv::Mat& ArtifactMeasurement::getDetectionImage() {
  return detection_image_;
}

const cv::Mat& ArtifactMeasurement::getDetectionImage() const {
  return detection_image_;
}

float& ArtifactMeasurement::getDetectionProbability() {
  return detection_probability_;
}

float ArtifactMeasurement::getDetectionProbability() const {
  return detection_probability_;
}

std::string& ArtifactMeasurement::getDetectedClassLabel() {
  return class_label_;
}

const std::string& ArtifactMeasurement::getDetectedClassLabel() const {
  return class_label_;
}

aslam::Transformation& ArtifactMeasurement::getArtifactLocation() {
  return artifact_location_;
}

const aslam::Transformation& ArtifactMeasurement::getArtifactLocation() const {
  return artifact_location_;
}

int64_t& ArtifactMeasurement::getTimestampNs() {
  return timestamp_ns_;
}

int64_t ArtifactMeasurement::getTimestampNs() const {
  return timestamp_ns_;
}

} // namespace vi_map
