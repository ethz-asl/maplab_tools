#ifndef SENSORS_ARTIFACT_DETECTION_H_
#define SENSORS_ARTIFACT_DETECTION_H_

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"
#include <cv_bridge/cv_bridge.h>

namespace vi_map {

class ArtifactDetection : public aslam::Sensor {
  public:
    MAPLAB_POINTER_TYPEDEFS(ArtifactDetection);

    ArtifactDetection();
    explicit ArtifactDetection(const aslam::SensorId& sensor_id,
        const std::string& topic);

    Sensor::Ptr cloneAsSensor() const {
      return std::dynamic_pointer_cast<Sensor>(
          aligned_shared<ArtifactDetection>(*this));
    }

    ArtifactDetection* cloneWithNewIds() const {
      ArtifactDetection* cloned_detection = new ArtifactDetection();
      *cloned_detection = *this;
      aslam::SensorId new_id;
      aslam::generateId(&new_id);
      cloned_detection->setId(new_id);
      return cloned_detection;
    }

    uint8_t getSensorType() const override {
      return SensorType::kArtifactDetection;
    }

    std::string getSensorTypeString() const override {
      return static_cast<std::string>(kOdometry6DoFIdentifier);
    }

  private:
    bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
    void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

    bool isValidImpl() const override {
      return true;
    }

    void setRandomImpl() override {}

    bool isEqualImpl(const aslam::Sensor&, 
        const bool = false) const override {
      return true;
    }

};

class ArtifactMeasurement final : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(ArtifactMeasurement);

  ArtifactMeasurement() = default;
  ArtifactMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds)
      : Measurement(sensor_id, timestamp_nanoseconds) {
    CHECK(isValid());
  }

  virtual ~ArtifactMeasurement() = default;

  bool operator==(const ArtifactMeasurement&) const {
    return true;
  }

  cv::Mat& getDetectionImage(); 
  const cv::Mat& getDetectionImage() const; 

  float& getDetectionProbability();
  float getDetectionProbability() const;

  std::string& getDetectedClassLabel();
  const std::string& getDetectedClassLabel() const;

  aslam::Transformation& getArtifactLocation();
  const aslam::Transformation& getArtifactLocation() const;

  int64_t& getTimestampNs();
  int64_t getTimestampNs() const;

 private:
  explicit ArtifactMeasurement(const aslam::SensorId& sensor_id)
      : Measurement(sensor_id) {}
  bool isValidImpl() const override;

  void setRandomImpl() override;

  cv::Mat detection_image_;
  float detection_probability_; 
  std::string class_label_;
  aslam::Transformation artifact_location_;
  int64_t timestamp_ns_;
};

} // namespace vi_map

#endif // SENSORS_ARTIFACT_DETECTION_H_ 
