#ifndef SENSORS_RELATIVE_6DOF_POSE_H_
#define SENSORS_RELATIVE_6DOF_POSE_H_

#include <string>

#include <aslam/common/sensor.h>
#include <maplab-common/macros.h>
#include <yaml-cpp/yaml.h>

#include "sensors/measurement.h"
#include "sensors/sensor-types.h"

namespace vi_map {

class Relative6DoF final : public aslam::Sensor {
 public:
  MAPLAB_POINTER_TYPEDEFS(Relative6DoF);

  Relative6DoF();
  explicit Relative6DoF(
      const aslam::SensorId& sensor_id, const std::string& topic);

  Sensor::Ptr cloneAsSensor() const {
    return std::dynamic_pointer_cast<Sensor>(
        aligned_shared<Relative6DoF>(*this));
  }

  Relative6DoF* cloneWithNewIds() const {
    Relative6DoF* cloned_relative_sensor = new Relative6DoF();
    *cloned_relative_sensor = *this;
    aslam::SensorId new_id;
    aslam::generateId(&new_id);
    cloned_relative_sensor->setId(new_id);
    return cloned_relative_sensor;
  }

  uint8_t getSensorType() const override {
    return SensorType::kRelative6DoF;
  }

  std::string getSensorTypeString() const override {
    return static_cast<std::string>(kRelative6DoFIdentifier);
  }

 private:
  bool loadFromYamlNodeImpl(const YAML::Node& sensor_node) override;
  void saveToYamlNodeImpl(YAML::Node* sensor_node) const override;

  bool isValidImpl() const override {
    return true;
  }

  void setRandomImpl() override {}

  bool isEqualImpl(
      const Sensor& /*other*/, const bool /*verbose*/) const override {
    return true;
  }
};

class Relative6DoFMeasurement : public Measurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(Relative6DoFMeasurement);

  Relative6DoFMeasurement() {
    T_B_A_.setIdentity();
    measurement_covariance_.setZero();
  }

  explicit Relative6DoFMeasurement(
      const aslam::SensorId& sensor_id, const int64_t timestamp_nanoseconds,
      const aslam::Transformation& T_B_A,
      const aslam::TransformationCovariance& covariance)
      : Measurement(sensor_id, timestamp_nanoseconds),
        T_B_A_(T_B_A),
        measurement_covariance_(covariance) {}

  const aslam::Transformation& get_T_B_A() const {
    return T_B_A_;
  }

  void set_T_B_A(const aslam::Transformation& T_B_A) {
    T_B_A_ = T_B_A;
  }

  const aslam::TransformationCovariance& getMeasurementCovariance() const {
    CHECK(!measurement_covariance_.isZero());
    return measurement_covariance_;
  }

 private:
  bool isValidImpl() const {
    return true;
  }

  void setRandomImpl() override {
    T_B_A_.setRandom();
    measurement_covariance_.setRandom();
  }

  aslam::Transformation T_B_A_;
  aslam::TransformationCovariance measurement_covariance_;
};

DEFINE_MEAUREMENT_CONTAINERS(Relative6DoFMeasurement)

}  // namespace vi_map

DEFINE_MEASUREMENT_HASH(Relative6DoFMeasurement)

#endif  // SENSORS_RELATIVE_6DOF_POSE_H_
