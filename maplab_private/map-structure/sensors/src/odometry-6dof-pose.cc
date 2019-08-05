#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/odometry-6dof-pose.h>

namespace vi_map {

Odometry6DoF::Odometry6DoF() : Sensor() {}

Odometry6DoF::Odometry6DoF(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool Odometry6DoF::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}
void Odometry6DoF::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}

}  // namespace vi_map
