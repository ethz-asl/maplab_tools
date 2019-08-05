#include <aslam/common/yaml-serialization.h>
#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>

#include <sensors/relative-6dof-pose.h>

namespace vi_map {

Relative6DoF::Relative6DoF() : Sensor() {}

Relative6DoF::Relative6DoF(
    const aslam::SensorId& sensor_id, const std::string& topic)
    : Sensor(sensor_id, topic) {}

bool Relative6DoF::loadFromYamlNodeImpl(const YAML::Node& /*sensor_node*/) {
  return true;
}

void Relative6DoF::saveToYamlNodeImpl(YAML::Node* /*sensor_node*/) const {}

}  // namespace vi_map
