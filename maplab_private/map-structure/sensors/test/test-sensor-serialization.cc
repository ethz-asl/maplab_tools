#include <random>

#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "sensors/gps-utm.h"
#include "sensors/gps-wgs.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"
#include "sensors/relative-6dof-pose.h"

namespace vi_map {

constexpr char kSensorFileName[] = "sensor.yaml";

template <class DerivedSensor>
void testYamlSerializationDeserialization() {
  typename aslam::Sensor::UniquePtr sensor = aligned_unique<DerivedSensor>();
  sensor->setRandom();
  ASSERT_TRUE(static_cast<bool>(sensor));

  sensor->serializeToFile(static_cast<std::string>(kSensorFileName));

  typename aslam::Sensor::UniquePtr deserialized_sensor =
      aligned_unique<DerivedSensor>();
  deserialized_sensor->deserializeFromFile(kSensorFileName);

  EXPECT_EQ(*sensor, *deserialized_sensor);
}

TEST(SensorsTest, YamlSeriazliation) {
  testYamlSerializationDeserialization<Imu>();
  testYamlSerializationDeserialization<Relative6DoF>();
  testYamlSerializationDeserialization<GpsUtm>();
  testYamlSerializationDeserialization<GpsWgs>();
  testYamlSerializationDeserialization<Lidar>();
}

}  // namespace vi_map

MAPLAB_UNITTEST_ENTRYPOINT
