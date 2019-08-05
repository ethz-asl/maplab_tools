#include "vi-map/sensor-utils.h"

#include <memory>

#include <aslam/common/sensor.h>
#include <sensors/imu.h>
#include <sensors/lidar.h>
#include <sensors/sensor-types.h>

#include "vi-map/sensor-manager.h"

DEFINE_string(
    selected_ncamera_sensor_id, "",
    "If there is more than one NCamera in the sensor manager, use this flag "
    "to determine which one is used.");

DEFINE_string(
    selected_imu_sensor_id, "",
    "If there is more than one IMU in the sensor manager, use this flag "
    "to determine which one is used.");

DEFINE_string(
    selected_lidar_sensor_id, "",
    "If there is more than one Lidar in the sensor manager, use this flag "
    "to determine which one is used.");

DEFINE_string(
    selected_odometry_6dof_sensor_id, "",
    "If there is more than one 6DOF Odometry sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_artifact_sensor_id, "",
    "If there is more than one artifact detection sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_relative_6dof_sensor_id, "",
    "If there is more than one Relative6DOF sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_gps_utm_sensor_id, "",
    "If there is more than one GPS UTM sensor in the sensor manager, use "
    "this flag to determine which one is used.");

DEFINE_string(
    selected_gps_wgs_sensor_id, "",
    "If there is more than one GPS WGS sensor in the sensor manager, use "
    "this flag to determine which one is used.");

namespace vi_map {

aslam::NCamera::Ptr getSelectedNCamera(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_ncamera_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kNCamera, &all_ncamera_ids);

  if (all_ncamera_ids.empty()) {
    VLOG(1) << "No NCameras were found in sensor manager!";
    return aslam::NCamera::Ptr();
  }

  aslam::SensorId mapping_ncamera_id;
  if (all_ncamera_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_ncamera_sensor_id.empty() &&
        mapping_ncamera_id.fromHexString(FLAGS_selected_ncamera_sensor_id))
        << "If more than one NCamera is provided in the sensor manager, use "
        << "--selected_ncamera_sensor_id to select which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_ncamera_id) &&
        (sensor_manager.getSensorType(mapping_ncamera_id) ==
         SensorType::kNCamera))
        << "The sensor id provided by --selected_ncamera_sensor_id is not "
        << "in the sensor manager or is not an NCamera!";
  } else {
    mapping_ncamera_id = *all_ncamera_ids.begin();
  }

  aslam::NCamera::Ptr mapping_ncamera_ptr =
      sensor_manager.getSensorPtr<aslam::NCamera>(mapping_ncamera_id);
  CHECK(mapping_ncamera_ptr);
  return mapping_ncamera_ptr;
}

Imu::Ptr getSelectedImu(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_imu_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kImu, &all_imu_ids);

  if (all_imu_ids.empty()) {
    VLOG(1) << "No Imu was found in the sensor manager!";
    return Imu::Ptr();
  }

  aslam::SensorId mapping_imu_id;
  if (all_imu_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_imu_sensor_id.empty() &&
        mapping_imu_id.fromHexString(FLAGS_selected_imu_sensor_id))
        << "If more than one Imu is provided in the sensor manager, use "
        << "--selected_imu_sensor_id to select which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_imu_id) &&
        (sensor_manager.getSensorType(mapping_imu_id) == SensorType::kImu))
        << "The sensor id provided by --selected_imu_sensor_id is not "
        << "in the sensor manager or is not an Imu!";
  } else {
    mapping_imu_id = *all_imu_ids.begin();
  }

  Imu::Ptr mapping_imu_ptr = sensor_manager.getSensorPtr<Imu>(mapping_imu_id);
  CHECK(mapping_imu_ptr);
  return mapping_imu_ptr;
}

Lidar::Ptr getSelectedLidar(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_lidar_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kLidar, &all_lidar_ids);

  if (all_lidar_ids.empty()) {
    VLOG(1) << "No Lidars found in sensor manager!";
    return Lidar::Ptr();
  }

  aslam::SensorId mapping_lidar_id;
  if (all_lidar_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_lidar_sensor_id.empty() &&
        mapping_lidar_id.fromHexString(FLAGS_selected_lidar_sensor_id))
        << "If more than one Lidar is provided in the sensor manager, use "
        << "--selected_lidar_sensor_id to select which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_lidar_id) &&
        (sensor_manager.getSensorType(mapping_lidar_id) == SensorType::kLidar))
        << "The sensor id provided by --selected_lidar_sensor_id is not "
        << "in the sensor manager or is not a Lidar!";
  } else {
    mapping_lidar_id = *all_lidar_ids.begin();
  }

  Lidar::Ptr mapping_lidar_ptr =
      sensor_manager.getSensorPtr<Lidar>(mapping_lidar_id);
  CHECK(mapping_lidar_ptr);
  return mapping_lidar_ptr;
}

Odometry6DoF::Ptr getSelectedOdometry6DoFSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_odometry_6dof_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kOdometry6DoF, &all_odometry_6dof_ids);

  if (all_odometry_6dof_ids.empty()) {
    VLOG(1) << "No Odometry6DoF sensors found in sensor manager!";
    return Odometry6DoF::Ptr();
  }

  aslam::SensorId mapping_odometry_6dof_id;
  if (all_odometry_6dof_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_odometry_6dof_sensor_id.empty() &&
        mapping_odometry_6dof_id.fromHexString(
            FLAGS_selected_odometry_6dof_sensor_id))
        << "If more than one Odometry6DoF sensor is provided in the "
        << "sensor manager, use --selected_odometry_6dof_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_odometry_6dof_id) &&
        (sensor_manager.getSensorType(mapping_odometry_6dof_id) ==
         SensorType::kOdometry6DoF))
        << "The sensor id provided by --selected_odometry_6dof_sensor_id is "
        << "not in the sensor manager or is not a Odometry6DoF sensor!";
  } else {
    mapping_odometry_6dof_id = *all_odometry_6dof_ids.begin();
  }

  Odometry6DoF::Ptr mapping_odometry_6dof_ptr =
      sensor_manager.getSensorPtr<Odometry6DoF>(mapping_odometry_6dof_id);
  CHECK(mapping_odometry_6dof_ptr);
  return mapping_odometry_6dof_ptr;
}

ArtifactDetection::Ptr getSelectedArtifactDetectionSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_artifact_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kArtifactDetection, &all_artifact_ids);

  if (all_artifact_ids.empty()) {
    VLOG(1) << "No artifact detection sensors found in sensor manager!";
    return ArtifactDetection::Ptr();
  }

  aslam::SensorId mapping_artifact_id;
  if (all_artifact_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_artifact_sensor_id.empty() &&
        mapping_artifact_id.fromHexString(
            FLAGS_selected_artifact_sensor_id))
        << "If more than one artifact detection sensor is provided in the "
        << "sensor manager, use --selected_artifact_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_artifact_id) &&
        (sensor_manager.getSensorType(mapping_artifact_id) ==
         SensorType::kArtifactDetection))
        << "The sensor id provided by --selected_artifact_sensor_id is "
        << "not in the sensor manager or is not a artifact detection sensor!";
  } else {
    mapping_artifact_id = *all_artifact_ids.begin();
  }

  ArtifactDetection::Ptr mapping_artifact_ptr =
      sensor_manager.getSensorPtr<ArtifactDetection>(mapping_artifact_id);
  CHECK(mapping_artifact_ptr);
  return mapping_artifact_ptr;
}

Relative6DoF::Ptr getSelectedRelative6DoFSensor(
    const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_relative_6dof_ids;
  sensor_manager.getAllSensorIdsOfType(
      SensorType::kRelative6DoF, &all_relative_6dof_ids);

  if (all_relative_6dof_ids.empty()) {
    VLOG(1) << "No Relative6DoF sensors found in sensor manager!";
    return Relative6DoF::Ptr();
  }

  aslam::SensorId mapping_relative_6dof_id;
  if (all_relative_6dof_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_relative_6dof_sensor_id.empty() &&
        mapping_relative_6dof_id.fromHexString(
            FLAGS_selected_relative_6dof_sensor_id))
        << "If more than one Relative6DoF sensor is provided in the "
        << "sensor manager, use --selected_relative_6dof_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_relative_6dof_id) &&
        (sensor_manager.getSensorType(mapping_relative_6dof_id) ==
         SensorType::kRelative6DoF))
        << "The sensor id provided by --selected_relative_6dof_sensor_id is "
        << "not in the sensor manager or is not a Relative6DoF sensor!";
  } else {
    mapping_relative_6dof_id = *all_relative_6dof_ids.begin();
  }

  Relative6DoF::Ptr mapping_relative_6dof_ptr =
      sensor_manager.getSensorPtr<Relative6DoF>(mapping_relative_6dof_id);
  CHECK(mapping_relative_6dof_ptr);
  return mapping_relative_6dof_ptr;
}

GpsUtm::Ptr getSelectedGpsUtmSensor(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_gps_utm_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kGpsUtm, &all_gps_utm_ids);

  if (all_gps_utm_ids.empty()) {
    VLOG(1) << "No GpsUtm sensors found in sensor manager!";
    return GpsUtm::Ptr();
  }

  aslam::SensorId mapping_gps_utm_id;
  if (all_gps_utm_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_gps_utm_sensor_id.empty() &&
        mapping_gps_utm_id.fromHexString(FLAGS_selected_gps_utm_sensor_id))
        << "If more than one GpsUtm sensor is provided in the "
        << "sensor manager, use --selected_gps_utm_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_gps_utm_id) &&
        (sensor_manager.getSensorType(mapping_gps_utm_id) ==
         SensorType::kGpsUtm))
        << "The sensor id provided by --selected_gps_utm_sensor_id is "
        << "not in the sensor manager or is not a GpsUtm sensor!";
  } else {
    mapping_gps_utm_id = *all_gps_utm_ids.begin();
  }

  GpsUtm::Ptr mapping_gps_utm_ptr =
      sensor_manager.getSensorPtr<GpsUtm>(mapping_gps_utm_id);
  CHECK(mapping_gps_utm_ptr);
  return mapping_gps_utm_ptr;
}

GpsWgs::Ptr getSelectedGpsWgsSensor(const SensorManager& sensor_manager) {
  aslam::SensorIdSet all_gps_utm_ids;
  sensor_manager.getAllSensorIdsOfType(SensorType::kGpsWgs, &all_gps_utm_ids);

  if (all_gps_utm_ids.empty()) {
    VLOG(1) << "No GpsWgs sensors found in sensor manager!";
    return GpsWgs::Ptr();
  }

  aslam::SensorId mapping_gps_utm_id;
  if (all_gps_utm_ids.size() > 1u) {
    CHECK(
        !FLAGS_selected_gps_utm_sensor_id.empty() &&
        mapping_gps_utm_id.fromHexString(FLAGS_selected_gps_utm_sensor_id))
        << "If more than one GpsWgs sensor is provided in the "
        << "sensor manager, use --selected_gps_utm_sensor_id to select "
        << "which one to use.";

    CHECK(
        sensor_manager.hasSensor(mapping_gps_utm_id) &&
        (sensor_manager.getSensorType(mapping_gps_utm_id) ==
         SensorType::kGpsWgs))
        << "The sensor id provided by --selected_gps_utm_sensor_id is "
        << "not in the sensor manager or is not a GpsWgs sensor!";
  } else {
    mapping_gps_utm_id = *all_gps_utm_ids.begin();
  }

  GpsWgs::Ptr mapping_gps_utm_ptr =
      sensor_manager.getSensorPtr<GpsWgs>(mapping_gps_utm_id);
  CHECK(mapping_gps_utm_ptr);
  return mapping_gps_utm_ptr;
}

}  // namespace vi_map
