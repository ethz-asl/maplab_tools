#include "vi-map-data-import-export/export-artifact-detections.h"
#include <console-common/command-registerer.h>
#include <maplab-common/file-system-tools.h>
#include <sensors/sensor-types.h>
#include <maplab-common/progress-bar.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <resources-common/artifact.h>
#include <sstream>

namespace data_import_export {

static bool getPosesWithTimestamps(
    const vi_map::VIMission& mission, const vi_map::VIMap& map,
    aslam::TransformationVector* poses_M_B,
    const common::TemporalBuffer<backend::ResourceId>::BufferType&
        resource_buffer) {
  // Check if there is IMU data to interpolate the optional sensor poses.
  const vi_map::MissionId& mission_id = mission.id();
  landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
  int64_t min_timestamp_ns;
  int64_t max_timestamp_ns;
  const landmark_triangulation::PoseInterpolator pose_interpolator;
  pose_interpolator.getVertexToTimeStampMap(
      map, mission_id, &vertex_to_time_map, &min_timestamp_ns,
      &max_timestamp_ns);
  if (vertex_to_time_map.empty()) {
    VLOG(2) << "Couldn't find any IMU data to interpolate exact optional "
            << "sensor position in mission " << mission_id;
    return false;
  }

  LOG(INFO) << "All resources within this time range will be integrated: ["
            << min_timestamp_ns << "," << max_timestamp_ns << "]";

  // Collect all timestamps that need to be interpolated.
  const std::size_t num_resources = resource_buffer.size();
  Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(num_resources);
  std::size_t idx = 0u;
  for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
       resource_buffer) {
    // If the resource timestamp does not lie within the min and max
    // timestamp of the vertices, we cannot interpolate the position. To
    // keep this efficient, we simply replace timestamps outside the range
    // with the min or max. Since their transformation will not be used
    // later, that's fine.
    resource_timestamps[idx] = std::max(
        min_timestamp_ns,
        std::min(max_timestamp_ns, stamped_resource_id.first));
    ++idx;
  }

  // Interpolate poses at resource timestamp.
  pose_interpolator.getPosesAtTime(
      map, mission_id, resource_timestamps, poses_M_B);
  return true;
}

static void reprojectResource(
    const vi_map::VIMission& mission, const vi_map::VIMap& map,
    const aslam::SensorId& sensor_id, const backend::ResourceType type,
    const aslam::TransformationVector& poses_M_B,
    const common::TemporalBuffer<backend::ResourceId>::BufferType&
        resource_buffer, std::ostream& out) {
  const aslam::Transformation& T_G_M =
      map.getMissionBaseFrameForMission(mission.id()).get_T_G_M();
  const aslam::Transformation& T_B_S =
      map.getSensorManager().getSensor_T_B_S(sensor_id);

  // Retrieve and integrate all resources.
  std::size_t idx = 0u;
  common::ProgressBar progress_bar(resource_buffer.size());
  for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
       resource_buffer) {
    progress_bar.increment();

    const aslam::Transformation& T_M_B = poses_M_B[idx];
    const aslam::Transformation T_G_S = T_G_M * T_M_B * T_B_S;
    ++idx;

    resources::Artifact resource;
    const int64_t timestamp_ns = stamped_resource_id.first;
    const int64_t tolerance_ns = 1e6;  // 10ms

    // Retrieve the resource at the timestamp.
    // try to find the closest one if there is no exact match.
    if (!map.hasSensorResource(mission, type, sensor_id, timestamp_ns)) {
      LOG(WARNING)
          << "Unable to find an associated resource "
          << "at the exact timestamp. Trying to find the closest resource.";
      int64_t closest_timestamp_ns = -1;
      map.getClosestSensorResource(
          mission, type, sensor_id, timestamp_ns, tolerance_ns, &resource,
          &closest_timestamp_ns);
      if (closest_timestamp_ns == -1) {
        LOG(ERROR)
            << "Unable to fidn the closest resource within the tolerance."
            << " Skipping this resource.";
      }
      LOG(WARNING) << "Found closest resource with difference: "
                   << timestamp_ns - closest_timestamp_ns;
    } else {
      map.getSensorResource(mission, type, sensor_id, timestamp_ns, &resource);
    }
    const std::vector<double>& vec = resource.T_S_Artifact;
    CHECK(vec.size() == 7u);
    aslam::Transformation T_S_Artifact(
        aslam::Quaternion(vec[0], vec[1], vec[2], vec[3]),
        aslam::Position3D(vec[4], vec[5], vec[6]));
    const aslam::Transformation T_G_Artifact = T_G_S * T_S_Artifact;

    out << resource.class_id << "\n" << T_G_Artifact << "\n";
  }
}

int exportArtifactDetections(
    const vi_map::MissionIdList& mission_ids, const vi_map::VIMap& map,
    const std::string& output_folder) {
  CHECK(!output_folder.empty());

  if (!common::createPath(output_folder)) {
    LOG(ERROR) << "Failed to create folder: " << output_folder << ".";
    return common::kStupidUserError;
  }

  std::stringstream ss;
  const backend::ResourceType artifact_type =
      backend::ResourceType::kArtifactDetection;
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::VIMission& mission = map.getMission(mission_id);
    const aslam::SensorId& artifact_sensor =
        mission.getArtifactDetectionSensor();
    // Collect the timestamps for the resources and find the associated poses.
    aslam::TransformationVector poses_M_B;
    const backend::TemporalResourceIdBuffer* temporal_resource_buffer =
        mission.getAllSensorResourceIdsForSensorOfType(
            artifact_type, artifact_sensor);
    const common::TemporalBuffer<backend::ResourceId>::BufferType&
        resource_buffer = temporal_resource_buffer->resource_id_map();

    if (!getPosesWithTimestamps(mission, map, &poses_M_B, resource_buffer)) {
      continue;
    }

    CHECK_EQ(poses_M_B.size(), resource_buffer.size());
    reprojectResource(
        mission, map, artifact_sensor, artifact_type, poses_M_B,
        resource_buffer, ss);
  }
  const std::string file = output_folder+"artifacts.txt";
  VLOG(1) << "writing artifacts file to: " file;
  std::ofstream out_file(file, std::ofstream::out);
  out_file << ss.str();
  VLOG(1) << "Contents: " << ss.str();
  out_file.close();
  return common::kSuccess;
}
}  // namespace data_import_export
