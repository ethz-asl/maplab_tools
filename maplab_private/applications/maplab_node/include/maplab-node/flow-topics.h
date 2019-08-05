#ifndef MAPLAB_NODE_FLOW_TOPICS_H_
#define MAPLAB_NODE_FLOW_TOPICS_H_
#include <maplab-common/localization-result.h>
#include <message-flow/message-topic-registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensors/lidar.h>
#include <sensors/artifact-detection.h>
#include <vi-map/vi-map.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>
#include <vio-common/vio-update.h>
#include "maplab-node/odometry-estimate.h"
#include "maplab-node/vi-map-with-mutex.h"

// Raw sensor data.
MESSAGE_FLOW_TOPIC(IMAGE_MEASUREMENTS, vio::ImageMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(IMU_MEASUREMENTS, vio::ImuMeasurement::ConstPtr);
MESSAGE_FLOW_TOPIC(LIDAR_MEASUREMENTS, vi_map::RosLidarMeasurement::ConstPtr);

// Odometry input
MESSAGE_FLOW_TOPIC(ODOMETRY_ESTIMATES, maplab::OdometryEstimate::ConstPtr);

// Artifact detection.
MESSAGE_FLOW_TOPIC(ARTIFACT_DETECTION, vi_map::ArtifactMeasurement::ConstPtr);

// Synchronized images from multiple cameras
MESSAGE_FLOW_TOPIC(SYNCED_NFRAMES, vio::SynchronizedNFrame::Ptr);
// Same as synchronized nframes but also includes feature tracks.
MESSAGE_FLOW_TOPIC(TRACKED_NFRAMES, vio::SynchronizedNFrame::ConstPtr);

// Synchronized lidar measurements.
MESSAGE_FLOW_TOPIC(
    SYNCED_LIDAR_MEASUREMENTS, vi_map::RosLidarMeasurement::ConstPtr);

// Synchronized artifact detection measurements.
MESSAGE_FLOW_TOPIC(
    SYNCED_ARTIFACT_DETECTION, vi_map::ArtifactMeasurement::ConstPtr);

// Output of the localizer.
MESSAGE_FLOW_TOPIC(LOCALIZATION_RESULT, common::LocalizationResult::ConstPtr);

// Output of the localization handler, fused localization results from all
// sources.
MESSAGE_FLOW_TOPIC(
    FUSED_LOCALIZATION_RESULT, common::LocalizationResult::ConstPtr);

// Raw estimate of the VINS.
MESSAGE_FLOW_TOPIC(MAP_UPDATES, vio::MapUpdate::ConstPtr);

// Resulting map.
MESSAGE_FLOW_TOPIC(RAW_VIMAP, maplab::VIMapWithMutex::ConstPtr);

namespace maplab {
constexpr int kExclusivityGroupIdRawSensorDataSubscribers = 0;
}

#endif  // MAPLAB_NODE_FLOW_TOPICS_H_
