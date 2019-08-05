#ifndef MAP_RESOURCES_RESOURCE_CONVERSION_INL_H_
#define MAP_RESOURCES_RESOURCE_CONVERSION_INL_H_

#include <vector>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/distortion.h>
#include <glog/logging.h>
#include <maplab-common/pose_types.h>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensors/lidar.h>
#include <voxblox/core/common.h>

#include "map-resources/resource-typedefs.h"

namespace backend {

// Adds a point to a point cloud at a specific index. This function assumes that
// the point cloud has already been resized to allow for direct insertion at
// this index.
template <typename PointCloudType>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    PointCloudType* point_cloud) {
  LOG(FATAL) << "This point cloud either does not support color "
             << "or it is not implemented!";
}
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    resources::PointCloud* point_cloud);
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    voxblox::Pointcloud* point_cloud);
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    resources::VoxbloxColorPointCloud* point_cloud);
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    sensor_msgs::PointCloud2* point_cloud);
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    pcl::PointCloud<pcl::PointXYZRGB>* point_cloud);
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    pcl::PointCloud<pcl::PointXYZRGBA>* point_cloud);
template <>
void addColorToPointCloud(
    const resources::RgbaColor& color, const size_t index,
    pcl::PointCloud<pcl::PointXYZRGBNormal>* point_cloud);

template <typename PointCloudType>
void addScalarToPointCloud(
    const float scalar, const size_t index, PointCloudType* point_cloud) {
  LOG(FATAL) << "This point cloud either does not support scalars/intensities "
             << "or it is not implemented!";
}

template <>
void addScalarToPointCloud(
    const float scalar, const size_t index, resources::PointCloud* point_cloud);
template <>
void addScalarToPointCloud(
    const float /*scalar*/, const size_t /*index*/,
    resources::VoxbloxColorPointCloud* /*point_cloud*/);
template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    sensor_msgs::PointCloud2* point_cloud);
template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    pcl::PointCloud<pcl::PointXYZI>* point_cloud);
template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    pcl::PointCloud<pcl::PointXYZINormal>* point_cloud);
template <>
void addScalarToPointCloud(
    const float scalar, const size_t index,
    pcl::PointCloud<pcl::OusterPointType>* point_cloud);

template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    voxblox::Pointcloud* point_cloud);
template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::VoxbloxColorPointCloud* point_cloud);
template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    resources::PointCloud* point_cloud);
template <>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    sensor_msgs::PointCloud2* point_cloud);
template <typename PointType>
void addPointToPointCloud(
    const Eigen::Vector3d& point_C, const size_t index,
    pcl::PointCloud<PointType>* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_LT(index, point_cloud->points.size());
  PointType& point = point_cloud->points[index];

  // NOTE: There are PCL point types that do not have x,y and z, but if someone
  // ever uses one of those this should not compile anymore, so this should be
  // safe.
  point.x = static_cast<float>(point_C(0));
  point.y = static_cast<float>(point_C(1));
  point.z = static_cast<float>(point_C(2));
}

template <typename PointCloudType>
bool hasColorInformation(const PointCloudType& /*point_cloud*/) {
  return false;
}
template <>
bool hasColorInformation(const resources::VoxbloxColorPointCloud& point_cloud);
template <>
bool hasColorInformation(const resources::PointCloud& point_cloud);
template <>
bool hasColorInformation(const sensor_msgs::PointCloud2& point_cloud);
template <>
bool hasColorInformation(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>& point_cloud);
template <>
bool hasColorInformation(const pcl::PointCloud<pcl::PointXYZRGBA>& point_cloud);
template <>
bool hasColorInformation(const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);

template <typename PointCloudType>
bool hasNormalsInformation(const PointCloudType& /*point_cloud*/) {
  return false;
}
template <>
bool hasNormalsInformation(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>& point_cloud);
template <>
bool hasNormalsInformation(
    const pcl::PointCloud<pcl::PointXYZINormal>& point_cloud);

template <typename PointCloudType>
bool hasScalarInformation(const PointCloudType& /*point_cloud*/) {
  return false;
}
template <>
bool hasScalarInformation(const sensor_msgs::PointCloud2& point_cloud);
template <>
bool hasScalarInformation(const resources::PointCloud& point_cloud);
template <>
bool hasScalarInformation(const pcl::PointCloud<pcl::PointXYZI>& point_cloud);
template <>
bool hasScalarInformation(
    const pcl::PointCloud<pcl::PointXYZINormal>& point_cloud);
template <>
bool hasScalarInformation(
    const pcl::PointCloud<pcl::OusterPointType>& point_cloud);

template <>
void resizePointCloud(
    const size_t size, const bool /*has_color*/, const bool /*has_normals*/,
    const bool /*has_scalar*/, voxblox::Pointcloud* point_cloud);
template <>
void resizePointCloud(
    const size_t size, const bool has_color, const bool /*has_normals*/,
    const bool /*has_scalar*/, resources::VoxbloxColorPointCloud* point_cloud);
template <>
void resizePointCloud(
    const size_t size, const bool has_color, const bool has_normals,
    const bool has_scalar, resources::PointCloud* point_cloud);
template <>
void resizePointCloud(
    const size_t num_points, const bool has_color, const bool /*has_normals*/,
    const bool has_scalar, sensor_msgs::PointCloud2* point_cloud);
template <typename PointType>
void resizePointCloud(
    const size_t num_points, const bool /*has_color*/,
    const bool /*has_normals*/, const bool /*has_scalar*/,
    pcl::PointCloud<PointType>* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK_GT(num_points, 0u);
  point_cloud->points.resize(num_points);
}

template <>
size_t getPointCloudSize(const voxblox::Pointcloud& point_cloud);
template <>
size_t getPointCloudSize(const resources::VoxbloxColorPointCloud& point_cloud);
template <>
size_t getPointCloudSize(const resources::PointCloud& point_cloud);
template <>
size_t getPointCloudSize(const sensor_msgs::PointCloud2& point_cloud);
template <typename PointType>
size_t getPointCloudSize(const pcl::PointCloud<PointType>& point_cloud) {
  return point_cloud.size();
}

template <typename PointCloudType>
void getPointFromPointCloud(
    const PointCloudType& point_cloud, const size_t index,
    Eigen::Vector3d* point_C);
template <>
void getPointFromPointCloud(
    const voxblox::Pointcloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C);
template <>
void getPointFromPointCloud(
    const resources::VoxbloxColorPointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C);
template <>
void getPointFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    Eigen::Vector3d* point_C);
template <>
void getPointFromPointCloud(
    const sensor_msgs::PointCloud2& point_cloud, const size_t index,
    Eigen::Vector3d* point_C);
template <typename PointType>
void getPointFromPointCloud(
    const pcl::PointCloud<PointType>& point_cloud, const size_t index,
    Eigen::Vector3d* point_C) {
  CHECK_NOTNULL(point_C);
  DCHECK_GT(point_cloud.size(), index);
  const PointType& point = point_cloud.points[index];

  // NOTE: There are PCL point types that do not have x,y and z, but if someone
  // ever uses one of those this should not compile anymore, so this should be
  // safe.
  point_C->x() = point.x;
  point_C->y() = point.y;
  point_C->z() = point.z;
}

template <typename PointCloudType>
void getColorFromPointCloud(
    const PointCloudType& /*point_cloud*/, const size_t /*index*/,
    resources::RgbaColor* /*color*/) {
  LOG(FATAL) << "This point cloud either does not support color or it is not "
             << "implemented!";
}
template <>
void getColorFromPointCloud(
    const resources::VoxbloxColorPointCloud& point_cloud, const size_t index,
    resources::RgbaColor* color);
template <>
void getColorFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    resources::RgbaColor* color);
template <>
void getColorFromPointCloud(
    const sensor_msgs::PointCloud2& point_cloud, const size_t index,
    resources::RgbaColor* color);
template <>
void getColorFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud, const size_t index,
    resources::RgbaColor* color);
template <>
void getColorFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGBA>& point_cloud, const size_t index,
    resources::RgbaColor* color);
template <>
void getColorFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>& point_cloud,
    const size_t index, resources::RgbaColor* color);

template <typename PointCloudType>
void getScalarFromPointCloud(
    const PointCloudType& /*point_cloud*/, const size_t /*index*/,
    float* /*scalar*/) {
  LOG(FATAL) << "This point cloud either does not support scalars/intesities "
                "or it is not "
             << "implemented!";
}
template <>
void getScalarFromPointCloud(
    const resources::PointCloud& point_cloud, const size_t index,
    float* scalar);
template <>
void getScalarFromPointCloud(
    const sensor_msgs::PointCloud2& point_cloud, const size_t index,
    float* scalar);
template <>
void getScalarFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>& point_cloud, const size_t index,
    float* scalar);
template <>
void getScalarFromPointCloud(
    const pcl::PointCloud<pcl::PointXYZINormal>& point_cloud,
    const size_t index, float* scalar);
template <>
void getScalarFromPointCloud(
    const pcl::PointCloud<pcl::OusterPointType>& point_cloud,
    const size_t index, float* scalar);

template <typename PointCloudType>
bool convertDepthMapToPointCloud(
    const cv::Mat& depth_map, const cv::Mat& image, const aslam::Camera& camera,
    PointCloudType* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  CHECK(!depth_map.empty());
  CHECK_GT(depth_map.rows, 0);
  CHECK_GT(depth_map.cols, 0);
  CHECK_EQ(CV_MAT_TYPE(depth_map.type()), CV_16U);
  // Image should either be grayscale 8bit image or color 3x8bit.
  CHECK(
      CV_MAT_TYPE(image.type()) == CV_8UC1 ||
      CV_MAT_TYPE(image.type()) == CV_8UC3);

  const bool has_image =
      (depth_map.rows == image.rows) && (depth_map.cols == image.cols);

  const bool has_three_channels = CV_MAT_TYPE(image.type()) == CV_8UC3;

  const size_t valid_depth_entries = cv::countNonZero(depth_map);

  if (valid_depth_entries == 0u) {
    VLOG(3) << "Depth map has no valid depth measurements!";
    return false;
  }

  const bool has_color = has_image;
  constexpr bool kHasNormals = false;
  constexpr bool kHasScalar = false;

  resizePointCloud(
      valid_depth_entries, has_color, kHasNormals, kHasScalar, point_cloud);

  constexpr double kMillimetersToMeters = 1e-3;
  constexpr double kEpsilon = 1e-6;

  resources::RgbaColor color(255u, 255u, 255u, 255u);
  const uint16_t* depth_map_ptr;
  size_t point_index = 0u;
  for (int v = 0; v < depth_map.rows; ++v) {
    depth_map_ptr = depth_map.ptr<uint16_t>(v);
    for (int u = 0; u < depth_map.cols; ++u) {
      const uint16_t depth = depth_map_ptr[u];
      if (depth == 0u) {
        continue;
      }

      Eigen::Vector3d point_C;
      Eigen::Vector2d image_point;
      image_point << u, v;
      camera.backProject3(image_point, &point_C);
      const double depth_in_meters =
          static_cast<double>(depth) * kMillimetersToMeters;

      if (point_C.z() < kEpsilon) {
        continue;
      }

      point_C /= point_C.z();
      point_C = depth_in_meters * point_C;

      if (has_image) {
        if (has_three_channels) {
          const cv::Vec3b& cv_color = image.at<cv::Vec3b>(v, u);
          // NOTE: Assumes the image are stored as BGR.
          color[0] = cv_color[2];
          color[1] = cv_color[1];
          color[2] = cv_color[0];
        } else {
          color[0] = image.at<uint8_t>(v, u);
          color[1] = color[0];
          color[2] = color[0];
        }
        color[3] = 255u;
      }

      CHECK_LT(point_index, valid_depth_entries);
      addPointToPointCloud(point_C, point_index, point_cloud);
      if (has_color) {
        addColorToPointCloud(color, point_index, point_cloud);
      }

      ++point_index;
    }
  }
  VLOG(3) << "Converted depth map to a point cloud of size " << point_index
          << ".";

  // Shrink pointcloud if necessary.
  resizePointCloud(
      point_index, has_color, kHasNormals, kHasScalar, point_cloud);

  if (point_index == 0u) {
    VLOG(3) << "Depth map has no valid depth measurements!";
    return false;
  }

  return true;
}

template <typename InputPointCloud, typename OutputPointCloud>
bool convertPointCloudType(
    const InputPointCloud& input_cloud, OutputPointCloud* output_cloud) {
  CHECK_NOTNULL(output_cloud);

  const bool input_has_normals = hasNormalsInformation(input_cloud);
  const bool input_has_scalars = hasScalarInformation(input_cloud);
  const bool input_has_color = hasColorInformation(input_cloud);

  const size_t num_points = getPointCloudSize(input_cloud);

  resizePointCloud(
      num_points, input_has_color, input_has_normals, input_has_scalars,
      output_cloud);

  const bool output_has_scalars = hasScalarInformation(*output_cloud);
  const bool output_has_color = hasColorInformation(*output_cloud);

  for (size_t point_idx = 0u; point_idx < num_points; ++point_idx) {
    Eigen::Vector3d point_C;
    getPointFromPointCloud(input_cloud, point_idx, &point_C);
    addPointToPointCloud(point_C, point_idx, output_cloud);

    if (input_has_color && output_has_color) {
      resources::RgbaColor color;
      getColorFromPointCloud(input_cloud, point_idx, &color);
      addColorToPointCloud(color, point_idx, output_cloud);
    }

    if (input_has_scalars && output_has_scalars) {
      float scalar;
      getScalarFromPointCloud(input_cloud, point_idx, &scalar);
      addScalarToPointCloud(scalar, point_idx, output_cloud);
    }
  }

  CHECK_EQ(getPointCloudSize(*output_cloud), num_points);
  return true;
}

template <typename PointCloudType>
backend::ResourceType getResourceTypeForPointCloud(
    const PointCloudType& point_cloud) {
  CHECK_NE(getPointCloudSize(point_cloud), 0u)
      << "Cannot determine resource type if point cloud is empty!";

  const bool has_normals = hasNormalsInformation(point_cloud);
  const bool has_scalars = hasScalarInformation(point_cloud);
  const bool has_color = hasColorInformation(point_cloud);

  if (has_color && has_normals && !has_scalars) {
    return backend::ResourceType::kPointCloudXYZRGBN;
  } else if (!has_color && !has_normals && has_scalars) {
    return backend::ResourceType::kPointCloudXYZI;
  } else if (!has_color && !has_normals && !has_scalars) {
    return backend::ResourceType::kPointCloudXYZ;
  }
  LOG(FATAL) << "Currently there is no point cloud type implemented as "
             << "resource that has this particular configuration of color("
             << has_color << "), normals(" << has_normals << ") and scalar("
             << has_scalars << ") data!";
}

}  // namespace backend

#endif  // MAP_RESOURCES_RESOURCE_CONVERSION_INL_H_