#ifndef MAPLAB_LIDAR_IMAGE_PROJECTION_ROTATION_UTILS_H_
#define MAPLAB_LIDAR_IMAGE_PROJECTION_ROTATION_UTILS_H_

#include <Eigen/Dense>

namespace maplab {

class TransformationUtils {
 public:
  TransformationUtils() = delete;

  static Eigen::Matrix4d CreateTransformation(
      const double alpha_rad, const double beta_rad, const double gamma_rad);

  static Eigen::Matrix4d CreateTransformationAroundX(const double alpha_rad);
  static Eigen::Matrix4d CreateTransformationAroundY(const double beta_rad);
  static Eigen::Matrix4d CreateTransformationAroundZ(const double gamma_rad);
};

}  // namespace maplab

#endif
