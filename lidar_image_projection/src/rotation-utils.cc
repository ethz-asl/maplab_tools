#include "lidar-image-projection/rotation-utils.h"

namespace maplab {

Eigen::Matrix4d RotationUtils::CreateTransformation(const double alpha_rad, 
    const double beta_rad, const double gamma_rad) {
  return CreateTransformationAroundZ(gamma_rad) 
    * CreateTransformationAroundY(beta_rad)
    * CreateTransformationAroundX(alpha_rad);
}

Eigen::Matrix4d RotationUtils::CreateTransformationAroundX(
    const double alpha_rad) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(1, 1) = std::cos(alpha_rad);
  T(1, 2) = -std::sin(alpha_rad);
  T(2, 1) = std::sin(alpha_rad);
  T(2, 2) = std::cos(alpha_rad);

  return T;
}

Eigen::Matrix4d RotationUtils::CreateTransformationAroundY(
    const double beta_rad) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = std::cos(beta_rad);
  T(0, 2) = std::sin(beta_rad);
  T(2, 0) = -std::sin(beta_rad);
  T(2, 2) = std::cos(beta_rad);

  return T;
}

Eigen::Matrix4d RotationUtils::CreateTransformationAroundZ(
    const double gamma_rad) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = std::cos(gamma_rad);
  T(0, 1) = -std::sin(gamma_rad);
  T(1, 0) = std::sin(gamma_rad);
  T(1, 1) = std::cos(gamma_rad);

  return T;
}

}  // maplab
