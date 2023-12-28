#include "serial-kinematics/kinematics.hpp"

#include <Eigen/Geometry>

namespace kinematics {
Eigen::Matrix4d get_transform(const Joint joint, const double joint_angle) {
  Eigen::Matrix4d H{Eigen::Matrix4d::Zero()};

  // Joint variables
  double theta = joint.theta + joint_angle * (Type::revolute == joint.type);
  double d = joint.d + joint_angle * (Type::prismatic == joint.type);

  // Fill in rotation
  H(0, 0) = std::cos(theta);
  H(0, 1) = -std::sin(theta) * cos(joint.alpha);
  H(0, 2) = std::sin(theta) * sin(joint.alpha);
  H(1, 0) = std::sin(theta);
  H(1, 1) = std::cos(theta) * cos(joint.alpha);
  H(1, 2) = -std::cos(theta) * sin(joint.alpha);
  H(2, 0) = 0;
  H(2, 1) = std::sin(joint.alpha);
  H(2, 2) = std::cos(joint.alpha);

  // Fill in translation
  H(0, 3) = joint.a * std::cos(theta);
  H(1, 3) = joint.a * std::sin(theta);
  H(2, 3) = d;
  H(3, 3) = 1;

  return H;
}
}  // namespace kinematics