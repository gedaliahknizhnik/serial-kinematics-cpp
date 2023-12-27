#ifndef SERIAL_KINEMATICS_HPP
#define SERIAL_KINEMATICS_HPP

#include <array>

#include "Eigen/Geometry"

namespace kinematics {

enum class Type { revolute, prismatic };

struct Joint {
  Type type{};
  double theta{};
  double a{};
  double d{};
  double alpha{};
};

template <int N>
struct DenhavitHartenbergParam {
  std::array<Joint, N> joints{};
};

Eigen::Matrix4d get_transform(const Joint joint, const double joint_angle);

}  // namespace kinematics

#endif
