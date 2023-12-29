#ifndef SERIAL_KINEMATICS_HPP
#define SERIAL_KINEMATICS_HPP

#include <array>
#include <stdexcept>

#include "Eigen/Geometry"

namespace kinematics {

enum class Type { revolute, prismatic, not_a_joint };
std::string type_to_string(const Type& type);

struct Joint {
  Type type{Type::not_a_joint};
  double theta{};
  double a{};
  double d{};
  double alpha{};
};

std::ostream& operator<<(std::ostream& out, const Joint& joint);

struct DenhavitHartenbergParam {
  std::vector<Joint> joints{};

  Joint& operator[](const int index) { return joints[index]; }
};

std::ostream& operator<<(std::ostream& out,
                         const DenhavitHartenbergParam& params);

Eigen::Matrix4d get_transform(const Joint joint, const double joint_angle);
}  // namespace kinematics

#endif
