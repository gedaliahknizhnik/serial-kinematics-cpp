#ifndef SERIAL_KINEMATICS_HPP
#define SERIAL_KINEMATICS_HPP

namespace kinematics {

enum class Type { revolute, prismatic };

struct Joint {
  Type type{};
  double theta{};
  double a{};
  double d{};
  double alpha{};
};

}  // namespace kinematics

#endif
