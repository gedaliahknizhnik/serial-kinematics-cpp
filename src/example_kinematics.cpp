#include <iostream>

#include "serial-kinematics/kinematics.hpp"

int main(int argc, char* argv[]) {
  kinematics::Joint joint{kinematics::Type::revolute, 0, 1, 0, 0};
  kinematics::DenhavitHartenbergParam<3> robots{
      std::array<kinematics::Joint, 3>{
          {{kinematics::Type::revolute, 0, 1, 0, 0},
           {kinematics::Type::revolute, 0, 2, 0, M_PI / 2},
           {kinematics::Type::prismatic, 0, 0.5, 0, 0}}}};

  std::cout << joint << "\n";
  std::cout << robots << "\n";

  return 0;
}
