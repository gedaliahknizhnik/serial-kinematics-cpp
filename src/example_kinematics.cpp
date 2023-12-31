#include <iostream>

#include "serial-kinematics/kinematics.hpp"

int main(int argc, char* argv[]) {
  kinematics::DenhavitHartenbergParam robots{
      {{{kinematics::Type::revolute, 0, 1, 0, 0},
        {kinematics::Type::revolute, 0, 2, 0, M_PI / 2},
        {kinematics::Type::prismatic, 0, 0.5, 0, 0}}}};

  std::cout << robots;

  return 0;
}
