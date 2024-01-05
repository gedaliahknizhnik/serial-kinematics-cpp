#include "serial-kinematics/kinematic-chain.hpp"

#include <iostream>
#include <stdexcept>

#include "catch2/catch_all.hpp"
#include "serial-kinematics/kinematics.hpp"

TEST_CASE("Test kinematic chain errors", "[Kinematic-Chain]") {
  SECTION("Constructor") {
    kinematics::DenhavitHartenbergParam dh{
        {{kinematics::Type::revolute, 0, 0, 0, 0},
         {kinematics::Type::revolute, 0, 0, 0, 0}}};
    REQUIRE_NOTHROW(kinematics::KinematicChain{dh});
  }

  SECTION("Setting joint variables") {
    kinematics::DenhavitHartenbergParam dh{
        {{kinematics::Type::revolute, 0, 0, 0, 0},
         {kinematics::Type::revolute, 0, 0, 0, 0}}};
    kinematics::KinematicChain robot{dh};
    REQUIRE_NOTHROW(robot.set_joint_vars(Eigen::Vector2d{1, 2}));
    REQUIRE_THROWS_AS(robot.set_joint_vars(Eigen::Vector3d{1, 2, 3}),
                      std::invalid_argument);
  }

  SECTION("Transformations") {
    // kinematics::Joint{
    // kinematics::type::revolute,
    // };
    kinematics::DenhavitHartenbergParam dh{
        {{kinematics::Type::revolute, 0, 0, 290, -M_PI / 2},
         {kinematics::Type::revolute, -M_PI / 2, 270, 0, 0},
         {kinematics::Type::revolute, M_PI, -70, 0, M_PI / 2},
         {kinematics::Type::revolute, 0, 0, 302, -M_PI / 2},
         {kinematics::Type::revolute, 0, 0, 0, M_PI / 2},
         {kinematics::Type::revolute, 0, 0, 72, 0}}};
    kinematics::KinematicChain robot{dh};

    robot.set_joint_vars(Eigen::Vector<double, 6>{0, 0, 0, 0, 0, 0});

    std::array<kinematics::HomMat, 7> T;
    T[0] = kinematics::HomMat::Identity();
    T[1] << 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 290, 0, 0, 0, 1;
    T[2] << 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 560, 0, 0, 0, 1;
    T[3] << 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 630, 0, 0, 0, 1;
    T[4] << 0, -1, 0, 302, 0, 0, 1, 0, -1, 0, 0, 630, 0, 0, 0, 1;
    T[5] << 0, 0, 1, 302, 0, 1, 0, 0, -1, 0, 0, 630, 0, 0, 0, 1;
    T[6] << 0, 0, 1, 374, 0, 1, 0, 0, -1, 0, 0, 630, 0, 0, 0, 1;

    for (size_t ii{0}; ii < T.size(); ++ii) {
      REQUIRE(T[ii].isApprox(robot.get_transform_upto(ii)));
    }

    robot.set_joint_vars(Eigen::Vector<double, 6>{
        M_PI / 3, -M_PI / 3, M_PI / 4, M_PI / 6, -M_PI / 6, M_PI / 9});

    std::array<kinematics::HomMat, 7> T1;
    T1[0] = kinematics::HomMat::Identity();
    T1[1] << 0.5, 0, -0.866, 0, 0.866, 0, 0.5, 0, 0, -1, 0, 290, 0, 0, 0, 1;
    T1[2] << -0.433, 0.25, -0.866, -116.914, -0.75, 0.433, 0.5, -202.5, 0.5,
        0.866, 0, 425, 0, 0, 0, 1;
    T1[3] << 0.129, -0.866, 0.483, -125.972, 0.224, 0.5, 0.837, -218.19, -0.966,
        0, 0.259, 492.615, 0, 0, 0, 1;
    T1[4] << -0.321, -0.483, -0.815, 19.883, 0.444, -0.837, 0.321, 34.438,
        -0.837, -0.259, 0.483, 570.778, 0, 0, 0, 1;
    T1[5] << -0.036, -0.815, 0.579, 19.883, 0.803, 0.321, 0.502, 34.438, -0.595,
        0.483, 0.642, 570.778, 0, 0, 0, 1;
    T1[6] << -0.313, -0.753, 0.579, 61.551, 0.864, 0.027, 0.502, 70.61, -0.394,
        0.657, 0.642, 617.031, 0, 0, 0, 1;

    for (size_t ii{0}; ii < T1.size(); ++ii) {
      REQUIRE(T1[ii].isApprox(robot.get_transform_upto(ii), 1e-3));
    }

    // TODO: Test reverse case and partial
  }
}
