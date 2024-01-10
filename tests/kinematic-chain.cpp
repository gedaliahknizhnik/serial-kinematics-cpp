#include "serial-kinematics/kinematic-chain.hpp"

#include <iostream>
#include <stdexcept>

#include "catch2/catch_all.hpp"
#include "catch2/catch_test_macros.hpp"
#include "serial-kinematics/kinematics.hpp"

TEST_CASE("Test kinematic chain errors", "[kinematic-chain]") {
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
}

TEST_CASE("Test Kinematic chain transformatons",
          "[kinematic-chain][transformation]") {
  // Robot parameters based on example from https://prajankya.me/dh/
  kinematics::DenhavitHartenbergParam dh{
      {{kinematics::Type::revolute, 0, 0, 290, -M_PI / 2},
       {kinematics::Type::revolute, -M_PI / 2, 270, 0, 0},
       {kinematics::Type::revolute, M_PI, -70, 0, M_PI / 2},
       {kinematics::Type::revolute, 0, 0, 302, -M_PI / 2},
       {kinematics::Type::revolute, 0, 0, 0, M_PI / 2},
       {kinematics::Type::revolute, 0, 0, 72, 0}}};
  kinematics::KinematicChain robot{dh};

  SECTION("Transformation Limits", "Test whether robot enforces DOF limits") {
    REQUIRE_THROWS_AS(robot.get_transform_upto(dh.size() + 1),
                      std::range_error);
    REQUIRE_THROWS_AS(robot.get_transform_upto(-1), std::range_error);
  }

  SECTION("Transformation Values - Basic",
          "Test whether kinematic chains are correctly transformed") {
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
      REQUIRE(T[ii].inverse().isApprox(robot.get_transform_upto(0, ii), 1e-3));
      REQUIRE(
          T[ii].block<3, 3>(0, 0).isApprox(robot.get_rotation_upto(ii), 1e-3));
      REQUIRE(T[ii].block<4, 1>(0, 3).isApprox(robot.get_position_vec_hom(ii),
                                               1e-3));
      REQUIRE(
          T[ii].block<3, 1>(0, 3).isApprox(robot.get_position_vec(ii), 1e-3));
      REQUIRE(T[ii].block<3, 1>(0, 3).isApprox(
          robot.transform_point(Eigen::Vector3d{0, 0, 0}, ii), 1e-3));
      REQUIRE(T[ii].block<4, 1>(0, 3).isApprox(
          robot.transform_point(Eigen::Vector4d{0, 0, 0, 1}, ii), 1e-3));
    }
  }

  SECTION("Transformation Values - Advanced",
          "Test whether kinematic cains are correctly transformed with "
          "non-zero joint variables") {
    robot.set_joint_vars(Eigen::Vector<double, 6>{
        M_PI / 3, -M_PI / 3, M_PI / 4, M_PI / 6, -M_PI / 6, M_PI / 9});

    std::array<kinematics::HomMat, 7> T;
    T[0] = kinematics::HomMat::Identity();
    T[1] << 0.5, 0, -0.866, 0, 0.866, 0, 0.5, 0, 0, -1, 0, 290, 0, 0, 0, 1;
    T[2] << -0.433, 0.25, -0.866, -116.914, -0.75, 0.433, 0.5, -202.5, 0.5,
        0.866, 0, 425, 0, 0, 0, 1;
    T[3] << 0.129, -0.866, 0.483, -125.972, 0.224, 0.5, 0.837, -218.19, -0.966,
        0, 0.259, 492.615, 0, 0, 0, 1;
    T[4] << -0.321, -0.483, -0.815, 19.883, 0.444, -0.837, 0.321, 34.438,
        -0.837, -0.259, 0.483, 570.778, 0, 0, 0, 1;
    T[5] << -0.036, -0.815, 0.579, 19.883, 0.803, 0.321, 0.502, 34.438, -0.595,
        0.483, 0.642, 570.778, 0, 0, 0, 1;
    T[6] << -0.313, -0.753, 0.579, 61.551, 0.864, 0.027, 0.502, 70.61, -0.394,
        0.657, 0.642, 617.031, 0, 0, 0, 1;

    for (size_t ii{0}; ii < T.size(); ++ii) {
      REQUIRE(T[ii].isApprox(robot.get_transform_upto(ii), 1e-3));
      REQUIRE(T[ii].inverse().isApprox(robot.get_transform_upto(0, ii), 1e-3));
      REQUIRE(
          T[ii].block<3, 3>(0, 0).isApprox(robot.get_rotation_upto(ii), 1e-3));
      REQUIRE(T[ii].block<4, 1>(0, 3).isApprox(robot.get_position_vec_hom(ii),
                                               1e-3));
      REQUIRE(
          T[ii].block<3, 1>(0, 3).isApprox(robot.get_position_vec(ii), 1e-3));
      REQUIRE(T[ii].block<3, 1>(0, 3).isApprox(
          robot.transform_point(Eigen::Vector3d{0, 0, 0}, ii), 1e-3));
      REQUIRE(T[ii].block<4, 1>(0, 3).isApprox(
          robot.transform_point(Eigen::Vector4d{0, 0, 0, 1}, ii), 1e-3));
    }
  }

  SECTION("Transformation Values - Partial",
          "Test whether partial transformations are correctly performed.") {
    robot.set_joint_vars(Eigen::Vector<double, 6>{
        M_PI / 3, -M_PI / 3, M_PI / 4, M_PI / 6, -M_PI / 6, M_PI / 9});

    kinematics::HomMat TPart1, TPart2;
    TPart1 << 0.592, -0.353, 0.724, 91.925, 0.394, -0.657, -0.642, -327.031,
        0.703, 0.666, -0.25, -18, 0, 0, 0, 1;
    TPart2 << 0.677, -0.129, 0.724, 39.765, 0.595, -0.483, -0.642, -280.778,
        0.433, 0.866, -0.25, 0, 0, 0, 0, 1;
    REQUIRE(TPart1.isApprox(robot.get_transform_upto(6, 1), 1e-3));
    REQUIRE(TPart2.isApprox(robot.get_transform_upto(5, 1), 1e-3));
  }
}
