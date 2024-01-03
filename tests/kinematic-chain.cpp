#include "serial-kinematics/kinematic-chain.hpp"

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

  // SECTION("Revolute Joint") {
  //   kinematics::Joint joint{kinematics::Type::revolute, 0, 1, 1, 0};
  //
  //   Eigen::Matrix4d H1;
  //   H1 << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1;
  //   REQUIRE(H1.isApprox(kinematics::get_transform(joint, 0)));
  //
  //   Eigen::Matrix4d H2;
  //   H2 << 0.248, -0.969, 0, 0.248, 0.969, 0.248, 0, 0.969, 0, 0, 1, 1, 0, 0,
  //   0,
  //       1;
  //   REQUIRE(H2.isApprox(kinematics::get_transform(joint, 1.32), 1e-3));
  // }
}
