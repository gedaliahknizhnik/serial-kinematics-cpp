#define CATCH_CONFIG_MAIN

#include "serial-kinematics/kinematics.hpp"

#include "catch2/catch_all.hpp"

TEST_CASE("Test homogeneous transforms", "[Kinematics]") {
  SECTION("Revolute Joint") {
    kinematics::Joint joint{kinematics::Type::revolute, 0, 1, 1, 0};

    Eigen::Matrix4d H1;
    H1 << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1;
    REQUIRE(H1.isApprox(kinematics::get_transform(joint, 0)));

    Eigen::Matrix4d H2;
    H2 << 0.248, -0.969, 0, 0.248, 0.969, 0.248, 0, 0.969, 0, 0, 1, 1, 0, 0, 0,
        1;
    REQUIRE(H2.isApprox(kinematics::get_transform(joint, 1.32), 1e-3));
  }

  SECTION("Prismatic joint") {
    kinematics::Joint joint{kinematics::Type::prismatic, M_PI / 2, 0.6, 0.8,
                            M_PI / 6};

    Eigen::Matrix4d H1;
    H1 << 0, -0.866, 0.5, 0, 1, 0, 0, 0.6, 0, 0.5, 0.866, 0.8, 0, 0, 0, 1;
    REQUIRE(H1.isApprox(kinematics::get_transform(joint, 0), 1e-3));

    Eigen::Matrix4d H2;
    H2 << 0, -0.866, 0.5, 0, 1, 0, 0, 0.6, 0, 0.5, 0.866, 1.8, 0, 0, 0, 1;
    REQUIRE(H2.isApprox(kinematics::get_transform(joint, 1), 1e-3));
  }
}
