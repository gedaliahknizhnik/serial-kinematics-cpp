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
}
