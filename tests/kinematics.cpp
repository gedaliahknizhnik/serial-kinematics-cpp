#define CATCH_CONFIG_MAIN

#include "serial-kinematics/kinematics.hpp"

// #include "catch.hpp"
// #include "catch2/catch.hpp"
#include "catch2/catch_all.hpp"

// Test case is a single test that you run
// You give it a name/description and also you give it some tags.
TEST_CASE("Testing framework is working fine", "[Catch2]") {
  // Tests have to meet some requirements to be considered valid
  REQUIRE(true);
}

// TEST_CASE("Testing MyLib", "[mylib]") {
//   int width = GetWidth();  // imaginary function in MyLib
//   REQUIRE(width == 1920);
//
//   // Sections would actually run the code from the beginning of the test case
//   // but they you will run sections one by one
//   SECTION("A Section") {
//     SetWidth(640);
//     width = GetWidth();
//     REQUIRE(width == 640);
//   }
// }
