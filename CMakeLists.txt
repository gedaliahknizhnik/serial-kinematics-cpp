cmake_minimum_required(VERSION 3.22)

project(
  SerialKinematicsCPP
  VERSION 1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# FLAGS #######################################################################

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# INCLUDES ####################################################################

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Catch2 REQUIRED)

include_directories(include)

# KINEMATICS LIBRARY ###########################################################

add_library(kinematics src/kinematics.cpp
                       include/serial-kinematics/kinematics.hpp)
target_link_libraries(kinematics Eigen3::Eigen)

add_library(kinematic-chain src/kinematic-chain.cpp
                            include/serial-kinematics/kinematic-chain.hpp)
target_link_libraries(kinematic-chain kinematics boost::boost)

# EXAMPLES #####################################################################

add_executable(ex_kinematics src/example-kinematics.cpp)
target_link_libraries(ex_kinematics kinematics kinematic-chain)

# TESTING ######################################################################

add_executable(tests tests/kinematics.cpp tests/kinematic-chain.cpp)
target_link_libraries(tests kinematics kinematic-chain Catch2::Catch2)

include(Catch)
catch_discover_tests(tests)
