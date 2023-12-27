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

include_directories(include)

# KINEMATICS LIBRARY ###########################################################

add_library(kinematics src/kinematics.cpp
                       include/serial-kinematics/kinematics.hpp)
target_link_libraries(kinematics Eigen3::Eigen)

# EXAMPLES ####################################################################

add_executable(ex_kinematics src/example_kinematics.cpp)
target_link_libraries(ex_kinematics kinematics)
