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

include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.4.0 # or a later release
)

FetchContent_MakeAvailable(Catch2)

include_directories(include)

# KINEMATICS LIBRARY ###########################################################

add_library(kinematics src/kinematics.cpp
                       include/serial-kinematics/kinematics.hpp)
target_link_libraries(kinematics Eigen3::Eigen)

# EXAMPLES #####################################################################

add_executable(ex_kinematics src/example_kinematics.cpp)
target_link_libraries(ex_kinematics kinematics)

# TESTING ######################################################################

add_executable(tests tests/kinematics.cpp)
target_link_libraries(tests PRIVATE kinematics Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(tests)
