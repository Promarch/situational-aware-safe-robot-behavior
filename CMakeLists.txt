cmake_minimum_required(VERSION 3.10)
project(franka_pick_place CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# libfranka is vendored as a git submodule pinned to 0.9.2 (must match robot system version).
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/libfranka EXCLUDE_FROM_ALL)

add_executable(read_state src/read_state.cpp)
target_link_libraries(read_state Franka::Franka)

# MotionGenerator and setDefaultBehavior from libfranka's examples.
# Eigen3::Eigen3 comes from libfranka's own FindEigen3 module.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libfranka/cmake")
find_package(Eigen3 REQUIRED)

add_library(examples_common STATIC third_party/libfranka/examples/examples_common.cpp)
target_include_directories(examples_common PUBLIC third_party/libfranka/examples)
target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

# 7-axis jerk-limited S-curve trajectory.
add_library(s_curve STATIC src/s_curve.cpp)
target_include_directories(s_curve PUBLIC src)
target_link_libraries(s_curve PUBLIC Franka::Franka)

add_executable(test_s_curve tests/test_s_curve.cpp)
target_link_libraries(test_s_curve s_curve)

# Blocking robot motions (joint S-curve, vertical and Cartesian pose moves).
add_library(robot_motion STATIC src/motion.cpp)
target_include_directories(robot_motion PUBLIC src)
target_link_libraries(robot_motion PUBLIC s_curve examples_common)

add_executable(record_pre_pick src/record_pre_pick.cpp)
target_link_libraries(record_pre_pick robot_motion)

add_executable(robot_test_pick tests/robot_test_pick.cpp)
target_link_libraries(robot_test_pick robot_motion)
