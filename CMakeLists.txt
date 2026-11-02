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

# MotionGenerator and setDefaultBehavior from libfranka's examples.
# Eigen3::Eigen3 comes from libfranka's own FindEigen3 module.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libfranka/cmake")
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(examples_common STATIC third_party/libfranka/examples/examples_common.cpp)
target_include_directories(examples_common PUBLIC third_party/libfranka/examples)
target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

# --- Libraries (src/)
# 7-axis jerk-limited S-curve with online re-planning (pure math).
add_library(s_curve STATIC src/s_curve.cpp)
target_include_directories(s_curve PUBLIC src)
target_link_libraries(s_curve PUBLIC Franka::Franka)

# Robot motions, hand-aware speed supervision and the UDP hand receiver.
add_library(robot_motion STATIC src/motion.cpp src/supervision.cpp src/hand_receiver.cpp)
target_link_libraries(robot_motion PUBLIC s_curve examples_common Threads::Threads)

# --- Programs (apps/)
add_executable(pick_place apps/pick_place.cpp)
target_link_libraries(pick_place robot_motion)

add_executable(calibrate_camera apps/calibrate_camera.cpp)
target_link_libraries(calibrate_camera robot_motion)

add_executable(test_hand_receiver apps/test_hand_receiver.cpp)
target_link_libraries(test_hand_receiver robot_motion)

add_executable(record_pose apps/record_pose.cpp)
target_link_libraries(record_pose robot_motion)

add_executable(read_state apps/read_state.cpp)
target_link_libraries(read_state Franka::Franka)

# --- Offline tests (tests/), no robot needed
add_executable(test_s_curve tests/test_s_curve.cpp)
target_link_libraries(test_s_curve s_curve)

add_executable(test_replan tests/test_replan.cpp)
target_link_libraries(test_replan s_curve)
