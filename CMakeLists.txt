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

# Robot motions (joint S-curve, vertical and Cartesian pose moves) and hand-aware supervision.
add_library(robot_motion STATIC src/motion.cpp src/segments.cpp src/supervision.cpp
            src/virtual_hand.cpp)
target_include_directories(robot_motion PUBLIC src)
target_link_libraries(robot_motion PUBLIC s_curve examples_common)

add_executable(record_pre_pick src/record_pre_pick.cpp)
target_link_libraries(record_pre_pick robot_motion)

add_executable(robot_test_pick tests/robot_test_pick.cpp)
target_link_libraries(robot_test_pick robot_motion)

add_executable(gripper_test src/gripper_test.cpp)
target_link_libraries(gripper_test Franka::Franka)

add_executable(test_replan tests/test_replan.cpp)
target_link_libraries(test_replan s_curve)

add_executable(sim_hand_slowdown tests/sim_hand_slowdown.cpp)
find_package(Threads REQUIRED)
target_link_libraries(sim_hand_slowdown robot_motion Threads::Threads)

add_executable(robot_test_slowdown tests/robot_test_slowdown.cpp)
target_link_libraries(robot_test_slowdown robot_motion Threads::Threads)
