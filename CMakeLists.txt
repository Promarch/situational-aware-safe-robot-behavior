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
