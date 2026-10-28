cmake_minimum_required(VERSION 3.0.2)
project(fr3_control LANGUAGES CXX)

# --- Language settings ---------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json for clangd / VS Code IntelliSense.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- External dependencies -----------------------------------------------
# find_package(Franka 0.9.1 REQUIRED) # Franka 0.9.1 is the compatible version to the version of our robot system, namely 4.2.2)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# --- Subdirectories ------------------------------------------------------
add_subdirectory(libs/udp_listener)

