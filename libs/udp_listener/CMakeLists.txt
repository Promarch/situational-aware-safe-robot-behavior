add_library(udp_listener src/udp_listener.cpp)

target_include_directories(udp_listener PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(udp_listener
    PUBLIC
      Eigen3::Eigen
)
    #   Franka::Franka
    #   control_utilities
#     PRIVATE
#       yaml-cpp
#       project_warnings)