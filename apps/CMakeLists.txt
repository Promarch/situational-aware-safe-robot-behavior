set(COMMON_LIBS
  Eigen3::Eigen
)

set(EXECUTABLES 
  udp_listener_test
  hand_udp_receiver_example
)

foreach(exe ${EXECUTABLES})
  add_executable(${exe} ${exe}.cpp)
#   target_link_libraries(${exe} PRIVATE ${COMMON_LIBS})
endforeach()

target_link_libraries(udp_listener_test PRIVATE udp_listener)
target_link_libraries(hand_udp_receiver_example PRIVATE udp_listener)