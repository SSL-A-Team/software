find_package(ament_cmake_gmock REQUIRED)
ament_add_gmock(unit_tests
  rnp_packet_helpers_tests.cpp
)
target_link_libraries(unit_tests ${PROJECT_NAME})
