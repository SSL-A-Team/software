find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(unit_tests
  conversion_tests.cpp
  rnp_packet_helpers_tests.cpp
)
target_link_libraries(unit_tests ${PROJECT_NAME})
