find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(ateam_kenobi_tests
  greedy_assignment_test.cpp
)

target_include_directories(ateam_kenobi_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

target_link_libraries(ateam_kenobi_tests
  kenobi_node_component
  ateam_geometry::ateam_geometry_testing
)
