find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(ateam_cuda_tests
)

target_include_directories(ateam_cuda_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

target_link_libraries(ateam_kenobi_tests
  kenobi_node_component
  ateam_geometry::ateam_geometry_testing
)

add_executable(random_path_planner_test random_path_planner_test.cpp)

target_include_directories(random_path_planner_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

target_link_libraries(random_path_planner_test
  kenobi_node_component
)
