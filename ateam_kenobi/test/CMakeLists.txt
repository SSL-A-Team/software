find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(ateam_kenobi_tests
  path_planner_test.cpp
)

target_include_directories(ateam_kenobi_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src>
)

target_link_libraries(ateam_kenobi_tests kenobi_node_component)
