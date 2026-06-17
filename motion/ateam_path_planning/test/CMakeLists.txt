find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(${PROJECT_NAME}_tests
  planner_tests.cpp
)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
