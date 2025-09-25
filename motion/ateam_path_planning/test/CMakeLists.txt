find_package(ament_cmake_gmock REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_tests
  planner_tests.cpp
)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
