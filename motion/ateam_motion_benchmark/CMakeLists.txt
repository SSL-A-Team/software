cmake_minimum_required(VERSION 3.8)
project(ateam_motion_benchmark)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ateam_msgs REQUIRED)
find_package(ateam_common REQUIRED)

add_executable(velocity_benchmark src/velocity_benchmark.cpp)
target_compile_features(velocity_benchmark PUBLIC cxx_std_20)
ament_target_dependencies(velocity_benchmark
  rclcpp
  ateam_common
  ateam_msgs
)

install(TARGETS velocity_benchmark DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
