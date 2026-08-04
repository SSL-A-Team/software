cmake_minimum_required(VERSION 3.8)
project(ateam_controls_analysis)

find_package(ament_cmake REQUIRED)

# This package ships only Foxglove assets: the controls-analysis layouts and the
# Foxglove extension source (which performs the ExtendedTelemetry analysis inside
# Foxglove -- there is no ROS node or derived message type).
install(DIRECTORY foxglove
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
