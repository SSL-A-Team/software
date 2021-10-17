cmake_minimum_required(VERSION 3.5)
project(ssl_league_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/GameEvent.msg
  msg/GameEventProposalGroup.msg
  msg/Referee.msg
  msg/TeamInfo.msg

  msg/VisionDetectionBall.msg
  msg/VisionDetectionRobot.msg
  msg/VisionDetectionFrame.msg

  msg/VisionFieldCircularArc.msg
  msg/VisionFieldLineSegment.msg
  msg/VisionGeometryFieldSize.msg
  msg/VisionGeometryCameraCalibration.msg
  msg/VisionGeometryData.msg

  msg/VisionWrapper.msg

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()