cmake_minimum_required(VERSION 3.5)
project(ateam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BallState.msg
  msg/BehaviorExecutorState.msg
  msg/Mesh1d.msg
  msg/Overlay.msg
  msg/OverlayArray.msg
  msg/RefereeInfo.msg
  msg/RobotFeedback.msg
  msg/RobotMotionCommand.msg
  msg/RobotMotionFeedback.msg
  msg/RobotMotorFeedback.msg
  msg/RobotState.msg
  msg/Sample3d.msg
  msg/TeamClientConnectionStatus.msg
  msg/Trajectory.msg
  msg/VisionCameraState.msg
  msg/VisionIMMState.msg
  msg/VisionMHTState.msg
  msg/VisionModelState.msg
  msg/VisionWorldState.msg
  msg/World.msg

  msg/FieldInfo.msg
  msg/FieldSidedInfo.msg

  msg/PlayInfo.msg

  srv/ReconnectTeamClient.srv
  srv/SetDesiredKeeper.srv
  srv/SetTeamAdvantageChoice.srv
  srv/SubstituteBot.srv

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  ssl_league_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
