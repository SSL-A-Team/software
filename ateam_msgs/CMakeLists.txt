cmake_minimum_required(VERSION 3.5)
project(ateam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ssl_league_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/JoystickControlStatus.msg
  msg/KenobiStatus.msg
  msg/Overlay.msg
  msg/OverlayArray.msg
  msg/RefereeInfo.msg
  msg/RobotMotionCommand.msg
  msg/TeamClientConnectionStatus.msg

  msg/GameStateBall.msg
  msg/GameStateRobot.msg
  msg/GameStateWorld.msg

  msg/VisionStateBall.msg
  msg/VisionStateCamera.msg
  msg/VisionStateCameraArray.msg
  msg/VisionStateIMM.msg
  msg/VisionStateMHT.msg
  msg/VisionStateModel.msg
  msg/VisionStateRobot.msg

  msg/FieldInfo.msg
  msg/FieldSidedInfo.msg

  msg/PlaybookState.msg
  msg/PlayInfo.msg

  srv/GetFirmwareParameter.srv
  srv/ReconnectTeamClient.srv
  srv/SetDesiredKeeper.srv
  srv/SetFirmwareParameter.srv
  srv/SetOverridePlay.srv
  srv/SetPlayEnabled.srv
  srv/SetTeamAdvantageChoice.srv
  srv/SubstituteBot.srv
  srv/SetIgnoreFieldSide.srv
  srv/SendSimulatorControlPacket.srv

  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  sensor_msgs
  ssl_league_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
