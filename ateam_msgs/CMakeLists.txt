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
  msg/RobotMotionCommand.msg
  msg/RobotState.msg

  srv/SetDesiredKeeper.srv
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
