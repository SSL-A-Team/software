cmake_minimum_required(VERSION 3.5)
project(ateam_ui)

find_package(ament_cmake REQUIRED)
find_package(rosbridge_server REQUIRED)


install(PROGRAMS
  scripts/ateam_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  launch/ateam_ui_launch.xml
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
