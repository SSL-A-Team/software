cmake_minimum_required(VERSION 3.5)
project(ateam_ui)

find_package(ament_cmake REQUIRED)
find_package(rosbridge_server REQUIRED)

add_custom_target(build_js ALL
  COMMAND bash ${CMAKE_SOURCE_DIR}/build.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

install(DIRECTORY
  launch
  src
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
