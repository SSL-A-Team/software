cmake_minimum_required(VERSION 3.5)
project(ateam_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_index_has_resource(ATEAM_YASS_PLUGINS_PREFIX packages ateam_yass_plugins)
if(NOT ATEAM_YASS_PLUGINS_PREFIX)
  message(FATAL_ERROR "Could not find ateam_yass_plugins in the ament index (AMENT_PREFIX_PATH='$ENV{AMENT_PREFIX_PATH}')")
endif()

configure_file(config/yass_config.yaml.in config/yass_config.yaml)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  images
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_BINARY_DIR}/config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
