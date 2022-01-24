cmake_minimum_required(VERSION 3.5)
project(ateam_ui)

find_package(ament_cmake REQUIRED)
find_package(rosbridge_server REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# Copy src to install directory
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/src ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
  OUTPUT_VARIABLE RESULT0
  ERROR_VARIABLE ERROR_RESULT
)

add_custom_target(build_js ALL
  # Install NodeJS if necessary
  COMMAND bash -c "(npm list --depth 1 --global n > /dev/null 2>&1) || (sudo npm install -g n && sudo n stable)"

  # Install yarn if necessary
  COMMAND bash -c "(npm list --depth 1 --global yarn > /dev/null 2>&1) || sudo npm install -g yarn"

  # Install neutralino CLI if necessary
  COMMAND bash -c "(npm list --depth 1 --global @neutralinojs/neu > /dev/null 2>&1) || sudo npm install -g @neutralinojs/neu"

  # Install Yarn dependencies
  COMMAND bash -c "yarn install"

  # Configure neu if necessary
  COMMAND bash -c "(test -d ./bin) || neu update"

  # Build the UI and fix permission issue if necessary
  COMMAND bash -c "(yarn vite build && neu build --release) && (test -x ./bin/neutralino-linux_x64 || (chmod +x ./dist/Ateam_UI/*))"

  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
  VERBATIM
)

ament_package()
