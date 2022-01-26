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


add_custom_command(
  OUTPUT file_list PRE_BUILD
  COMMAND bash -c "ls -R > ${CMAKE_BINARY_DIR}/file_list"
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src
  VERBATIM
)

add_custom_target(build_js ALL
  COMMAND ${CMAKE_COMMAND}
    -DINSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DBINARY_DIR=${CMAKE_BINARY_DIR}
    -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DPROJECT_NAME=${PROJECT_NAME}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/CopySrc.cmake

  # Install NodeJS if necessary
  COMMAND bash -c "(npm list --quiet --depth 1 --global n > /dev/null 2>&1) || (sudo npm install -g n && sudo n stable)"

  # Install yarn if necessary
  COMMAND bash -c "(npm list --quiet --depth 1 --global yarn > /dev/null 2>&1) || sudo npm install -g yarn"

  # Install neutralino CLI if necessary
  COMMAND bash -c "(npm list --quiet --depth 1 --global @neutralinojs/neu > /dev/null 2>&1) || sudo npm install -g @neutralinojs/neu"

  # Install Yarn dependencies
  COMMAND bash -c "yarn install"

  # Configure neu if necessary
  COMMAND bash -c "(test -d ./bin) || neu update"

  # Build the UI and fix permission issue if necessary
  COMMAND bash -c "(yarn vite build && neu build --release) && (test -x ./bin/neutralino-linux_x64 || (chmod +x ./dist/Ateam_UI/*))"

  DEPENDS file_list
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
  VERBATIM
)


ament_package()
