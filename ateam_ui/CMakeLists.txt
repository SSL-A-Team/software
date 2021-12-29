cmake_minimum_required(VERSION 3.5)
project(ateam_ui)

find_package(ament_cmake REQUIRED)
find_package(rosbridge_server REQUIRED)


install(DIRECTORY
  launch
  src
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install NodeJS if necessary
execute_process(
  COMMAND bash -c "(npm list --depth 1 --global n > /dev/null 2>&1) || (sudo npm install -g n && sudo n stable)"
)

# Install yarn if necessary
execute_process(
  COMMAND bash -c "(npm list --depth 1 --global yarn > /dev/null 2>&1) || sudo npm install -g yarn"
)

# Install neutralino CLI if necessary
execute_process(
  COMMAND bash -c "(npm list --depth 1 --global @neutralinojs/neu > /dev/null 2>&1) || sudo npm install -g @neutralinojs/neu"
)

# Install Yarn dependencies
execute_process(
  COMMAND bash -c "yarn install"
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
)

# Configure neu if necessary
execute_process(
  COMMAND bash -c "(test -d ./bin) || neu update"
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
)


# # This step is currently incomplete, "neu build --release" not working
# Build the UI and fix permission issue if necessary
execute_process(
  COMMAND bash -c "yarn vite build"
  WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/src
)

ament_package()
