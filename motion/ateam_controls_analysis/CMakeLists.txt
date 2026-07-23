cmake_minimum_required(VERSION 3.8)
project(ateam_controls_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Pure-Python analysis package (importable library + routing logic).
ament_python_install_package(${PROJECT_NAME})

# Executable node / helper scripts.
install(PROGRAMS
  scripts/controls_analysis_node
  scripts/generate_layout.py
  DESTINATION lib/${PROJECT_NAME}
)

# PlotJuggler layouts and launch files.
install(DIRECTORY plotjuggler launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(routing_test test/test_routing.py)
endif()

ament_package()
