find_package(launch_testing_ament_cmake REQUIRED)

# TODO(barulicm): Figure out why these tests got so flaky and fix them.

# add_launch_test(joystick_control_node_test.py
#   APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
#   TIMEOUT 10
# )
