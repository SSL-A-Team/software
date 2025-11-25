ros2 launch ateam_bringup joystick_only_stack.launch.py
colcon build --packages-select ateam_bringup; source ./install/setup.bash; ros2 run ateam_bringup trajectory.py
ros2 topic echo --qos-durability volatile /robot_motion_commands/robot0
