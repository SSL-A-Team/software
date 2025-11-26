ros2 launch ateam_bringup joystick_only_stack.launch.py
colcon build --packages-select ateam_bringup; source ./install/setup.bash; ros2 run ateam_bringup trajectory.py
ros2 topic echo --qos-durability volatile /robot_motion_commands/robot2
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build --packages-select ateam_bangbang && source install/setup.bash && ros2 run ateam_bangbang bangbang_node
ln -s build/compile_commands.json .
ros2 topic echo /yellow_team/robot2 --field pose
ros2 bag record -a -o rosbag_idle
ros2 bag play rosbag_idle
