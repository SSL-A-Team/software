ros2 launch ateam_bringup joystick_only_stack.launch.py
colcon build --packages-select ateam_bangbang ateam_msgs ateam_bringup ateam_radio_msgs ateam_radio_bridge && source ./install/setup.bash && ros2 launch ateam_bringup bringup_bangbang.launch.py command_frequency:=100.0
colcon build --packages-select ateam_bringup; source ./install/setup.bash; ros2 run ateam_bringup trajectory.py
ros2 topic echo --qos-durability volatile /robot_motion_commands/robot2
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build --packages-select ateam_bangbang && source install/setup.bash && ros2 run ateam_bangbang bangbang_node
ln -s build/compile_commands.json .
ros2 topic echo /yellow_team/robot2 --field pose
ros2 bag record -a -o rosbag_idle
ros2 bag play rosbag_idle
colcon build --packages-select ateam_bangbang && ros2 run ateam_bangbang bangbang_node --ros-args -p a_linear:=0.2 -p a_angular:=0.0
