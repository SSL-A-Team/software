colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-select ateam_bangbang ateam_msgs ateam_bringup ateam_radio_msgs ateam_radio_bridge
ln -s build/compile_commands.json .

ros2 launch ateam_bringup joystick_only_stack.launch.py
ros2 launch ateam_bringup bringup_connection.launch.py command_frequency:=100.0
ros2 run ateam_motion_input motion_input_node --ros-args -p dimension:=theta -p fn_type:=oscillate -p amp:=30 -p freq:=0.5 -p duration:=0.0
python3 ./analysis/scripts/param_tuning_loop.py

ros2 service call /set_firmware_param ateam_msgs/srv/SetFirmwareParameter "{robot_id: 2, parameter_id: 6, data: [0.1]}"

ros2 topic echo --qos-durability volatile /robot_motion_commands/robot2
ros2 topic echo /yellow_team/robot2 --field pose

ros2 bag record -a -o rosbag_idle
ros2 bag play rosbag_idle