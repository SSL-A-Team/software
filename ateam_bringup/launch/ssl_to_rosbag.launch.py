# Copyright 2021 A Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from pathlib import Path
import launch
import os
from launch.substitutions import LaunchConfiguration
from launch.substitutions import AnonName
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ssl_log_dir_value = LaunchConfiguration("ssl_log_dir")
    # ssl_log_dir_arg = DeclareLaunchArgument(
    #     "ssl_log_dir", default_value=""
    # )
    # val = str(ssl_log_dir_value.parse())
    # print(val)

    ssl_log_dir = Path(input("input ssl log directory: "))

    ateam_bringup_path = os.path.join(
        get_package_share_directory("ateam_bringup"), "launch"
    )

    image = "ateam_workspace:latest"

    launch_descriptions = []
    for path in ssl_log_dir.glob("*.log*"):
        # container per file
        file = path.name
        docker_command = 'ros2 launch ateam_bringup ssl_to_rosbag.launch.xml logfile:=/logs/' + file
        print(docker_command)
        launch_descriptions.append(
            ExecuteProcess(cmd=['docker', 'run', '-v', f"{ssl_log_dir}:/logs", image, docker_command], output="both", shell=True)
        )
        print(" ".join(['docker', 'run', '-v', f"{ssl_log_dir}:/logs", image, docker_command]))

    ui_path = os.path.join(get_package_share_directory("ateam_ui"), "launch")
    ui_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ui_path, "/ateam_ui_debug_launch.py"])
    )
    launch_descriptions.append(ui_launch)

    return launch.LaunchDescription(launch_descriptions)
