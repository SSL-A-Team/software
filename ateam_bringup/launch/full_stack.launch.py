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

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ateam_bringup_path = os.path.join(get_package_share_directory('ateam_bringup') , 'launch')
    grsim_launch = launch.actions.IncludeLaunchDescription(FrontendLaunchDescriptionSource([ateam_bringup_path, '/ssl_grsim.launch.xml']))
    game_controller_launch = launch.actions.IncludeLaunchDescription(FrontendLaunchDescriptionSource([ateam_bringup_path, '/ssl_game_controller.launch.xml']))
    autonomy_launch = launch.actions.IncludeLaunchDescription(FrontendLaunchDescriptionSource([ateam_bringup_path, '/autonomy.launch.xml']))  

    ui_path = os.path.join(get_package_share_directory('ateam_ui') , 'launch')
    ui_launch = launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource([ui_path, '/ateam_ui_launch.py']))

    return launch.LaunchDescription([
        grsim_launch,
        game_controller_launch,
        autonomy_launch,
        ui_launch,
    ])
