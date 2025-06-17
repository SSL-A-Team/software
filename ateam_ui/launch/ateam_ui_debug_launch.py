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
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ateam_ui_src = os.path.join(get_package_share_directory("ateam_ui"), "src")
    ui_process = launch.actions.ExecuteProcess(
        cmd=["yarn", "dev"],
        cwd=ateam_ui_src,
        output="screen"
    )

    rosbridge_node = launch_ros.actions.Node(
        package="rosbridge_server",
        name="rosbridge",
        executable="rosbridge_websocket.py"
    )

    rosapi_node = launch_ros.actions.Node(
        package="rosapi",
        name="rosapi",
        executable="rosapi_node"
    )

    shutdown_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=ui_process,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown(reason="Window Closed")
            )]
        )
    )

    return launch.LaunchDescription([
        ui_process,
        rosbridge_node,
        rosapi_node,
        shutdown_handler
    ])
