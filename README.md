# software

## Gettings Started

**Note:** The following instructions are written assuming you are using Bash as your shell. The file extension of the setup scripts may be different if you are using a different shell. (`.bash`, `.sh`, and `.zsh` are available)

1. Install ROS 2

   Follow the [instructions for ROS 2 Jazzy](http://docs.ros.org/en/jazzy/Installation.html) for your system.

1. Initialize rosdep dependency manager

   ```bash
   sudo rosdep init
   rosdep update
   ```

1. Make a ROS workspace and clone our repo

   ```bash
   mkdir ateam_ws
   cd ateam_ws
   mkdir src
   cd src
   git clone --recursive git@github.com:SSL-A-Team/software.git
   ```

1. Install our ROS dependencies

   ```bash
   # Source underlay if you haven't already in this terminal
   source /opt/ros/jazzy/setup.bash
   # In the ateam_ws directory
   rosdep install --from-paths . --ignore-src -y
   ```

1. Install our non-ROS dependencies

   ```bash
   # Source underlay if you haven't already in this terminal
   source /opt/ros/jazzy/setup.bash
   # In the ateam_ws directory
   ./src/software/ateam_ui/install_deps.sh
   ```

1. Build the code

   ```bash
   # Source underlay if you haven't already in this terminal
   source /opt/ros/jazzy/setup.bash
   # In the ateam_ws directory
   colcon build
   ```

1. [Optionally] Run the tests

   ```bash
   # Source underlay if you haven't already in this terminal
   source /opt/ros/jazzy/setup.bash
   # In the ateam_ws directory
   colcon test && colcon test-result --verbose
   ```

**Note:** You'll need to source both the underlay (`/opt/ros/jazzy/setup.bash`) and our workspace's overlay (`ateam_ws/install/setup.bash`) in every terminal session before running any of our code.
