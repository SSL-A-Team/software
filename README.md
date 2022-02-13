# software

## Gettings Started

1. Install ROS 2

   Follow the [instructions for ROS 2 Galactic](http://docs.ros.org/en/galactic/Installation.html) for your system.

1. Make a ROS workspace and clone our repo

   ```bash
   mkdir ateam_ws
   cd ateam_ws
   mkdir src
   cd src
   git clone git@github.com:SSL-A-Team/software.git
   ```

1. Install our ROS dependencies

   ```bash
   # In the ateam_ws directory
   rosdep install --from-paths . --ignore-src -y
   ```

1. Install our non-ROS dependencies

   ```bash
   # In the ateam_ws directory
   source /opt/ros/galactic/setup.bash
   ./ateam_ui/install_deps.sh
   ```

1. Build the code

   ```bash
   # In the ateam_ws directory
   source /opt/ros/galactic/setup.bash
   colcon build
   ```

**Note:** You'll need to source both the underlay (`/opt/ros/galactic/setup.bash`) and our workspace's overlay (`ateam_ws/install/setup.bash`) in every terminal session before running any of our code.
