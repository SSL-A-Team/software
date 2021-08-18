# software

## Gettings Started

1. Install ROS 2

   Follow the [instructions for ROS 2 Galactic](http://docs.ros.org/en/galactic/Installation.html) for your system.

1. Make a ROS workspace and clone our repo

   ```bash
   mkdir ateam_ws
   cd ateam_ws
   mkdir src
   git clone git@github.com:SSL-A-Team/software.git
   ```

1. Install our ROS dependencies

   ```bash
   # In the ateam_ws directory
   rosdep install --from-paths . --ignore-src -y
   ```

1. Build the code

   ```bash
   # In the ateam_ws directory
   colcon build
   ```