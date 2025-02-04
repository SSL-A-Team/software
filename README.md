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

## Running In Simulation

Our software can be run against [the ER-Force Framework simulator](https://github.com/robotics-erlangen/framework). These instructions should work to get everything set up and running in simulation.

### Install ER-Force Simulator

1. Clone the framework repository

   ```bash
   git clone https://github.com/robotics-erlangen/framework.git
   ```

1. Install dependencies

   ```bash
   sudo apt install cmake protobuf-compiler libprotobuf-dev qtbase5-dev libqt5opengl5-dev g++ libusb-1.0-0-dev libsdl2-dev libqt5svg5-dev libssl-dev
   ```

1. Build simulator

   ```bash
   cd framework
   mkdir build
   cd build
   cmake ..
   cmake --build . --target project_bullet simulator-cli
   ```

1. Setup environment
   
   For our launch utilities to know where to find the simulator-cli executable, you'll need to configure your environment in one of two ways.

   - _(Recommended)_ Set `SIMULATORCLI_PATH` to the full path to the simluator-cli executable.

      ```bash
      # Example assuming you cloned into your home directory
      echo $'export SIMULATORCLI_PATH=~/framework/build/bin/simulator-cli\n' >> ~/.bashrc
      source ~/.bashrc
      ```

   - Add framework/build/bin/ to your `PATH` environment variable such that running `simulator-cli` from any directory correctly starts the simulator.

      ```bash
      # Example assuming you cloned into your home directory
      echo $'export PATH=$PATH:~/framework/build/bin/simulator-cli\n' >> ~/.bashrc
      source ~/.bashrc
      ```

### Starting Simulation Stack

We have a convenient launch file for starting up the complete stack with the simulator.

```bash
ros2 launch ateam_bringup bringup_simulation.launch.py
```

This will start the simlator, game controller, and our full autonomous gameplay stack at one. Launch arguments are available to disable starting up certain components.

**Note:** If you notice issues receiving vision messages from the simulator, or if vision messages are coming in at an unexpectedly low rate, you may need to enable multicast on your loopback interface. We have a script for that:

```bash
ros2 run ateam_bringup enable_loopback_multicast.sh
```

#### Running Two Gameplay Stacks for Self-Scrimmaging.

You can run two instances of our gameplay stack against the same simulator to run simulated self-scrimmages. Just remember to set the team names in the game controller to be different teams.

**Note:** Currently, only one instance of our AI can be run at a time, so the second gameplay stack runs headless.

```bash
# In the first terminal
ros2 launch ateam_bringup bringup_simulation.launch.py

# In the second terminal
ros2 launch ateam_bringup bringup_simulation.launch.py start_sim:=false start_gc:=false start_ui:=false team_name:=Unknown
```
