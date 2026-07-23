# ateam_controls_analysis

PlotJuggler-based controls analysis for a single robot, driven entirely by
`ateam_radio_msgs/ExtendedTelemetry`.

A small ROS 2 node (`controls_analysis_node`) subscribes to one robot's
`/robot_feedback/extended/robot{id}` and republishes a flat, PlotJuggler-friendly
`ateam_controls_analysis_msgs/ControlsAnalysis` message on
`/controls_analysis/robot{id}`. The node performs the computations PlotJuggler
can't do natively:

- routes the software command (`cmd_echo`) onto the correct time-derivative
  based on the active body control mode,
- rotates local-frame commands (`BCM_LOCAL_*`) into the global frame using the
  robot's theta estimate,
- inserts gaps (NaN) for `BCM_OFF`/`BCM_ESTOP_BRAKE` and the non-x/y/theta modes
  (pivot / line), and breaks curves at mode transitions,
- splits the reference trajectory and command into **per-mode series** so each
  body control mode is drawn in its own color with natural gaps.

## Packages

- `ateam_controls_analysis_msgs` — the `ControlsAnalysis` message.
- `ateam_controls_analysis` — the republisher node, PlotJuggler layouts, launch
  file, and the layout generator.

## Views

Three views (X, Y, Theta). Each view is three vertically stacked plots sharing a
time axis:

| Plot | Derivative | Curves |
|------|-----------|--------|
| Position | 1st | state estimate, reference trajectory (per-mode colored), vision measurement, software cmd (`BCM_GLOBAL_POSITION`) |
| Velocity | 2nd | state estimate, reference trajectory (per-mode colored), software cmd (`BCM_GLOBAL_VELOCITY`/`BCM_LOCAL_VELOCITY`), gyro (theta only) |
| Acceleration | 3rd | firmware output accel + friction-compensated output accel, software cmd (`BCM_GLOBAL_ACCEL`/`BCM_LOCAL_ACCEL`), IMU accel (x/y only) |

## Body control mode → derivative mapping (software command)

| Mode | Value | Command routed to | Frame |
|------|-------|-------------------|-------|
| `BCM_OFF` | 0 | — (gap) | — |
| `BCM_ESTOP_BRAKE` | 1 | — (gap) | — |
| `BCM_GLOBAL_POSITION` | 10 | position | global |
| `BCM_GLOBAL_VELOCITY` | 11 | velocity | global |
| `BCM_LOCAL_VELOCITY` | 12 | velocity | local → rotated to global |
| `BCM_GLOBAL_ACCEL` | 13 | acceleration | global |
| `BCM_LOCAL_ACCEL` | 14 | acceleration | local → rotated to global |
| `BCM_HEADING_PIVOT` | 20 | — (gap) | — |
| `BCM_POINT_PIVOT` | 21 | — (gap) | — |
| `BCM_HEADING_LINE` | 30 | — (gap) | — |
| `BCM_POINT_LINE` | 31 | — (gap) | — |

Measurements are always placed on their physically correct derivative:
vision → position, gyro → theta velocity, IMU → x/y acceleration. The state
estimate is always available on position and velocity; the firmware output
acceleration (raw and friction-compensated) is always available on acceleration.

## Build

```bash
cd <ateam_ws>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ateam_controls_analysis_msgs ateam_controls_analysis
source install/setup.bash
```

## Usage

### Live streaming

```bash
ros2 launch ateam_controls_analysis controls_analysis.launch.py robot_id:=2
```

Then open PlotJuggler (`ros2 run plotjuggler plotjuggler`), start the
**ROS2 Topic Subscriber** streaming plugin, subscribe to
`/controls_analysis/robot2`, and load a layout:
`Layout → File → share/ateam_controls_analysis/plotjuggler/controls_analysis_robot2.xml`.

### From a ROS bag

Play the bag through the node (native PlotJuggler bag loading would not run the
node, so replay + stream instead):

```bash
ros2 launch ateam_controls_analysis controls_analysis.launch.py \
    robot_id:=0 bag:=/path/to/bag
```

or manually:

```bash
ros2 run ateam_controls_analysis controls_analysis_node --ros-args -p robot_id:=0 &
ros2 bag play /path/to/bag --topics /robot_feedback/extended/robot0
```

> Tip: a robot may only connect partway through a game. Use
> `ros2 bag play ... --start-offset <seconds>` to jump to where that robot's
> extended telemetry begins, and `--rate <n>` to fast-forward.

## Node parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_id` | `0` | Robot whose telemetry to analyze. |
| `input_topic` | `""` | Override input topic (default `/robot_feedback/extended/robot{robot_id}`). |
| `output_topic` | `""` | Override output topic (default `/controls_analysis/robot{robot_id}`). |
| `reliability` | `reliable` | Telemetry subscription QoS: `reliable` or `best_effort`. |
| `use_robot_time` | `true` | Stamp the output header with the reconstructed robot timeline (see below). When `false`, uses the ROS receive time. |
| `reboot_reset_threshold_us` | `1000000` | Backward jump in the robot µs counter treated as a reboot. |

## Time axis (reconstructed robot time)

Each `ExtendedTelemetry` carries the robot's own 64-bit microsecond uptime
counter (`timestamp_us_hi`/`lo`). With `use_robot_time:=true` (default) the node
builds the plot time axis from it:

- The first message is grounded at its ROS receive time.
- Subsequent messages advance by the robot's own elapsed time, so the timeline
  reflects **true robot time** — replaying a bag at `--rate 25` still shows the
  real per-sample timing, not the compressed playback timing.
- On a **reboot** the counter jumps backward (uptime restarts near zero). The
  node detects this (drop > `reboot_reset_threshold_us`), re-grounds the axis at
  that message's ROS time, and continues forward from there.

The reconstructed time is written to `header.stamp`, so in PlotJuggler enable
**"Use timestamp from field/header"** on the ROS2 streaming plugin to plot
against it.

### Reboot indicators

- `reboot_event` (float32) pulses to `1.0` on the first sample after a detected
  reboot (else `0.0`).
- `reboot_count` (uint32) is a monotonically increasing staircase of reboots.

The bundled layouts add `reboot_event` (deep-pink) to every plot, so each reboot
shows as a spike at the same instant across all stacked views on the shared time
axis. (PlotJuggler has no true data-driven full-height vertical line; the
`reboot_event` pulse and the `reboot_count` staircase are the practical markers.)

## PlotJuggler layouts

Prebuilt layouts for robots 0 and 2 live in `plotjuggler/`. Generate one for any
robot id:

```bash
ros2 run ateam_controls_analysis generate_layout.py --robot 3 \
    --output controls_analysis_robot3.xml
```

Each body control mode has a consistent color across every plot/tab, so mode
transitions (e.g. `BCM_POINT_PIVOT` → `BCM_HEADING_LINE`) are visible as a color
change with a gap on the reference-trajectory curves.

## Relationship to the legacy workflow

This supersedes the bag → NPZ → matplotlib workflow under
`motion/ateam_controls/controls/analysis` (`telem_visualize.py`,
`telem_bag2np.py`, `record_and_visualize.sh`, and the visualization step of
`controller_tune.py`), which are now marked deprecated. The signal-generation
(`signal_input.py`), parameter-upload, and `accel_tuning` tooling there remain in
use — record a bag from those trials and replay it through this node.
