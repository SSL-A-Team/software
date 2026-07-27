# ateam_controls_analysis

PlotJuggler-based controls analysis for a single robot, driven entirely by
`ateam_radio_msgs/ExtendedTelemetry`.

A small ROS 2 node (`controls_analysis_node`) subscribes to one robot's
`/robot_feedback/extended/robot{id}` and republishes a flat, PlotJuggler-friendly
`ateam_controls_analysis_msgs/ControlsAnalysis` message on `/controls_analysis`
(one robot at a time, selected by the `robot_id` param). The node performs the
computations PlotJuggler can't do natively:

- routes the software command (`cmd_echo`) onto the correct time-derivative
  based on the active body control mode,
- rotates local-frame commands (`BCM_LOCAL_*`) into the global frame using the
  robot's theta estimate,
- inserts gaps (NaN) for `BCM_OFF`/`BCM_ESTOP_BRAKE` and the non-x/y/theta modes
  (pivot / line), and breaks curves at mode transitions,
- splits the reference trajectory and command into **per-mode series** so each
  body control mode is drawn in its own color with natural gaps.

The same computations are available offline as a **bag → bag converter**
(`controls_analysis_bag_convert`): it reads a recorded `ExtendedTelemetry` bag
and writes a new bag containing `/controls_analysis`, which PlotJuggler can open
**directly** (native full-bag load) — no node, no replay-streaming. See
[From a ROS bag](#from-a-ros-bag).

## Packages

- `ateam_controls_analysis_msgs` — the `ControlsAnalysis` message.
- `ateam_controls_analysis` — the republisher node, the offline bag converter,
  the PlotJuggler layout, and the launch file.

## Views

Three body-frame views (X, Y, Theta). Each view is three vertically stacked
plots sharing a time axis:

| Plot | Derivative | Curves |
|------|-----------|--------|
| Position | 1st | state estimate, reference trajectory (per-mode colored), vision measurement, software cmd (`BCM_GLOBAL_POSITION`) |
| Velocity | 2nd | state estimate, reference trajectory (per-mode colored), software cmd (`BCM_GLOBAL_VELOCITY`/`BCM_LOCAL_VELOCITY`), gyro (theta only) |
| Acceleration | 3rd | firmware output accel + friction-compensated output accel, software cmd (`BCM_GLOBAL_ACCEL`/`BCM_LOCAL_ACCEL`), IMU accel (x/y only) |

Three additional wheel/boot views, whose time axis is linked to the body-frame
views (all plots are `TimeSeries`, so panning/zooming the time axis is shared):

| View | Layout | Curves |
|------|--------|--------|
| Velocity | 4 stacked plots (front-left, back-left, back-right, front-right) | measured wheel velocity (`vel`, rad/s) vs. setpoint (`vel_setpoint`) per wheel |
| Current | 4 stacked plots (same wheel order) | mean measured current (`current`, mA — mean of `current_samples_ma`, signed by the commanded `current_setpoint`) vs. setpoint (`current_setpoint`) per wheel |
| Boot | 1 plot | boot count (`reboot_count`) as a step signal |

Wheel telemetry is sourced from the four `CcmTelemetry` motors in
`ExtendedTelemetry` and is always present (never gapped by control mode); a
wheel's `current` is NaN only when its per-cycle current-sample buffer is empty.

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
**ROS2 Topic Subscriber** streaming plugin (tick **"Use header stamp"** to plot
against robot time), subscribe to `/controls_analysis`, and load the layout:
`Layout → File → share/ateam_controls_analysis/plotjuggler/controls_analysis.xml`.

### From a ROS bag

There are two ways to analyze a recorded bag.

#### Native full-bag load (recommended)

Convert the recorded `ExtendedTelemetry` bag into a new bag that contains the
flat `/controls_analysis` signals, then open that bag **directly** in PlotJuggler
(File → Open, no node, no streaming). This loads the entire timeline at once so
you can scrub the whole run:

```bash
ros2 run ateam_controls_analysis controls_analysis_bag_convert \
    /path/to/game_bag --robot-id 0
# writes /path/to/game_bag_controls_analysis
```

Then in PlotJuggler open `game_bag_controls_analysis` and load the bundled
layout (`Layout → File → share/ateam_controls_analysis/plotjuggler/controls_analysis.xml`).
The layout binds to the same `/controls_analysis` topic, so it works identically
for the converted bag and the live stream.

The converter reconstructs the robot-time axis by default (same as the node's
`use_robot_time=true`): output message timestamps advance by the robot's own
elapsed microseconds and re-ground on reboot, independent of how the bag was
recorded. Pass `--no-use-robot-time` to keep the original bag timestamps.

Converter options:

| Flag | Default | Description |
|------|---------|-------------|
| `input_bag` (positional) | — | Path to the recorded bag. |
| `-o`, `--output` | `<input>_controls_analysis` | Output bag path. |
| `--robot-id` | `0` | Robot whose telemetry to convert. |
| `--input-topic` | `/robot_feedback/extended/robot{id}` | Override input topic. |
| `--output-topic` | `/controls_analysis` | Output topic. |
| `--use-robot-time` / `--no-use-robot-time` | robot time | Output timeline. |
| `--reboot-reset-threshold-us` | `1000000` | Backward µs jump treated as a reboot. |
| `--input-storage-id` | auto | Input storage plugin (empty = auto-detect). |
| `--output-storage-id` | input's | Output storage plugin (default: match input). |

> Unlike the streaming node, the converter applies **no heartbeat** — for an
> offline full-bag load pure NaN gaps are correct (there is no live buffer whose
> time axis could stretch).

#### Replay + stream

Alternatively, play the bag through the node and stream into PlotJuggler (useful
for watching a run unfold at a chosen rate):

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
| `output_topic` | `/controls_analysis` | Topic to publish the analysis message on. |
| `reliability` | `reliable` | Telemetry subscription QoS: `reliable` or `best_effort`. |
| `use_robot_time` | `true` | Stamp the output header with the reconstructed robot timeline (see below). When `false`, uses the ROS receive time. |
| `reboot_reset_threshold_us` | `1000000` | Backward jump in the robot µs counter treated as a reboot. |
| `heartbeat_enabled` | `true` | Periodically publish `0.0` on mode-dependent curves so they keep trimming while streaming (see below). |
| `heartbeat_period_sec` | `1.0` | Heartbeat interval, measured on the (robot) plot timeline. |

## Heartbeat (streaming: keep idle curves from stretching the time axis)

PlotJuggler **discards NaN/inf samples** and trims each series only relative to
that series' own newest sample. A software-command or per-mode-split curve that
goes idle (its mode is inactive) therefore stops receiving finite points, its
stale tail never trims, and it anchors the left edge of the plot — stretching
the streaming view so your live buffer shrinks to a sliver on the right.

To avoid this, `heartbeat_enabled` (default `true`) publishes `0.0` on the
mode-dependent curves (the `*_cmd` single-series, the per-mode `*_cmd_*` splits,
and the per-mode `*_traj_*` splits) once per `heartbeat_period_sec` of plot time
whenever they are inactive. This keeps those series advancing so PlotJuggler
trims them and the trailing window stays put. Trade-off: idle curves show a
`0.0` dot roughly once per second (visible in Dots mode) instead of a pure gap.
Estimate, single-series trajectory, firmware-output, and measurement curves are
never gapped and so are never heartbeated. Set `heartbeat_enabled:=false` to
restore pure NaN gaps.

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

### Reboot indicator

`reboot_count` (uint32) is a monotonically increasing staircase of reboots seen
so far. The bundled layout plots it as a step signal in the **Boot** view, so
each reboot shows as a step up on the shared time axis. (PlotJuggler has no true
data-driven full-height vertical line; the `reboot_count` staircase is the
practical marker.)

## PlotJuggler layout

A single checked-in layout lives at `plotjuggler/controls_analysis.xml` and
binds to the fixed `/controls_analysis` topic (independent of which robot the
node is analyzing). Load it via `Layout → File`.

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
