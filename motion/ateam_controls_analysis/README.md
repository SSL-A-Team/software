# ateam_controls_analysis

Foxglove-based controls analysis for the robot fleet, driven entirely by
`ateam_radio_msgs/ExtendedTelemetry`.

A ROS 2 node (`controls_analysis_node`) subscribes to every robot's
`/robot_feedback/extended/robot{id}` and republishes a flat, Foxglove-friendly
`ateam_controls_analysis_msgs/ControlsAnalysis` message **per robot** on
`/controls_analysis/robot{id}`. The node performs the computations the plot
viewer can't do natively:

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
and writes a single new bag containing `/controls_analysis/robot{id}` for every
robot, which Foxglove can open **directly** (full-bag load) — no node, no
replay-streaming. See [From a ROS bag](#from-a-ros-bag).

## Packages

- `ateam_controls_analysis_msgs` — the `ControlsAnalysis` message.
- `ateam_controls_analysis` — the republisher node, the offline bag converter,
  the Foxglove layouts + robot-selector extension, and the launch file.

## Views

Three body-frame views (X, Y, Theta). Each view is three vertically stacked
plots sharing a time axis:

| Plot | Derivative | Curves |
|------|-----------|--------|
| Position | 1st | state estimate, reference trajectory (per-mode colored), vision measurement, software cmd (`BCM_GLOBAL_POSITION`) |
| Velocity | 2nd | state estimate, reference trajectory (per-mode colored), software cmd (`BCM_GLOBAL_VELOCITY`/`BCM_LOCAL_VELOCITY`), gyro (theta only) |
| Acceleration | 3rd | firmware output accel + friction-compensated output accel, software cmd (`BCM_GLOBAL_ACCEL`/`BCM_LOCAL_ACCEL`), IMU accel (x/y only) |

Three additional wheel/boot views share the same time axis:

| View | Layout | Curves |
|------|--------|--------|
| Velocity | 4 stacked plots (front-left, back-left, back-right, front-right) | measured wheel velocity (`vel`, rad/s) vs. setpoint (`vel_setpoint`) per wheel |
| Current | 4 stacked plots (same wheel order) | mean measured current (`current`, mA — mean of `current_samples_ma`, signed by the commanded `current_setpoint`) vs. setpoint (`current_setpoint`) per wheel |
| Boot | 1 plot | boot count (`reboot_count`) as a staircase signal |

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

Start the node (it analyzes all robots 0..15 by default) and connect Foxglove to
the ROS graph via [`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge):

```bash
sudo apt install ros-jazzy-foxglove-bridge     # one-time
ros2 run foxglove_bridge foxglove_bridge       # serves ws://localhost:8765
ros2 launch ateam_controls_analysis controls_analysis.launch.py
```

In Foxglove: **Open connection → Foxglove WebSocket → `ws://localhost:8765`**,
then load a layout and select a robot (see [Foxglove layouts](#foxglove-layouts)
and [Selecting which robot is displayed](#selecting-which-robot-is-displayed)).

### From a ROS bag

Convert the recorded `ExtendedTelemetry` bag into a single new bag containing the
flat `/controls_analysis/robot{id}` signals for every robot, then open that bag
**directly** in Foxglove (Open local file, no node, no streaming). This loads
the entire timeline at once so you can scrub the whole run. Foxglove loads
`.mcap` most smoothly, so prefer an mcap output:

```bash
ros2 run ateam_controls_analysis controls_analysis_bag_convert \
    /path/to/game_bag --output-storage-id mcap
# writes /path/to/game_bag_controls_analysis  (all robots, one bag)
```

To convert only a subset of robots, pass `--robot-ids`:

```bash
ros2 run ateam_controls_analysis controls_analysis_bag_convert \
    /path/to/game_bag --robot-ids 0 2 3
```

The converter reconstructs the robot-time axis by default (same as the node's
`use_robot_time=true`): output message timestamps advance by each robot's own
elapsed microseconds and re-ground on reboot, independent of how the bag was
recorded. Pass `--no-use-robot-time` to keep the original bag timestamps.

Converter options:

| Flag | Default | Description |
|------|---------|-------------|
| `input_bag` (positional) | — | Path to the recorded bag. |
| `-o`, `--output` | `<input>_controls_analysis` | Output bag path (one combined bag). |
| `--robot-ids` | all robots in bag | Robots to convert (space-separated). |
| `--use-robot-time` / `--no-use-robot-time` | robot time | Output timeline. |
| `--reboot-reset-threshold-us` | `1000000` | Backward µs jump treated as a reboot. |
| `--input-storage-id` | auto | Input storage plugin (empty = auto-detect). |
| `--output-storage-id` | input's | Output storage plugin (default: match input). |

> Unlike the streaming node, the converter applies **no heartbeat** — for an
> offline full-bag load pure NaN gaps are correct (there is no live buffer whose
> time axis could stretch).

#### Replay + stream

Alternatively, play the original bag through the node and stream into Foxglove
(useful for watching a run unfold at a chosen rate):

```bash
ros2 run foxglove_bridge foxglove_bridge
ros2 launch ateam_controls_analysis controls_analysis.launch.py \
    bag:=/path/to/bag rate:=1.0
```

or, if you already converted the bag (it carries `/controls_analysis/robot{id}`
directly), just play it under the bridge with no node:

```bash
ros2 run foxglove_bridge foxglove_bridge
ros2 bag play /path/to/game_bag_controls_analysis
```

> Tip: a robot may only connect partway through a game. Use
> `ros2 bag play ... --start-offset <seconds>` to jump ahead, and `--rate <n>`
> to fast-forward.

## Foxglove layouts

Two layouts are checked in under `foxglove/`; both bind to the single alias topic
`/controls_analysis_selected` (see below) and default all curves to points-only
(`showLine: false`).

- **`controls_analysis.json`** — tabbed: six tabs (X / Y / Theta / Velocity /
  Current / Boot), one view at a time.
- **`controls_analysis_singleview.json`** — every plot on one always-mounted
  grid, legends off to save space.

Load either via **Layout → Import from file…** →
`share/ateam_controls_analysis/foxglove/<layout>.json`.

Each body control mode has a consistent color across every plot, so mode
transitions (e.g. `BCM_POINT_PIVOT` → `BCM_HEADING_LINE`) are visible as a color
change with a gap on the reference-trajectory curves.

> **Tabbed vs. single-view when live streaming.** Foxglove **unmounts inactive
> tabs**, and a live WebSocket source has no historical backfill, so switching
> tabs mid-stream starts that view empty and only fills going forward. For live
> streaming prefer the single-view layout (all plots stay mounted and keep
> collecting). The tabbed layout is ideal for a loaded bag file, where the whole
> timeline is present so tabs backfill instantly on switch.

The layouts plot every series against the message `header.stamp` (Foxglove path
`timestampMethod: headerStamp`), matching the reconstructed robot-time axis
(`use_robot_time`, the default) — this works for both a converted bag and a live
stream. Foxglove renders NaN as a gap, so the per-mode-split curves break cleanly
at mode transitions.

The layouts also ship with a **following (sliding) window** of 10 s
(`followingViewWidth`): during playback each plot shows only the trailing 10 s
with the playhead pinned at the right edge, giving a streaming-style view even on
a loaded bag. Adjust it per plot via the gear icon → **X Axis → Following window
(seconds)**, or change `followingViewWidth` in the layout JSON. (Panning a plot
temporarily disengages the following window until you resume/seek.)

> **Playback cursor note.** Foxglove's playback cursor (the vertical current-time
> line) tracks the source's log/receive time, not `header.stamp`. On a
> **converted bag** the two are identical (the converter writes each message's
> bag log-time and `header.stamp` to the same reconstructed robot time), so the
> cursor lines up. On a **live** stream the bridge stamps log-time with wall
> clock, so the cursor won't track the robot-time axis while scrubbing (the
> plots still follow the live edge). If you specifically need the cursor to line
> up on a live stream, switch a plot's timestamp method to receive time in
> Foxglove (trading the robot-time axis for wall-clock).

## Selecting which robot is displayed

Each robot has its own `/controls_analysis/robot{id}` topic. Foxglove message
paths can use a `$variable` in field/index/filter positions but **not** in the
topic-name position, so a variable alone can't retarget a per-robot topic.

The bundled Foxglove extension in
[`foxglove/robot_selector_extension/`](foxglove/robot_selector_extension) solves
this with a **topic alias**: it maps `/controls_analysis/robot{robot}` (driven by
the `robot` global variable, default `0`) onto the stable topic
`/controls_analysis_selected` that the layouts subscribe to. Change the `robot`
variable in Foxglove's **Variables** tab and every panel re-points instantly.

Because a topic alias operates at Foxglove's data-source layer, it works
identically for a **loaded bag** (full backfill — the newly selected robot's
plots immediately show the whole timeline, no reload) and a live connection.

Install it once (requires Node.js):

```bash
cd foxglove/robot_selector_extension
npm install
npm run local-install    # builds + installs into your local Foxglove
```

Then set the `robot` variable to the id you want. See that directory's README
for details.

## Node parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ids` | `0..15` | Robots to analyze; each publishes `/controls_analysis/robot{id}`. |
| `input_topic_template` | `/robot_feedback/extended/robot{id}` | Input topic template (`{id}` → robot id). |
| `output_topic_template` | `/controls_analysis/robot{id}` | Output topic template (`{id}` → robot id). |
| `reliability` | `reliable` | Telemetry subscription QoS: `reliable` or `best_effort`. |
| `use_robot_time` | `true` | Stamp the output header with the reconstructed robot timeline (see below). When `false`, uses the ROS receive time. |
| `reboot_reset_threshold_us` | `1000000` | Backward jump in the robot µs counter treated as a reboot. |
| `heartbeat_enabled` | `false` | Periodically publish `0.0` on mode-dependent curves (see below). Off by default — Foxglove doesn't need it. |
| `heartbeat_period_sec` | `1.0` | Heartbeat interval, measured on the (robot) plot timeline. |

## Heartbeat (optional, for trailing-buffer stream viewers)

Some streaming plot viewers discard NaN/inf samples and trim each series only
relative to that series' own newest sample. A software-command or per-mode-split
curve that goes idle then stops receiving finite points, its stale tail never
trims, and it can anchor the left edge of the plot.

`heartbeat_enabled` publishes `0.0` on the mode-dependent curves (the `*_cmd`
single-series, the per-mode `*_cmd_*` splits, and the per-mode `*_traj_*` splits)
once per `heartbeat_period_sec` whenever they are inactive, keeping those series
advancing. **Foxglove plots against a fixed time range rather than a per-series
trailing buffer, so it doesn't need the heartbeat — it is disabled by default.**
Estimate, single-series trajectory, firmware-output, and measurement curves are
never gapped and so are never heartbeated.

## Time axis (reconstructed robot time)

Each `ExtendedTelemetry` carries the robot's own 64-bit microsecond uptime
counter (`timestamp_us_hi`/`lo`). With `use_robot_time:=true` (default) the node
builds the plot time axis from it, **independently per robot**:

- The first message is grounded at its ROS receive time.
- Subsequent messages advance by the robot's own elapsed time, so the timeline
  reflects **true robot time** — replaying a bag at `--rate 25` still shows the
  real per-sample timing, not the compressed playback timing.
- On a **reboot** the counter jumps backward (uptime restarts near zero). The
  node detects this (drop > `reboot_reset_threshold_us`), re-grounds the axis at
  that message's ROS time, and continues forward from there.

The reconstructed time is written to `header.stamp`; the bundled layouts plot
against it (`headerStamp`).

### Reboot indicator

`reboot_count` (uint32) is a monotonically increasing staircase of reboots seen
so far. The **Boot** view plots it as a staircase, so each reboot shows as a step
up on the shared time axis.

## Relationship to the legacy workflow

This supersedes the bag → NPZ → matplotlib workflow under
`motion/ateam_controls/controls/analysis` (`telem_visualize.py`,
`telem_bag2np.py`, `record_and_visualize.sh`, and the visualization step of
`controller_tune.py`), which are now marked deprecated. The signal-generation
(`signal_input.py`), parameter-upload, and `accel_tuning` tooling there remain in
use — record a bag from those trials and replay it through this node.
