# ateam_controls_analysis

Foxglove-based controls analysis for the robot fleet.

All analysis happens **inside Foxglove** via a bundled
[extension](foxglove/controls_analysis_extension): there is no ROS republisher
node, no derived message type, and no bag conversion step. Open a recorded bag
(or connect to a live ROS graph through
[`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge)), load a
layout, and pick a robot.

The extension is deliberately thin. It **selects** the robot/team you're viewing
with **topic aliases** (driven by the `robot` and `team` global variables) that
map each raw per-robot source topic onto a stable name the layouts bind to:

| Selected topic | Source (raw) | Driven by |
|---|---|---|
| `/analysis_telem_selected` | `/robot_feedback/extended/robot{robot}` | `robot` |
| `/analysis_command_selected` | `/robot_motion_commands/robot{robot}` | `robot` |
| `/analysis_vision_selected` | `/{team}_team/robot{robot}` | `robot`, `team` |

Because an alias maps a stable name onto **one real source topic**, only the
selected robot's data is loaded (no converter fan-in) — load time scales with one
robot, not the fleet. Everything the plots need is then either a **raw field** or
a **`{body_control_mode==N}` message-path filter** in the layout (per-mode
trajectory coloring, command gating). The only two values a message path can't
compute are added by tiny **topic converters** fed by the selection aliases (one
input topic each, so no fan-in): the fresh vision **heading**
(`/analysis_vision_theta_selected.theta` = yaw of the vision pose quaternion) and
the per-wheel **current** (`/analysis_wheelcurrent_selected.{wheel}.current`,
signed mean of the current-sample buffer). Plots use each message's receive (log)
time as the x-axis.

## Contents

- `foxglove/controls_analysis_extension/` — the Foxglove extension: topic aliases
  for `robot`/`team` selection, plus two topic converters fed by those aliases
  (vision → heading `/analysis_vision_theta_selected`, telemetry → per-wheel
  current mean `/analysis_wheelcurrent_selected`).
- `foxglove/controls_analysis.json`, `foxglove/controls_analysis_singleview.json`
  — the two bundled layouts (plus `*_lines.json` variants; see [layouts](#foxglove-layouts)).

## Views

Three body-frame views (X, Y, Theta). Each view is three vertically stacked
plots sharing a time axis:

| Plot | Derivative | Curves |
|------|-----------|--------|
| Position | 1st | state estimate, reference trajectory (per-mode colored), fresh vision estimate + round-tripped vision, pre-send + round-tripped software cmd (`BCM_GLOBAL_POSITION`) |
| Velocity | 2nd | state estimate, reference trajectory (per-mode colored), pre-send + round-tripped software cmd (`BCM_GLOBAL_VELOCITY`/`BCM_LOCAL_VELOCITY`), gyro (theta only) |
| Acceleration | 3rd | firmware output accel + friction-compensated output accel, pre-send + round-tripped software cmd (`BCM_GLOBAL_ACCEL`/`BCM_LOCAL_ACCEL`), IMU accel (x/y only) |

### Pre-send vs. round-tripped overlays (round-trip delay)

Several signals appear **twice**: once as the value the software produced
(before it was sent to the robot) and once after a full round trip to the robot
and back through telemetry (delayed, with jitter). Overlaying the two shows the
round-trip delay. By **default the pre-send (software-side) curve is shown and
the round-tripped (robot-side) curve is hidden** — toggle the robot-side one on
to compare. The pre-send curve uses a **darker** shade of the round-tripped
curve's color:

| Signal | Pre-send (software-side, shown) | Round-tripped (robot-side, hidden) |
|--------|--------------------------------|------------------------------------|
| Vision (position plots) | mint `#3eb489`, from `/analysis_vision_selected` (`ateam_msgs/VisionStateRobot` on `/{team}_team/robot{id}`) | cyan `#00dac7`, `body_control_telemetry.vision_pose[i]` (vision echoed back in `ExtendedTelemetry`) |
| Command (pos / vel / accel plots) | blue `#0000ff`, from `/analysis_command_selected` (`ateam_msgs/RobotMotionCommand`) | light blue `#6666ff`, `body_control_telemetry.maneuver_*.cmd_echo.*` (`cmd_echo` in `ExtendedTelemetry`) |

Both curves are gated by a `{body_control_mode==N}` layout filter, so each only
draws while its mode is active. Only **global-frame** modes are plotted;
**local-frame** commands (`BCM_LOCAL_*`) are intentionally not plotted on either
side (no local→global rotation). The per-mode-colored reference-trajectory curves
still indicate the active mode, including for local modes. Vision `theta` is the
yaw of the pose quaternion, computed by the extension's vision topic converter. Command
curves draw as **points only** in every layout (including the `*_lines`
variants), since they are stepped/sparse.

Two additional wheel views share the same time axis:

| View | Layout | Curves |
|------|--------|--------|
| Velocity | 4 stacked plots (front-left, back-left, back-right, front-right) | measured wheel velocity (`vel`, rad/s) vs. setpoint (`vel_setpoint`) per wheel |
| Current | 4 stacked plots (same wheel order) | mean measured current (`current`, mA — mean of `current_samples_ma`, signed by the commanded `current_setpoint`) vs. setpoint (`current_setpoint`) per wheel |

Wheel telemetry is sourced from the four `CcmTelemetry` motors in
`ExtendedTelemetry`. Wheel velocity is a raw field; wheel `current` is the signed
mean of the per-cycle current-sample buffer, computed by the extension's schema
converter (NaN only when the buffer is empty).

## Body control mode → command mapping

Command curves are gated in the layouts by a `{body_control_mode==N}` filter, so
each command curve only draws while its mode is active. Only global-frame modes
are plotted (local-frame commands are not plotted — no local→global rotation):

| Mode | Value | Command curve |
|------|-------|---------------|
| `BCM_GLOBAL_POSITION` | 10 | position plots (`pose` / `cmd_echo.global_x…`) |
| `BCM_GLOBAL_VELOCITY` | 11 | velocity plots (`velocity` / `cmd_echo.global_xd…`) |
| `BCM_GLOBAL_ACCEL` | 13 | acceleration plots (`acceleration` / `cmd_echo.global_xdd…`) |
| `BCM_LOCAL_VELOCITY` (12), `BCM_LOCAL_ACCEL` (14) | | not plotted |
| all others (OFF, ESTOP, pivot, line) | | no command curve |

The reference **trajectory** is drawn as one filtered copy of `body_traj_pos[i]` /
`body_traj_vel[i]` per mode, each in the mode's color, for every mode that
produces a trajectory setpoint (global position/velocity, local velocity, and the
pivot/line modes) — so the trajectory curve indicates the active mode even for
modes whose command isn't plotted.

Measurements are placed on their physically correct derivative: vision →
position, gyro → theta velocity, IMU → x/y acceleration. The state estimate is on
position and velocity; the firmware output acceleration (raw and
friction-compensated) is on acceleration.

## Install the extension

Requires Node.js.

**Desktop app** — build + install directly:

```bash
cd foxglove/controls_analysis_extension
npm install
npm run local-install    # builds + installs into your local Foxglove
```

**Web app (app.foxglove.dev)** — package a `.foxe` and drag it in:

```bash
cd foxglove/controls_analysis_extension
npm install
npm run package          # produces ateam.ateam-controls-analysis-<version>.foxe
```

Then open app.foxglove.dev and drag the `.foxe` onto the window.

Restart / reload Foxglove afterward. See the extension
[README](foxglove/controls_analysis_extension/README.md) for details and
troubleshooting.

## Usage

### From a ROS bag

Open the recorded bag **directly** in Foxglove (Open local file). This loads the
entire timeline at once so you can scrub the whole run; the aliases and schema
converters run on the raw topics, so no pre-processing is needed. Foxglove loads
`.mcap` most smoothly.

> Tip: a robot may only connect partway through a game. Use Foxglove's playback
> controls to seek; the file source has full backfill.

### Live streaming

Connect Foxglove to the ROS graph via
[`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge):

```bash
sudo apt install ros-jazzy-foxglove-bridge     # one-time
ros2 run foxglove_bridge foxglove_bridge       # serves ws://localhost:8765
```

In Foxglove: **Open connection → Foxglove WebSocket → `ws://localhost:8765`**.

## Foxglove layouts

Four layouts are checked in under `foxglove/`; all bind to the `/analysis_*_selected`
alias topics (see [Selecting which robot is
displayed](#selecting-which-robot-is-displayed)).

- **`controls_analysis.json`** — tabbed: five tabs (X / Y / Theta / Velocity /
  Current), one view at a time.
- **`controls_analysis_singleview.json`** — every plot on one always-mounted
  grid, legends off to save space.

Both default all curves to **points-only** (`showLine: false`). Each also has a
**lines** variant that is identical except every curve draws a connecting line
(`showLine: true`):

- **`controls_analysis_lines.json`** — tabbed, lines.
- **`controls_analysis_singleview_lines.json`** — single-view, lines.

Foxglove has no live "toggle lines on all plots" control (`showLine` is a
per-series setting), so switching styles is done by importing the other variant.
The lines variants are generated from the points-only layouts by turning every
`showLine` on **except command curves** (which stay points, since they're
stepped/sparse); regenerate them after editing a base layout with:

```bash
cd foxglove
for base in controls_analysis_singleview controls_analysis; do
  jq 'walk(if type=="object" and has("showLine")
          then .showLine = (((.value|test("cmd_echo")) or (.value|startswith("/analysis_command_selected"))) | not)
          else . end)' "$base.json" > "${base}_lines.json"
done
```

Load any of them via **Layout → Import from file…** →
`share/ateam_controls_analysis/foxglove/<layout>.json` (or straight from this
source tree).

Each body control mode has a consistent color across every plot, so mode
transitions (e.g. `BCM_POINT_PIVOT` → `BCM_HEADING_LINE`) are visible as a color
change with a gap on the reference-trajectory curves.

> **Tabbed vs. single-view when live streaming.** Foxglove **unmounts inactive
> tabs**, and a live WebSocket source has no historical backfill, so switching
> tabs mid-stream starts that view empty and only fills going forward. For live
> streaming prefer the single-view layout (all plots stay mounted and keep
> collecting). The tabbed layout is ideal for a loaded bag file, where the whole
> timeline is present so tabs backfill instantly on switch.

The layouts plot every series against message **receive time**
(`timestampMethod: receiveTime`), so a converted per-message sample lands on the
log-time axis for both a loaded bag and a live stream. Foxglove renders NaN as a
gap, so the per-mode-split curves break cleanly at mode transitions.

The layouts also ship with a **following (sliding) window** of 10 s
(`followingViewWidth`): during playback each plot shows only the trailing 10 s
with the playhead pinned at the right edge. Adjust it per plot via the gear icon
→ **X Axis → Following window (seconds)**, or change `followingViewWidth` in the
layout JSON. (Panning a plot temporarily disengages the following window until
you resume/seek.)

## Selecting which robot is displayed

Each robot has its own raw per-robot topics (`/robot_feedback/extended/robot{id}`,
`/robot_motion_commands/robot{id}`, `/{team}_team/robot{id}`). Foxglove message
paths can use a `$variable` in field/index/filter positions but **not** in the
topic-name position, so a variable alone can't retarget a per-robot topic.

The extension solves this with **topic aliases**: it maps each selected raw topic
onto a stable `/analysis_*_selected` name that the layouts bind to, driven by the
`robot` and `team` global variables. Change them in Foxglove's **Variables** tab
and every panel re-points instantly.

| Variable | Default | Meaning |
|----------|---------|---------|
| `robot` | `0` | Robot id shown by every panel. |
| `team` | `blue` | Our team color, selecting `/{team}_team/robot{id}` for the fresh-vision overlay (`blue` / `yellow`). |

Because an alias resolves to **one real source topic**, only the selected robot's
data is loaded — load time scales with one robot, not the fleet — and it works
identically for a **loaded bag** (full backfill) and a live connection. (Team is
a manual variable; no referee introspection.)

## Relationship to the legacy workflow

This supersedes the bag → NPZ → matplotlib workflow under
`motion/ateam_controls/controls/analysis` (`telem_visualize.py`,
`telem_bag2np.py`, `record_and_visualize.sh`, and the visualization step of
`controller_tune.py`), which are now marked deprecated. The signal-generation
(`signal_input.py`), parameter-upload, and `accel_tuning` tooling there remain in
use — record an `ExtendedTelemetry` bag from those trials and open it in
Foxglove.
