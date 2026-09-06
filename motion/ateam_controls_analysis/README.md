# ateam_controls_analysis

Foxglove-based controls analysis for the robot fleet.

All analysis happens **inside Foxglove** via a bundled
[extension](foxglove/controls_analysis_extension): there is no ROS republisher
node, no derived message type, and no bag conversion step. Open a recorded bag
(or connect to a live ROS graph through
[`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge)), load a
layout, and pick a robot/team.

The extension registers three **topic converters** that read every robot's raw
topics and emit only the selected robot's converted message (driven by the `robot`
global variable, plus `team` for vision) onto the flat topics the layouts plot:

| Reads (all robots) | → Output topic (layouts plot) | Computation |
|---|---|---|
| `/robot_feedback/extended/robot0..15` | `/analysis_telem` | `ControlsAnalysis`: per-dim pos/vel/accel, per-mode-colored trajectory splits, `cmd_echo` command, per-wheel velocity + signed-mean current |
| `/robot_motion_commands/robot0..15` | `/analysis_control` | `MotionCommand`: pre-send software command, routed per body control mode |
| `/{blue,yellow}_team/robot0..15` | `/analysis_vision` | `VisionState` `{x,y,theta,visible}`, `theta` = quaternion yaw |

Each converter uses `watchVariables` and filters its inputs to the selected robot
(and team). Only global-frame command modes are plotted (no local→global
rotation); the per-mode-colored trajectory curves indicate the active mode. The
conversion is **stateless** — one message in, one out; plots use each message's
receive (log) time as the x-axis.

## Contents

- `foxglove/controls_analysis_extension/` — the Foxglove extension: 3 topic
  converters selecting the robot/`team` and emitting `/analysis_telem`,
  `/analysis_control`, `/analysis_vision`.
- `foxglove/controls_analysis_coach_time.json`,
  `foxglove/controls_analysis_robot_time.json` — the two bundled layouts
  (coach/PC time vs. robot time; see [layouts](#foxglove-layouts)).

## Views

Three body-frame views (X, Y, Theta). Each view is three vertically stacked
plots sharing a time axis:

| Plot | Derivative | Curves (labels) |
|------|-----------|--------|
| Position | 1st | per-mode trajectory `traj_*_telem`, software cmd `cmd` + round-tripped `cmd_telem`, fresh vision `vision` + round-tripped `vision_telem`, estimate `est_telem`, EKF state `ekf_telem` (hidden by default) |
| Velocity | 2nd | per-mode trajectory `traj_*_telem`, software cmd `cmd` + round-tripped `cmd_telem`, gyro `gyro_telem` (theta only), estimate `est_telem`, EKF state `ekf_telem` (hidden by default) |
| Acceleration | 3rd | software cmd `cmd` + round-tripped `cmd_telem`, firmware output `accel_u_telem` + friction-compensated `accel_u_fric_comp_telem`, IMU `imu_telem` (x/y only) |

### Software-side vs. round-tripped overlays (round-trip delay)

Two signals are plotted as a **pair** — the software-side value and the same value
after a round trip to the robot and back through `ExtendedTelemetry` (delayed,
with jitter). By default the software-side curve is shown and the round-tripped
one is hidden; toggle the latter on to read the round-trip delay. Colors follow
**dark = software / light = robot**:

| Signal | Software-side (shown) | Round-tripped (hidden) |
|--------|-----------------------|------------------------|
| Command (pos/vel/accel plots) | `cmd`, dark blue `#0000ff`, from `/analysis_control` | `cmd_telem`, light blue `#6666ff`, from `/analysis_telem` (`cmd_echo`) |
| Vision (position plots) | `vision`, mint `#3eb489`, from `/analysis_vision` (`theta` = quaternion yaw) | `vision_telem`, cyan `#00dac7`, from `/analysis_telem` (`vision_pose`, emitted only when the telemetry `vision_update` bit is set) |

Two additional wheel views share the same time axis:

| View | Layout | Curves (labels) |
|------|--------|--------|
| Velocity | 4 stacked plots (front-left, back-left, back-right, front-right) | setpoint `vel_setpoint_telem` vs. measured `vel_telem` (rad/s) per wheel |
| Current | 4 stacked plots (same wheel order) | setpoint `current_setpoint_telem` vs. measured `current_telem` (mA — signed mean of `current_samples_ma`) per wheel |

Wheel telemetry is sourced from the four `CcmTelemetry` motors in
`ExtendedTelemetry`; a wheel's `current` is NaN only when its per-cycle
current-sample buffer is empty.

## Body control mode → command mapping

Only **global-frame** modes plot a command curve; local-frame commands
(`BCM_LOCAL_*`) are not plotted (no local→global rotation). This applies to both
the software command (`/analysis_control`) and the round-tripped `cmd_echo`
(`/analysis_telem`).

| Mode | Value | Command curve |
|------|-------|---------------|
| `BCM_GLOBAL_POSITION` | 10 | position plots |
| `BCM_GLOBAL_VELOCITY` | 11 | velocity plots |
| `BCM_GLOBAL_ACCEL` | 13 | acceleration plots |
| `BCM_LOCAL_VELOCITY` (12), `BCM_LOCAL_ACCEL` (14) | | not plotted |
| all others (OFF, ESTOP, pivot, line) | | no command curve |

The reference trajectory is drawn as one per-mode-colored copy for **every** mode
that produces a trajectory setpoint (global position/velocity, local velocity, and
the pivot/line modes), so the trajectory curve indicates the active mode even for
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

Open the recorded `ExtendedTelemetry` bag **directly** in Foxglove (Open local
file). This loads the entire timeline at once so you can scrub the whole run;
the converter runs on the raw telemetry topics, so no pre-processing is needed.
Foxglove loads `.mcap` most smoothly.

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

Two tabbed layouts are checked in under `foxglove/` (five tabs each: X / Y /
Theta / Velocity / Current), binding to the converter output topics
`/analysis_telem`, `/analysis_control`, `/analysis_vision` (see
[Selecting which robot is displayed](#selecting-which-robot-is-displayed)). Both
draw lines (`showLine: true`) and carry the same curves; they differ only in the
**time basis** of the X axis:

- **`controls_analysis_coach_time.json`** — X axis is **coach/PC time** (message
  receive time, `timestampMethod: receiveTime`). Shows every curve, including the
  software-side `/analysis_control` command and `/analysis_vision` curves.
- **`controls_analysis_robot_time.json`** — X axis is **robot time** (see
  [Robot time](#robot-time)). Telem-only.

Load either via **Layout → Import from file…** →
`share/ateam_controls_analysis/foxglove/<layout>.json` (or straight from this
source tree).

Each body control mode has a consistent color across every plot, so mode
transitions (e.g. `BCM_POINT_PIVOT` → `BCM_HEADING_LINE`) are visible as a color
change with a gap on the reference-trajectory curves. Foxglove renders NaN as a
gap, so the per-mode-split curves break cleanly at mode transitions.

> **Tabs when live streaming.** Both layouts are tabbed, and Foxglove **unmounts
> inactive tabs**; a live WebSocket source has no historical backfill, so
> switching tabs mid-stream starts that view empty and only fills going forward.
> A loaded bag backfills instantly on switch since the whole timeline is present.

### Robot time

`controls_analysis_robot_time.json` plots against the **robot's own clock**
instead of coach/PC receive time, which is what you want when inspecting the
1 kHz control loop / delayed-EKF timing. The converter emits a synthetic
`header.stamp` derived from the robot clock
(`ExtendedTelemetry.timestamp_us_lo/hi`) but **anchored onto the PC/Unix
timeline**: it captures the robot→PC offset (`pc_receive_time − robot_time`) from
the first packet of the selected robot and holds it, so the stamp sits on the
same absolute timescale as Foxglove's timeline (rather than ~10⁹ s away at
seconds-since-boot) while still advancing on the robot's own jitter-free clock.
The layout stays in **Timestamp** X-axis mode but sets every series'
`timestampMethod` to **`headerStamp`**, so it plots against that anchored robot
time while remaining in the mode where `isSynced` works: cross-panel X sync, the
shared hover cursor, and the "hold **h**" horizontal cursor all function, and
because every panel reads the same `/analysis_telem` header clock, hovering one
panel at robot-time *T* lines up with every other panel at *T* — while still live
streaming.

The anchor is captured per selected robot (it re-anchors when you change the
`robot` variable) and bakes one packet's radio latency into a constant offset;
later packets are placed by robot elapsed time, so radio jitter doesn't move the
X axis. Telem-only because `/analysis_control` / `/analysis_vision` have no
robot-time header. The global playback scrubber still reflects PC receive time
(playback order is unchanged). Reboots (robot clock jumps back to ~0) show as a
jump on the X axis. The converter also exposes the raw robot seconds-since-boot as
`/analysis_telem.robot_time_s` if you prefer a custom message-path X axis instead.

Regenerate `controls_analysis_robot_time.json` after editing the coach-time
layout with:

```bash
cd foxglove
python3 - <<'EOF'
import json, copy
d = copy.deepcopy(json.load(open('controls_analysis_coach_time.json')))
def conv(o):
    if isinstance(o, dict):
        if 'paths' in o and 'xAxisVal' in o:
            o['paths'] = [p for p in o['paths'] if str(p.get('value','')).startswith('/analysis_telem')]
            o['xAxisVal'] = 'timestamp'; o['isSynced'] = True
            for p in o['paths']: p['timestampMethod'] = 'headerStamp'
            return
        for v in o.values(): conv(v)
    elif isinstance(o, list):
        for v in o: conv(v)
conv(d)
json.dump(d, open('controls_analysis_robot_time.json','w'), indent=2); open('controls_analysis_robot_time.json','a').write('\n')
EOF
```

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

The extension solves this in its **topic converters**: each reads every robot's
raw topics and, using `watchVariables`, emits only the selected robot's converted
message onto `/analysis_telem`, `/analysis_control`, `/analysis_vision`. Change
these variables in Foxglove's **Variables** tab and every panel re-points:

| Variable | Default | Meaning |
|----------|---------|---------|
| `robot` | `0` | Robot id shown by every panel. |
| `team` | `blue` | Our team color, selecting `/{team}_team/robot{id}` for the vision overlay (`blue` / `yellow`). |

This works identically for a **loaded bag** (full backfill) and a live connection.
Team is a manual variable — no referee introspection.

> Because a topic converter's `inputTopics` is a static list, Foxglove preloads
> all of a converter's declared inputs (all 16 robots per stream) even though only
> the selected one is emitted. If bag load time becomes a problem, crop the bag to
> the robots/time range you need before opening it.

## Relationship to the legacy workflow

This supersedes the bag → NPZ → matplotlib workflow under
`motion/ateam_controls/controls/analysis` (`telem_visualize.py`,
`telem_bag2np.py`, `record_and_visualize.sh`, and the visualization step of
`controller_tune.py`), which are now marked deprecated. The signal-generation
(`signal_input.py`), parameter-upload, and `accel_tuning` tooling there remain in
use — record an `ExtendedTelemetry` bag from those trials and open it in
Foxglove.
