# ateam_controls_analysis

Foxglove-based controls analysis for the robot fleet, driven entirely by
`ateam_radio_msgs/ExtendedTelemetry`.

All analysis happens **inside Foxglove** via a bundled
[extension](foxglove/controls_analysis_extension): there is no ROS republisher
node, no derived message type, and no bag conversion step. Open a recorded
`ExtendedTelemetry` bag (or connect to a live ROS graph through
[`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge)), load a
layout, and pick a robot.

The extension registers a **stateless topic converter** that reads the fleet's
`ateam_radio_msgs/msg/ExtendedTelemetry` topics and produces one dedicated in-app
topic, `/controls_analysis_selected` (schema
`ateam_controls_analysis/ControlsAnalysis`), that the layouts bind to. Per
telemetry message it:

- flattens the sample into per-dimension (x / y / theta) position / velocity /
  acceleration series and per-wheel velocity / current series,
- routes the software command (`cmd_echo`) onto the derivative implied by the
  active body control mode, rotating local-frame commands (`BCM_LOCAL_*`) into
  the global frame using the robot's theta estimate,
- splits the reference trajectory and command into **per-mode series** so each
  body control mode is drawn in its own color and the reference/command curves
  **change color as the active control mode changes**,
- inserts gaps (NaN) for modes/measurements that don't apply.

The conversion is **stateless** — one telemetry message maps to exactly one
output message, never comparing against a previous message. There is no
robot-time reconstruction, no reboot tracking, and no heartbeat; plots use each
message's receive (log) time as the x-axis.

## Contents

- `foxglove/controls_analysis_extension/` — the Foxglove extension. Two topic
  converters, both selecting the robot via the `robot` global variable:
  - `ExtendedTelemetry` → `/controls_analysis_selected` (the main analysis), and
  - the friendly team's `/{color}_team/robot{id}` → `/vision_state_selected` (the
    fresh vision estimate overlay; friendly color auto-detected from the referee).
- `foxglove/controls_analysis.json`, `foxglove/controls_analysis_singleview.json`
  — the two bundled layouts (plus `*_lines.json` variants; see [layouts](#foxglove-layouts)).

## Views

Three body-frame views (X, Y, Theta). Each view is three vertically stacked
plots sharing a time axis:

| Plot | Derivative | Curves |
|------|-----------|--------|
| Position | 1st | state estimate, reference trajectory (per-mode colored), vision measurement, software cmd (`BCM_GLOBAL_POSITION`), **fresh vision estimate (off by default; see below)** |
| Velocity | 2nd | state estimate, reference trajectory (per-mode colored), software cmd (`BCM_GLOBAL_VELOCITY`/`BCM_LOCAL_VELOCITY`), gyro (theta only) |
| Acceleration | 3rd | firmware output accel + friction-compensated output accel, software cmd (`BCM_GLOBAL_ACCEL`/`BCM_LOCAL_ACCEL`), IMU accel (x/y only) |

### Fresh vision estimate overlay (round-trip delay)

Each position plot (x / y / theta) carries an extra **mint**-colored curve, **off
by default**, sourced from `/vision_state_selected` (see [the extension](foxglove/controls_analysis_extension)).
This is the *fresh* software-side vision estimate (`ateam_msgs/VisionStateRobot`
on the friendly team's `/{color}_team/robot{id}`) — the pose the software actually
uses to make decisions. The cyan `pos_vision` curve on the same plot is the *same*
measurement after it has round-tripped to the robot and returned in
`ExtendedTelemetry` (delayed, with jitter). Toggle the mint curve on to compare
the two and read off the round-trip delay. theta is the yaw of the vision pose
quaternion, computed by the extension.

Two additional wheel views share the same time axis:

| View | Layout | Curves |
|------|--------|--------|
| Velocity | 4 stacked plots (front-left, back-left, back-right, front-right) | measured wheel velocity (`vel`, rad/s) vs. setpoint (`vel_setpoint`) per wheel |
| Current | 4 stacked plots (same wheel order) | mean measured current (`current`, mA — mean of `current_samples_ma`, signed by the commanded `current_setpoint`) vs. setpoint (`current_setpoint`) per wheel |

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

The reference trajectory is split per mode for **all** modes that produce a
trajectory setpoint (global position/velocity, local velocity, and the
pivot/line modes), so those curves also color-by-mode even where the software
command has no direct x/y/theta mapping.

Measurements are always placed on their physically correct derivative:
vision → position, gyro → theta velocity, IMU → x/y acceleration. The state
estimate is always available on position and velocity; the firmware output
acceleration (raw and friction-compensated) is always available on acceleration.

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

Four layouts are checked in under `foxglove/`; all bind to the single converter
output topic `/controls_analysis_selected` (see [Selecting which robot is
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
The lines variants are generated from the points-only layouts by setting every
`showLine` to `true`; regenerate them after editing a base layout with:

```bash
cd foxglove
jq '(.. | objects | select(has("showLine"))).showLine = true' \
   controls_analysis_singleview.json > controls_analysis_singleview_lines.json
jq '(.. | objects | select(has("showLine"))).showLine = true' \
   controls_analysis.json > controls_analysis_lines.json
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

Each robot has its own `/robot_feedback/extended/robot{id}` telemetry topic.
Foxglove message paths can use a `$variable` in field/index/filter positions but
**not** in the topic-name position, so a variable alone can't retarget a
per-robot topic.

The bundled extension solves this in its **topic converter**: it listens to every
robot's telemetry topic but emits only the one selected by the `robot` global
variable (default `0`) onto the single output topic `/controls_analysis_selected`
that the layouts subscribe to. Change the `robot` variable in Foxglove's
**Variables** tab and every panel re-points instantly.

Because the converter runs at Foxglove's data-source layer, it works identically
for a **loaded bag** (full backfill — the newly selected robot's plots
immediately show the whole timeline, no reload) and a live connection.

The same `robot` variable also drives the fresh-vision-estimate overlay: a second
converter emits the selected robot's `/vision_state_selected`. It picks the
**friendly** team's `/{color}_team/robot{id}` automatically by matching our team
name against `/referee_messages` (the same logic the stack uses at runtime).
Overrides via Foxglove **Variables**:

| Variable | Default | Meaning |
|----------|---------|---------|
| `robot` | `0` | Robot id shown by every panel. |
| `friendly_team` | `auto` | `auto` detects our color from the referee; set `blue`/`yellow` to force it (e.g. bags without `/referee_messages`). |
| `team_name` | `A-Team` | Our team name, matched against the referee teams when `friendly_team` is `auto`. |

## Relationship to the legacy workflow

This supersedes the bag → NPZ → matplotlib workflow under
`motion/ateam_controls/controls/analysis` (`telem_visualize.py`,
`telem_bag2np.py`, `record_and_visualize.sh`, and the visualization step of
`controller_tune.py`), which are now marked deprecated. The signal-generation
(`signal_input.py`), parameter-upload, and `accel_tuning` tooling there remain in
use — record an `ExtendedTelemetry` bag from those trials and open it in
Foxglove.
