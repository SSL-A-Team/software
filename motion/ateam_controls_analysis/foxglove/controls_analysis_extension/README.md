# Controls Analysis (Foxglove extension)

A [Foxglove extension](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
that powers the bundled controls-analysis layouts. It does the analysis **inside
Foxglove** — there is no ROS republisher node and no derived message type. Point
Foxglove at a recorded `ateam_radio_msgs/ExtendedTelemetry` bag (or a live
connection) and the layouts just work.

## What it does

It registers three **topic converters** that read every robot's raw topics and
emit, per stream, **only the selected robot's** converted message onto one stable
output topic the layouts bind to. It runs at Foxglove's data-source layer, so it
behaves identically for a loaded bag (full backfill) and a live connection.

| Output topic (layouts bind here) | Reads (all robots) | Schema / computation |
|---|---|---|
| `/analysis_telem` | `/robot_feedback/extended/robot0..15` (`ExtendedTelemetry`) | `ControlsAnalysis`: per-dim pos/vel/accel, per-mode-colored trajectory splits, `cmd_echo` command (global modes only), per-wheel velocity + signed-mean current. See [`src/controlsAnalysis.ts`](src/controlsAnalysis.ts). |
| `/analysis_control` | `/robot_motion_commands/robot0..15` (`RobotMotionCommand`) | `MotionCommand`: the pre-send software command routed per body control mode (global modes only). See [`src/motionCommand.ts`](src/motionCommand.ts). |
| `/analysis_vision` | `/{blue,yellow}_team/robot0..15` (`VisionStateRobot`) | `VisionState` `{x, y, theta, visible}`, `theta` = yaw of the pose quaternion. See [`src/visionState.ts`](src/visionState.ts). |

### Selection (`robot` + `team` variables)

Each converter is registered with `watchVariables`, so it is recreated when the
selection changes; the recreated converter closes over the selected source topic
name and drops every message whose topic doesn't match. Change `robot` (id,
default `0`) or, for vision, `team` (`blue`/`yellow`, default `blue`) in the
**Variables** tab and every panel re-points, no bag reload. `team` is a manual
variable — no referee introspection.

> Because `inputTopics` is a static list, Foxglove preloads all of a converter's
> declared inputs; the per-robot filtering happens inside the converter. (An
> earlier variant narrowed each converter to a single topic alias to avoid this,
> but alias re-pointing didn't reliably re-target the converters, so selection is
> done inside the converter instead.)

Only **global-frame** command modes are plotted; local-frame commands
(`BCM_LOCAL_*`) are intentionally not plotted (no local→global rotation). The
per-mode-colored trajectory curves still show the active mode. When a robot is not
`visible`, its vision `x/y/theta` are NaN so the curve gaps.

> *Topic* converters are used (not `type: "schema"` converters) because a schema
> converter's output fields don't resolve in the Plot panel here; a topic
> converter produces a genuine dedicated output topic the layout binds to.

Plots use each message's **receive (log) time** as the x-axis; there is no robot
time axis.

### Curve labels & colors

Every layout curve has a **label**. Convention: a `_telem` suffix means the curve
is sourced from `/analysis_telem` (the robot-side / round-tripped signal, or a
telemetry-only signal); no suffix means the software-side source
(`/analysis_control` for `cmd`, `/analysis_vision` for `vision`). Colors follow
**dark = software-side, light = robot-side**: software command `#0000ff` vs
round-tripped `#6666ff`; fresh vision mint `#3eb489` vs round-tripped cyan
`#00dac7`. By default the software-side curve of each pair is shown and the
round-tripped one is hidden.

## Build & install

Requires Node.js. From this directory, `npm install` first, then pick the target
matching your Foxglove:

**Web app (app.foxglove.dev):**

```bash
npm install
npm run package         # produces ateam.ateam-controls-analysis-<version>.foxe
```

Open app.foxglove.dev and **drag the `.foxe` onto the window** to install it,
then reload the page. (The web-app install is per-browser and does not persist
across cleared site data.)

**Desktop app:**

```bash
npm install
npm run local-install   # builds and installs into ~/.foxglove-studio/extensions/
```

`local-install` targets the desktop app only; it does **not** work for the
browser app. Restart Foxglove (or reload with the app menu → View → Reload)
after installing to pick up the extension either way.

## Usage

1. Open a recorded bag containing `/robot_feedback/extended/robot{id}`,
   `/robot_motion_commands/robot{id}`, and `/{blue,yellow}_team/robot{id}` (Open
   local file) or connect live via
   [`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge).
2. Import a layout from `../` — `controls_analysis.json` (tabbed) or
   `controls_analysis_singleview.json` (grid), or their `*_lines.json` variants
   that draw connecting lines instead of points-only. All bind the
   `/analysis_telem`, `/analysis_control`, and `/analysis_vision` topics.
3. Open the **Variables** tab (right sidebar in current Foxglove — toggle with
   the `]` key) and set `robot` to the id you want (e.g. `2`) and `team` to your
   color. All panels switch with no reload.

## Verifying / troubleshooting

- After installing, confirm Foxglove lists it under **Settings → Extensions**
  (or the Extensions sidebar) as "A-Team Controls Analysis".
- If plots are empty, check the **Topics** list for the raw source topics
  (`/robot_feedback/extended/robot{id}`, `/robot_motion_commands/robot{id}`,
  `/{team}_team/robot{id}`). The aliases and converters only work if those exist.
  `/analysis_telem` / `/analysis_control` / `/analysis_vision` are converter
  outputs — they won't appear in the raw source-topic list, but message paths
  referencing them resolve, and they appear once a panel subscribes.
- The extension only takes effect after a reload; a freshly installed extension
  won't apply to an already-open session until you reload.
