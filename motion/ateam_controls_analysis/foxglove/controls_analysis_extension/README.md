# Controls Analysis (Foxglove extension)

A [Foxglove extension](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
that powers the bundled controls-analysis layouts. It does the analysis **inside
Foxglove** — there is no ROS republisher node and no derived message type. Point
Foxglove at a recorded `ateam_radio_msgs/ExtendedTelemetry` bag (or a live
connection) and the layouts just work.

## What it does

It **selects** the robot/team you're viewing with topic aliases, then **converts**
the three selected topics into the flat schemas the layouts plot. All of it runs
at Foxglove's data-source layer, so it behaves identically for a loaded bag (full
backfill) and a live connection.

### Selection — topic aliases (`robot` + `team` variables)

`registerTopicAliases` maps each selected raw per-robot topic onto a stable
"source" name:

| Alias | Raw source | Driven by |
|---|---|---|
| `/analysis_telem_src` | `/robot_feedback/extended/robot{robot}` | `robot` |
| `/analysis_control_src` | `/robot_motion_commands/robot{robot}` | `robot` |
| `/analysis_vision_src` | `/{team}_team/robot{robot}` | `robot`, `team` |

Change `robot` (id, default `0`) or `team` (`blue`/`yellow`, default `blue`) in
the **Variables** tab and every panel re-points instantly, no bag reload. `team`
is a manual variable — no referee introspection.

### Conversion — one topic converter per stream (single aliased input)

Each converter's **only** `inputTopic` is one of the aliases above, so Foxglove
loads just the three selected topics — there is **no converter fan-in**, and load
time scales with one robot, not the fleet. Because the alias already carries the
selection, the converters need no `watchVariables` and no per-topic filtering;
they just convert each message (stateless — one in, one out):

| Output topic (layouts bind here) | From | Schema / computation |
|---|---|---|
| `/analysis_telem` | `/analysis_telem_src` (`ExtendedTelemetry`) | `ControlsAnalysis`: per-dim pos/vel/accel, per-mode-colored trajectory splits, `cmd_echo` command (global modes only), per-wheel velocity + signed-mean current. See [`src/controlsAnalysis.ts`](src/controlsAnalysis.ts). |
| `/analysis_control` | `/analysis_control_src` (`RobotMotionCommand`) | `MotionCommand`: the pre-send software command routed per body control mode (global modes only). See [`src/motionCommand.ts`](src/motionCommand.ts). |
| `/analysis_vision` | `/analysis_vision_src` (`VisionStateRobot`) | `VisionState` `{x, y, theta, visible}`, `theta` = yaw of the pose quaternion. See [`src/visionState.ts`](src/visionState.ts). |

Only **global-frame** command modes are plotted; local-frame commands
(`BCM_LOCAL_*`) are intentionally not plotted (no local→global rotation). The
per-mode-colored trajectory curves still show the active mode. When a robot is not
`visible`, its vision `x/y/theta` are NaN so the curve gaps.

> *Topic* converters are used (not `type: "schema"` converters) because a schema
> converter's output fields don't resolve in the Plot panel here; a topic
> converter produces a genuine dedicated output topic the layout binds to. Feeding
> it the selection alias as its lone input keeps the fan-in at one topic.

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
