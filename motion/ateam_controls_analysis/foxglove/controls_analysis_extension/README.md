# Controls Analysis (Foxglove extension)

A [Foxglove extension](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
that powers the bundled controls-analysis layouts. It does the analysis **inside
Foxglove** — there is no ROS republisher node and no derived message type. Point
Foxglove at a recorded `ateam_radio_msgs/ExtendedTelemetry` bag (or a live
connection) and the layouts just work.

## What it does

It registers two **topic converters**, both operating at Foxglove's data-source
layer (so they behave identically for a loaded bag with full backfill and a live
connection), and both selecting the robot via the `robot` global variable
(default `0`, `watchVariables: ["robot"]`) — change `robot` in the **Variables**
tab and every panel re-points instantly, no bag reload.

### 1. Controls analysis — `/controls_analysis_selected`

Reads the fleet's recorded `ateam_radio_msgs/msg/ExtendedTelemetry` topics
(`/robot_feedback/extended/robot0..15`) and produces one new in-app topic,
**`/controls_analysis_selected`**, carrying the flat
`ateam_controls_analysis/ControlsAnalysis` schema the layouts bind to.

For each telemetry message it flattens the sample into per-dimension (x / y /
theta) position / velocity / acceleration series and per-wheel velocity / current
series. The reference trajectory and the software command are additionally
emitted as **per-mode split series** (each non-NaN only while its body control
mode is active), so each mode is drawn in its own color and the reference/command
curves **change color as the active control mode changes**. The conversion is
**stateless**: one telemetry message maps to exactly one output message — no
message is ever compared against a previous one (no robot-time reconstruction, no
reboot tracking, no mode-transition gaps, no heartbeat). See
[`src/controlsAnalysis.ts`](src/controlsAnalysis.ts).

### 2. Fresh vision estimate — `/vision_state_selected`

Reads the friendly team's per-robot vision-state topics
(`/{color}_team/robot{id}`, `ateam_msgs/VisionStateRobot`) and produces one new
in-app topic, **`/vision_state_selected`**, with a flat `{x, y, theta, visible}`
schema. This is the *fresh* software-side vision estimate — the pose the software
uses to make decisions. The position plots overlay it (mint, off by default)
against the cyan `pos_vision` curve, which is the *same* measurement after it has
round-tripped to the robot and returned in `ExtendedTelemetry`; the offset
between them is the round-trip delay. `theta` is the yaw of the pose quaternion,
computed here because message paths can't derive it. See
[`src/visionState.ts`](src/visionState.ts).

The **friendly** color is auto-detected from `/referee_messages` by matching our
team name against the two teams (the same logic the stack uses at runtime), so
the correct `/{color}_team/robot{id}` is chosen without picking a color. Two
optional variables tune this:

| Variable | Default | Meaning |
|----------|---------|---------|
| `friendly_team` | `auto` | `auto` detects from the referee; `blue`/`yellow` forces it (e.g. bags without `/referee_messages`). |
| `team_name` | `A-Team` | Our team name, matched against the referee teams when `friendly_team` is `auto`. |

This converter's only state is the last-seen friendly color across referee
messages; no vision sample interacts with another. When the robot is not
`visible`, its `x/y/theta` are NaN so the curve gaps.

> *Topic* converters are used (rather than schema converters plus topic aliases)
> so each output is a genuine dedicated topic whose **only** schema is the
> converted one. The layout paths (e.g. `.x.pos_cmd`) then resolve unambiguously,
> instead of competing with the raw input schema on an aliased topic. (A topic
> alias also can't target a converter output or expose the computed `theta`.)

Plots use each message's **receive (log) time** as the x-axis; there is no robot
time axis.

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

1. Open a recorded `ExtendedTelemetry` bag (Open local file) or connect live via
   [`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge).
2. Import a layout from `../` — `controls_analysis.json` (tabbed) or
   `controls_analysis_singleview.json` (grid), or their `*_lines.json` variants
   that draw connecting lines instead of points-only. All bind
   `/controls_analysis_selected`.
3. Open the **Variables** tab (right sidebar in current Foxglove — toggle with
   the `]` key) and set `robot` to the id you want (e.g. `2`). All panels switch
   to that robot with no reload.

## Verifying / troubleshooting

- After installing, confirm Foxglove lists it under **Settings → Extensions**
  (or the Extensions sidebar) as "A-Team Controls Analysis".
- If plots are empty, check the **Topics** list for
  `/robot_feedback/extended/robot{id}` entries in your data source. The converter
  only works if those source telemetry topics exist.
  `/controls_analysis_selected` is a converter output topic — it won't appear in
  the raw source-topic list, but message paths referencing it will resolve, and
  it appears once a panel subscribes.
- The extension only takes effect after a reload; a freshly installed extension
  won't apply to an already-open session until you reload.
