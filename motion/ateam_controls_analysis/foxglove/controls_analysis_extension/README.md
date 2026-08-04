# Controls Analysis (Foxglove extension)

A [Foxglove extension](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
that powers the bundled controls-analysis layouts. It does the analysis **inside
Foxglove** — there is no ROS republisher node and no derived message type. Point
Foxglove at a recorded `ateam_radio_msgs/ExtendedTelemetry` bag (or a live
connection) and the layouts just work.

## What it does

It registers a single **topic converter**, operating at Foxglove's data-source
layer (so it behaves identically for a loaded bag with full backfill and a live
connection). The converter reads the fleet's recorded
`ateam_radio_msgs/msg/ExtendedTelemetry` topics
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

**Robot selection** is driven by the `robot` global variable (default `0`). The
converter (registered with `watchVariables: ["robot"]`) only emits for the
selected robot's telemetry topic and drops the rest, so changing `robot` in
Foxglove's **Variables** tab instantly re-points every panel — no bag reload.

> A *topic* converter is used (rather than a schema converter plus a topic alias)
> precisely so the output is a genuine dedicated topic whose **only** schema is
> `ControlsAnalysis`. The layout paths (e.g. `.x.pos_cmd`) then resolve
> unambiguously, instead of competing with the raw `ExtendedTelemetry` schema on
> an aliased telemetry topic.

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
2. Import a layout from `../` (`controls_analysis.json` or
   `controls_analysis_singleview.json`) — both bind `/controls_analysis_selected`.
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
