# Controls Analysis (Foxglove extension)

A [Foxglove extension](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
that powers the bundled controls-analysis layouts. It does the **minimum** work
in Foxglove: it selects which robot (and team) you're viewing, and computes only
the two values a plot message path can't. Everything else — per-mode trajectory
coloring, command gating, software-vs-robot overlays — is done directly in the
layouts with raw message fields and `{body_control_mode==N}` path filters.

## What it does

### 1. Robot/team selection — topic aliases

The extension registers **topic aliases** (driven by the `robot` and `team`
global variables) that map each raw per-robot source topic onto a stable
"selected" topic the layouts bind to:

| Selected topic (layouts use these) | Source (raw) | Driven by |
|---|---|---|
| `/analysis_telem_selected` | `/robot_feedback/extended/robot{robot}` | `robot` |
| `/analysis_command_selected` | `/robot_motion_commands/robot{robot}` | `robot` |
| `/analysis_vision_selected` | `/{team}_team/robot{robot}` | `robot`, `team` |

Because an alias maps a stable name onto **one real source topic**, only the
selected robot's data is ever loaded — there is **no converter fan-in**, so load
time scales with one robot, not the whole fleet. Change `robot` (id, default `0`)
or `team` (`blue`/`yellow`, default `blue`) in the **Variables** tab and every
panel re-points instantly, no bag reload. (Team is a manual variable — no referee
introspection.)

### 2. The only two computed values — topic converters (fed by aliases)

A Foxglove message path can read fields and filter, but can't compute. Two values
need real computation, so the extension adds them with **topic converters** whose
single `inputTopic` is the selection **alias** — so they inherit the "one robot"
scope with no fan-in:

- `/analysis_vision_selected` → **`/analysis_vision_theta_selected`**
  `{x, y, theta, visible}` — the fresh vision **heading** is `theta` = yaw of the
  pose quaternion. See [`src/visionState.ts`](src/visionState.ts).
- `/analysis_telem_selected` → **`/analysis_wheelcurrent_selected`**
  per-wheel `{current, current_setpoint}` — `current` is the signed mean of the
  per-cycle `current_samples_ma` buffer (unsigned magnitudes × sign of the
  setpoint). See [`src/wheelCurrent.ts`](src/wheelCurrent.ts).

> **Why topic converters, not schema converters?** A `type: "schema"` converter's
> output fields do **not** resolve in the Plot panel in this setup (verified: the
> converted field was empty even on the original, un-aliased topic). A
> `type: "topic"` converter instead produces a genuine dedicated output topic the
> layout binds to directly, which does resolve. Feeding it the alias as its lone
> input keeps the fan-in at one topic, so load time is unaffected.

### Everything else is layout path filters (no code)

The layouts read raw fields off the aliased topics and use `{body_control_mode==N}`
filters for the routing that used to live in a converter:

- **Per-mode trajectory coloring:** one filtered copy of `body_traj_pos[i]` /
  `body_traj_vel[i]` per mode, each its own color — e.g.
  `…body_control_telemetry{body_control_mode==10}.body_traj_pos[0]` (green).
- **Command gapping:** the software command
  (`/analysis_command_selected{body_control_mode==11}.velocity.x`) and the
  round-tripped `cmd_echo`
  (`…{body_control_mode==11}.maneuver_global_vel.cmd_echo.global_xd`) only draw
  during their mode. Only global-frame modes are plotted (no local→global
  rotation). The mode filter also sidesteps the `cmd_echo` union aliasing when a
  maneuver is inactive.

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

1. Open a recorded bag containing `/robot_feedback/extended/robot{id}`,
   `/robot_motion_commands/robot{id}`, and `/{blue,yellow}_team/robot{id}` (Open
   local file) or connect live via
   [`foxglove_bridge`](https://github.com/foxglove/ros-foxglove-bridge).
2. Import a layout from `../` — `controls_analysis.json` (tabbed) or
   `controls_analysis_singleview.json` (grid), or their `*_lines.json` variants
   that draw connecting lines instead of points-only. All bind the
   `/analysis_*_selected` topics.
3. Open the **Variables** tab (right sidebar in current Foxglove — toggle with
   the `]` key) and set `robot` to the id you want (e.g. `2`) and `team` to your
   color. All panels switch with no reload.

## Verifying / troubleshooting

- After installing, confirm Foxglove lists it under **Settings → Extensions**
  (or the Extensions sidebar) as "A-Team Controls Analysis".
- If plots are empty, check the **Topics** list for the raw source topics
  (`/robot_feedback/extended/robot{id}`, `/robot_motion_commands/robot{id}`,
  `/{team}_team/robot{id}`). The aliases only work if those topics exist. The
  `/analysis_*_selected` names are aliases — they won't appear in the raw
  source-topic list, but message paths referencing them resolve.
- If the mint vision **heading** (`theta`) or the wheel **current** curves are
  empty but other curves work, the topic converters aren't resolving — confirm
  the extension is installed and reloaded. (Position x/y and current setpoint are
  raw fields and don't depend on the converters.)
- The extension only takes effect after a reload; a freshly installed extension
  won't apply to an already-open session until you reload.
