# Changelog

## 3.0.0

- **Rearchitected to aliases + layout filters.** Replaced the three stateful
  topic converters (which forced Foxglove to preload every robot's telemetry,
  both teams' vision, and every command topic — the cause of slow bag loads) with
  lightweight **topic aliases**:
  `/robot_feedback/extended/robot{robot}` → `/analysis_telem_selected`,
  `/robot_motion_commands/robot{robot}` → `/analysis_command_selected`,
  `/{team}_team/robot{robot}` → `/analysis_vision_selected`, driven by the `robot`
  and `team` global variables. Only the selected robot's topics load, so load time
  scales with one robot, not the fleet.
- All per-mode trajectory coloring and command gating now live in the **layouts**
  as `{body_control_mode==N}` message-path filters over raw fields — no converter.
- Kept only the two values a message path can't compute, as **topic converters**
  whose single input is the selection alias (so no fan-in): vision heading
  (`/analysis_vision_selected` → `/analysis_vision_theta_selected`, `theta` =
  quaternion yaw) and per-wheel signed current mean (`/analysis_telem_selected` →
  `/analysis_wheelcurrent_selected`). Topic converters are used rather than
  `type: "schema"` converters because schema-converter output fields don't resolve
  in the Plot panel (verified empty even on the un-aliased source topic); a topic
  converter produces a real output topic the layout binds to. Vision x/y and wheel
  current setpoint stay **raw** fields off the aliases (no converter needed).
- Removed the `ControlsAnalysis`, `VisionState`, and `MotionCommand` topic
  outputs and their `controlsAnalysis.ts` / `motionCommand.ts` modules.
- Team is now a manual `team` variable (`blue`/`yellow`); no more referee-message
  introspection.

## 2.3.0

- Removed the local→global rotation for commands (brittle w.r.t. future changes).
  Both the round-tripped `cmd_echo` (ControlsAnalysis) and the pre-send command
  now plot **global-frame modes only**; local-frame commands (`BCM_LOCAL_*`) are
  not plotted. The per-mode-colored reference-trajectory curves (which still
  include local velocity) indicate the active mode.

## 2.2.0

- Added a third topic converter for the **pre-send software command**: the
  fleet's `/robot_motion_commands/robot{id}` (`ateam_msgs/RobotMotionCommand`) →
  `/robot_motion_command_selected` (nested `{x,y,theta}.{pos_cmd,vel_cmd,accel_cmd}`),
  selected by the `robot` global variable. Routed onto the derivative implied by
  the body control mode (global-frame modes only; see 2.3.0).
- Position / velocity / acceleration plots now overlay each command **twice**:
  the pre-send software command (dark blue `#0000ff`) and the round-tripped
  `cmd_echo` (light blue `#6666ff`), to show the round-trip delay — mirroring the
  vision pair (mint `#3eb489` fresh vs. cyan `#00dac7` round-tripped).
- Layout default changes: the pre-send/software-side curve of each duplicate pair
  (mint vision, dark-blue command) is shown, and the round-tripped/robot-side
  curve (cyan vision, light-blue command) is hidden. Command curves draw as
  points only in every layout, including the `*_lines` variants.

## 2.1.0

- Added a second topic converter for the **fresh vision estimate**: the friendly
  team's `/{color}_team/robot{id}` (`ateam_msgs/VisionStateRobot`) →
  `/vision_state_selected` (`{x, y, theta, visible}`), selected by the same
  `robot` global variable. The friendly color is auto-detected from
  `/referee_messages` (team-name match), overridable via the `friendly_team`
  (`auto`/`blue`/`yellow`) and `team_name` variables. `theta` is the yaw of the
  pose quaternion, computed in the converter. The position plots overlay it in
  mint (off by default) against the delayed cyan `pos_vision` curve to read the
  round-trip delay to the robot.

## 2.0.0

- Moved the controls analysis entirely into this extension. It registers a
  stateless **topic converter** that reads the fleet's
  `ateam_radio_msgs/ExtendedTelemetry` topics and produces one dedicated in-app
  topic, `/controls_analysis_selected`, carrying the flat
  `ateam_controls_analysis/ControlsAnalysis` schema the layouts plot
  (per-derivative state / trajectory / command / measurement series, with the
  reference-trajectory and command curves split per active body control mode so
  they change color as the control mode changes).
- Robot selection is driven by the `robot` global variable (`watchVariables`):
  the converter emits only the selected robot's telemetry and drops the rest, so
  switching robots needs no bag reload. Using a topic converter (instead of a
  schema converter + topic alias) makes the output a genuine dedicated topic
  whose only schema is `ControlsAnalysis`, so the layout paths resolve
  unambiguously.
- Removed the ROS republisher node, the offline bag converter, and the derived
  message package; no conversion happens in ROS any more.
- Dropped robot-time reconstruction, reboot-count tracking, and the heartbeat.
  Conversion is stateless: one telemetry message maps to one output message.

## 1.0.0

- Initial release. Aliased `/controls_analysis/robot{robot}` (from the `robot`
  global variable) onto `/controls_analysis_selected` so the controls-analysis
  layouts could switch robots without reloading the bag.
