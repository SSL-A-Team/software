# Changelog

## 3.0.0

- **Selection inside the converters (no aliases).** Three topic converters read
  every robot's raw topics and emit only the selected robot's converted message
  onto `/analysis_telem`, `/analysis_control`, `/analysis_vision`. Selection is by
  the `robot` global variable (plus `team` for vision); each converter uses
  `watchVariables` and filters `inputTopics` to the selected source topic.
  (An interim variant used topic aliases as single converter inputs to avoid the
  input fan-in, but alias re-pointing didn't reliably re-target the converters, so
  selection was moved back inside the converters.)
- **Renamed the layout-facing output topics** to `/analysis_telem`,
  `/analysis_control`, `/analysis_vision`.
- **Re-added the pre-send software command** stream (`/analysis_control`,
  `RobotMotionCommand`), plotted against the round-tripped `cmd_echo`.
- **Removed the local→global command rotation** and the dead
  `vel_cmd_local_vel` / `accel_cmd_local_acc` split fields. Only global-frame
  command modes are plotted; local modes show via the trajectory color only.
- **Removed friendly-team auto-detection** (no more `/referee_messages`
  introspection, no `friendly_team` / `team_name` variables). Team is now a manual
  `team` variable (`blue`/`yellow`, default `blue`).
- **Color convention** made explicit: dark = software-side, light = robot-side
  (software cmd `#0000ff` / round-tripped `#6666ff`; fresh vision `#3eb489` /
  round-tripped `#00dac7`). Software-side shown, robot-side hidden by default.
- **Added a label to every layout curve** (`_telem` suffix = from `/analysis_telem`;
  no suffix = software side).

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
