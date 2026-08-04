# Changelog

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
