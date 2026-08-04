// Copyright 2026 A Team
//
// Foxglove extension: controls analysis.
//
// This extension does all controls analysis inside Foxglove -- there is no ROS
// republisher node and no derived message type. It registers a single stateless
// *topic converter* that reads the fleet's recorded
// `ateam_radio_msgs/ExtendedTelemetry` topics and produces one new in-app topic,
// `/controls_analysis_selected`, carrying the flat, per-mode-colored
// `ControlsAnalysis` schema the bundled layouts plot against.
//
// Robot selection is driven by the `robot` global variable: the converter only
// emits for the currently selected robot's telemetry topic and drops the rest,
// so changing `robot` in Foxglove's Variables tab re-points every panel with no
// bag reload. A topic converter (rather than a schema converter + topic alias)
// is used so the output is a genuine dedicated topic whose *only* schema is
// ControlsAnalysis -- the layout paths (e.g. `.x.pos_cmd`) resolve unambiguously
// instead of competing with the raw ExtendedTelemetry schema on an aliased topic.
//
// It operates at Foxglove's data-source layer, so it behaves identically for a
// loaded bag (full backfill) and a live connection.

import { ExtensionContext, MessageSchemaDescription } from "@foxglove/extension";

import {
  buildControlsAnalysis,
  controlsAnalysisSchemaDescription,
} from "./controlsAnalysis";

// The stable topic every layout subscribes to (the converter's output).
export const SELECTED_TOPIC = "/controls_analysis_selected";
// Schema name of the flat converted message the layouts plot against.
export const CONTROLS_ANALYSIS_SCHEMA = "ateam_controls_analysis/ControlsAnalysis";
// Template for the per-robot extended-telemetry topics (converter inputs).
export const EXTENDED_TOPIC_TEMPLATE = "/robot_feedback/extended/robot{robot}";
// Foxglove global variable that selects the robot id (defaults to 0).
export const ROBOT_VARIABLE = "robot";
// Robot ids the converter listens to. The fleet is 0..15; input topics that a
// given data source doesn't contain are simply never delivered.
export const ROBOT_IDS = Array.from({ length: 16 }, (_, i) => i);

function extendedTopic(robot: number): string {
  return EXTENDED_TOPIC_TEMPLATE.replace("{robot}", String(robot));
}

function robotIdFrom(value: unknown): number {
  const n =
    typeof value === "number"
      ? value
      : typeof value === "string"
        ? Number.parseInt(value, 10)
        : NaN;
  return Number.isFinite(n) && n >= 0 ? Math.trunc(n) : 0;
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: ROBOT_IDS.map(extendedTopic),
    outputTopic: SELECTED_TOPIC,
    outputSchemaName: CONTROLS_ANALYSIS_SCHEMA,
    outputSchemaDescription: controlsAnalysisSchemaDescription(),
    // Recreate the converter when the selected robot changes so the closure
    // below filters to the new robot's telemetry topic.
    watchVariables: [ROBOT_VARIABLE],
    create: (globalVariables) => {
      const selected = extendedTopic(robotIdFrom(globalVariables[ROBOT_VARIABLE]));
      return (messageEvent) => {
        // Stateless per-message conversion; emit only the selected robot.
        if (messageEvent.topic !== selected) {
          return undefined;
        }
        return buildControlsAnalysis(messageEvent.message);
      };
    },
  });
}
