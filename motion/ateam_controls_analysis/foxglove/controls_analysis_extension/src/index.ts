// Copyright 2026 A Team
//
// Foxglove extension: controls analysis.
//
// This extension does all controls analysis inside Foxglove -- there is no ROS
// republisher node and no derived message type. It registers two stateless
// *topic converters*:
//
//   1. the fleet's `ateam_radio_msgs/ExtendedTelemetry` topics -> one in-app
//      topic `/controls_analysis_selected`, the flat, per-mode-colored
//      `ControlsAnalysis` schema the bundled layouts plot against; and
//   2. the friendly team's `/{color}_team/robot{id}` vision-state topics -> one
//      in-app topic `/vision_state_selected`, the fresh software-side vision
//      estimate the position plots overlay (see below).
//
// Robot selection is driven by the `robot` global variable: each converter only
// emits for the currently selected robot's topic and drops the rest, so changing
// `robot` in Foxglove's Variables tab re-points every panel with no bag reload.
// A topic converter (rather than a schema converter + topic alias) is used so
// each output is a genuine dedicated topic whose *only* schema is the converted
// one -- the layout paths resolve unambiguously instead of competing with the raw
// input schema on an aliased topic.
//
// It operates at Foxglove's data-source layer, so it behaves identically for a
// loaded bag (full backfill) and a live connection.

import { ExtensionContext, MessageSchemaDescription } from "@foxglove/extension";

import {
  buildControlsAnalysis,
  controlsAnalysisSchemaDescription,
} from "./controlsAnalysis";
import {
  buildVisionState,
  friendlyColorFromReferee,
  parseTeamRobotTopic,
  TeamColor,
  TEAM_COLORS,
  visionStateSchemaDescription,
} from "./visionState";

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

// --- Vision-state overlay (fresh software-side vision estimate) ---------------
// The stable topic the position plots overlay for the fresh vision estimate.
export const VISION_STATE_SELECTED_TOPIC = "/vision_state_selected";
// Schema name of the flat vision-state message.
export const VISION_STATE_SCHEMA = "ateam_controls_analysis/VisionState";
// Referee topic used to auto-detect which team color is friendly (ours).
export const REFEREE_TOPIC = "/referee_messages";
// Per-team per-robot vision-state topic templates (converter inputs).
export const TEAM_TOPIC_TEMPLATE = "/{color}_team/robot{robot}";
// Global variable holding our team name, matched against the referee teams to
// auto-detect the friendly color. Defaults to the codebase default "A-Team".
export const TEAM_NAME_VARIABLE = "team_name";
export const DEFAULT_TEAM_NAME = "A-Team";
// Global variable overriding the friendly color: "auto" (default, detect from
// the referee message) or an explicit "blue" / "yellow".
export const FRIENDLY_TEAM_VARIABLE = "friendly_team";

function extendedTopic(robot: number): string {
  return EXTENDED_TOPIC_TEMPLATE.replace("{robot}", String(robot));
}

function teamTopic(color: TeamColor, robot: number): string {
  return TEAM_TOPIC_TEMPLATE.replace("{color}", color).replace("{robot}", String(robot));
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

  // Fresh software-side vision estimate for the selected robot, routed onto its
  // own topic (kept separate from ControlsAnalysis on purpose). The friendly team
  // color is auto-detected from the referee message (matching our team name), so
  // the correct `/{color}_team/robot{id}` is selected without the user picking a
  // color. theta is computed here (yaw from the pose quaternion) because message
  // paths can't derive it. This converter is stateful only in the minimal sense
  // that it remembers the last-seen friendly color across referee messages -- no
  // vision sample interacts with another.
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [
      REFEREE_TOPIC,
      ...TEAM_COLORS.flatMap((color) => ROBOT_IDS.map((id) => teamTopic(color, id))),
    ],
    outputTopic: VISION_STATE_SELECTED_TOPIC,
    outputSchemaName: VISION_STATE_SCHEMA,
    outputSchemaDescription: visionStateSchemaDescription(),
    watchVariables: [ROBOT_VARIABLE, TEAM_NAME_VARIABLE, FRIENDLY_TEAM_VARIABLE],
    create: (globalVariables) => {
      const selectedId = robotIdFrom(globalVariables[ROBOT_VARIABLE]);
      const teamName =
        typeof globalVariables[TEAM_NAME_VARIABLE] === "string"
          ? (globalVariables[TEAM_NAME_VARIABLE] as string)
          : DEFAULT_TEAM_NAME;
      const override = globalVariables[FRIENDLY_TEAM_VARIABLE];
      // "auto" (or unset) detects from the referee; an explicit color forces it.
      let friendly: TeamColor | undefined =
        override === "blue" || override === "yellow" ? override : undefined;
      const autoDetect = friendly == undefined;

      return (messageEvent) => {
        if (messageEvent.topic === REFEREE_TOPIC) {
          if (autoDetect) {
            const detected = friendlyColorFromReferee(messageEvent.message, teamName);
            if (detected != undefined) {
              friendly = detected;
            }
          }
          return undefined;
        }
        const parsed = parseTeamRobotTopic(messageEvent.topic);
        if (
          parsed == undefined ||
          parsed.id !== selectedId ||
          friendly == undefined ||
          parsed.color !== friendly
        ) {
          return undefined;
        }
        return buildVisionState(messageEvent.message);
      };
    },
  });
}
