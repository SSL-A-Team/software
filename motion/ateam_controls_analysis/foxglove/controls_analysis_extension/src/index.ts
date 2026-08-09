// Copyright 2026 A Team
//
// Foxglove extension: controls analysis.
//
// This extension does all controls analysis inside Foxglove -- there is no ROS
// republisher node and no derived message type. It selects which robot (and
// team) you are viewing with topic aliases, and converts the selected topics into
// the flat, plot-friendly schemas the layouts bind to.
//
// Selection (topic aliases, driven by the `robot` and `team` global variables):
//   /robot_feedback/extended/robot{robot} -> /analysis_telem_src
//   /robot_motion_commands/robot{robot}   -> /analysis_control_src
//   /{team}_team/robot{robot}             -> /analysis_vision_src
//
// Conversion (one topic converter per stream, each with a SINGLE aliased input):
//   /analysis_telem_src   -> /analysis_telem    (ControlsAnalysis)
//   /analysis_control_src -> /analysis_control   (MotionCommand)
//   /analysis_vision_src  -> /analysis_vision    (VisionState)
//
// Because each converter's only input is an alias that resolves to ONE real
// source topic, Foxglove loads just the three selected topics -- there is no
// converter fan-in, so load time scales with one robot, not the fleet. The
// aliases carry the selection, so the converters need no `watchVariables` and no
// per-topic filtering; they just convert each message. Changing `robot`/`team` in
// the Variables tab re-points every panel with no bag reload.
//
// It operates at Foxglove's data-source layer, so it behaves identically for a
// loaded bag (full backfill) and a live connection.

import { ExtensionContext } from "@foxglove/extension";

import {
  buildControlsAnalysis,
  controlsAnalysisSchemaDescription,
} from "./controlsAnalysis";
import {
  buildMotionCommand,
  motionCommandSchemaDescription,
} from "./motionCommand";
import { buildVisionState, visionStateSchemaDescription } from "./visionState";

// Global variables that select what is displayed.
export const ROBOT_VARIABLE = "robot"; // robot id, default 0
export const TEAM_VARIABLE = "team"; // "blue" | "yellow", default "blue"
export const DEFAULT_TEAM = "blue";

// Raw per-robot source topic templates (the alias sources).
export const EXTENDED_TOPIC_TEMPLATE = "/robot_feedback/extended/robot{robot}";
export const COMMAND_TOPIC_TEMPLATE = "/robot_motion_commands/robot{robot}";
export const TEAM_TOPIC_TEMPLATE = "/{team}_team/robot{robot}";

// Alias names = the single input each converter reads.
export const TELEM_SRC_TOPIC = "/analysis_telem_src";
export const CONTROL_SRC_TOPIC = "/analysis_control_src";
export const VISION_SRC_TOPIC = "/analysis_vision_src";

// Converter output topics = what the layouts bind to.
export const TELEM_TOPIC = "/analysis_telem";
export const CONTROL_TOPIC = "/analysis_control";
export const VISION_TOPIC = "/analysis_vision";

// Converted schema names.
export const CONTROLS_ANALYSIS_SCHEMA = "ateam_controls_analysis/ControlsAnalysis";
export const MOTION_COMMAND_SCHEMA = "ateam_controls_analysis/MotionCommand";
export const VISION_STATE_SCHEMA = "ateam_controls_analysis/VisionState";

function robotIdFrom(value: unknown): number {
  const n =
    typeof value === "number"
      ? value
      : typeof value === "string"
        ? Number.parseInt(value, 10)
        : NaN;
  return Number.isFinite(n) && n >= 0 ? Math.trunc(n) : 0;
}

function teamFrom(value: unknown): string {
  return value === "yellow" || value === "blue" ? value : DEFAULT_TEAM;
}

export function activate(extensionContext: ExtensionContext): void {
  // Robot/team selection via topic aliases. The alias function re-runs whenever
  // the data-source topics or global variables change, so changing `robot`/`team`
  // re-points every converter's input with no bag reload.
  extensionContext.registerTopicAliases((args) => {
    const robot = String(robotIdFrom(args.globalVariables[ROBOT_VARIABLE]));
    const team = teamFrom(args.globalVariables[TEAM_VARIABLE]);
    return [
      {
        name: TELEM_SRC_TOPIC,
        sourceTopicName: EXTENDED_TOPIC_TEMPLATE.replace("{robot}", robot),
      },
      {
        name: CONTROL_SRC_TOPIC,
        sourceTopicName: COMMAND_TOPIC_TEMPLATE.replace("{robot}", robot),
      },
      {
        name: VISION_SRC_TOPIC,
        sourceTopicName: TEAM_TOPIC_TEMPLATE.replace("{team}", team).replace("{robot}", robot),
      },
    ];
  });

  // ExtendedTelemetry -> ControlsAnalysis (the main analysis).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [TELEM_SRC_TOPIC],
    outputTopic: TELEM_TOPIC,
    outputSchemaName: CONTROLS_ANALYSIS_SCHEMA,
    outputSchemaDescription: controlsAnalysisSchemaDescription(),
    create: () => (messageEvent) => buildControlsAnalysis(messageEvent.message),
  });

  // RobotMotionCommand -> MotionCommand (pre-send software command).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [CONTROL_SRC_TOPIC],
    outputTopic: CONTROL_TOPIC,
    outputSchemaName: MOTION_COMMAND_SCHEMA,
    outputSchemaDescription: motionCommandSchemaDescription(),
    create: () => (messageEvent) => buildMotionCommand(messageEvent.message),
  });

  // VisionStateRobot -> VisionState (fresh software-side vision estimate).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [VISION_SRC_TOPIC],
    outputTopic: VISION_TOPIC,
    outputSchemaName: VISION_STATE_SCHEMA,
    outputSchemaDescription: visionStateSchemaDescription(),
    create: () => (messageEvent) => buildVisionState(messageEvent.message),
  });
}
