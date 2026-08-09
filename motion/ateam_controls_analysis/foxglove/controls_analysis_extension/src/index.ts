// Copyright 2026 A Team
//
// Foxglove extension: controls analysis.
//
// This extension does the *minimum* work in Foxglove: it selects the robot (and
// team) you're viewing and computes only the two values a plot message path
// cannot. Everything else -- per-mode trajectory coloring, command gating,
// software-vs-robot overlays -- is done directly in the layouts with raw message
// fields and `{body_control_mode==N}` path filters.
//
// It registers:
//
//   1. Topic aliases (driven by the `robot` and `team` global variables) that map
//      each raw per-robot source topic onto a stable "selected" topic the layouts
//      bind to:
//        /robot_feedback/extended/robot{robot}  -> /analysis_telem_selected
//        /robot_motion_commands/robot{robot}    -> /analysis_command_selected
//        /{team}_team/robot{robot}              -> /analysis_vision_selected
//      Because an alias maps a stable name onto ONE real source topic, only the
//      selected robot's data is ever loaded -- there is no converter fan-in, so
//      load time scales with one robot, not the whole fleet.
//
//   2. Two *topic* converters for the only values a message path can't express,
//      each fed by the selection alias as its single input (so no fan-in):
//        /analysis_vision_selected  -> /analysis_vision_theta_selected
//            (heading = yaw of the pose quaternion, plus x/y/visible)
//        /analysis_telem_selected   -> /analysis_wheelcurrent_selected
//            (per-wheel signed mean of the current-sample buffer)
//      A topic converter (not a schema converter) is used because schema-
//      converter output fields do not resolve in the Plot panel here, whereas a
//      topic converter produces a genuine dedicated output topic the layout binds
//      to. Its single input is the alias, which resolves to one real source
//      topic, so it adds no fan-in.
//
// Both operate at Foxglove's data-source layer, so they behave identically for a
// loaded bag (full backfill) and a live connection.

import { ExtensionContext } from "@foxglove/extension";

import {
  VISION_TO_SCHEMA,
  visionConverter,
  visionSchemaDescription,
} from "./visionState";
import {
  WHEEL_CURRENT_TO_SCHEMA,
  wheelCurrentConverter,
  wheelCurrentSchemaDescription,
} from "./wheelCurrent";

// Global variables that select what is displayed.
export const ROBOT_VARIABLE = "robot"; // robot id, default 0
export const TEAM_VARIABLE = "team"; // "blue" | "yellow", default "blue"
export const DEFAULT_TEAM = "blue";

// Stable "selected" topics the layouts bind to. The three *_selected topics are
// aliases onto raw source topics; the two computed topics are topic-converter
// outputs fed by those aliases.
export const TELEM_SELECTED_TOPIC = "/analysis_telem_selected";
export const COMMAND_SELECTED_TOPIC = "/analysis_command_selected";
export const VISION_SELECTED_TOPIC = "/analysis_vision_selected";
export const VISION_THETA_TOPIC = "/analysis_vision_theta_selected";
export const WHEEL_CURRENT_TOPIC = "/analysis_wheelcurrent_selected";

// Raw per-robot source topic templates.
export const EXTENDED_TOPIC_TEMPLATE = "/robot_feedback/extended/robot{robot}";
export const COMMAND_TOPIC_TEMPLATE = "/robot_motion_commands/robot{robot}";
export const TEAM_TOPIC_TEMPLATE = "/{team}_team/robot{robot}";

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
  // in the Variables tab re-points every panel with no bag reload.
  extensionContext.registerTopicAliases((args) => {
    const robot = String(robotIdFrom(args.globalVariables[ROBOT_VARIABLE]));
    const team = teamFrom(args.globalVariables[TEAM_VARIABLE]);
    return [
      {
        name: TELEM_SELECTED_TOPIC,
        sourceTopicName: EXTENDED_TOPIC_TEMPLATE.replace("{robot}", robot),
      },
      {
        name: COMMAND_SELECTED_TOPIC,
        sourceTopicName: COMMAND_TOPIC_TEMPLATE.replace("{robot}", robot),
      },
      {
        name: VISION_SELECTED_TOPIC,
        sourceTopicName: TEAM_TOPIC_TEMPLATE.replace("{team}", team).replace("{robot}", robot),
      },
    ];
  });

  // Vision heading (yaw) + x/y/visible for the fresh-vision overlay. Topic
  // converter fed by the vision alias (one input -> no fan-in).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [VISION_SELECTED_TOPIC],
    outputTopic: VISION_THETA_TOPIC,
    outputSchemaName: VISION_TO_SCHEMA,
    outputSchemaDescription: visionSchemaDescription(),
    create: () => visionConverter,
  });

  // Per-wheel signed mean current for the current plots. Topic converter fed by
  // the telemetry alias (one input -> no fan-in).
  extensionContext.registerMessageConverter({
    type: "topic",
    inputTopics: [TELEM_SELECTED_TOPIC],
    outputTopic: WHEEL_CURRENT_TOPIC,
    outputSchemaName: WHEEL_CURRENT_TO_SCHEMA,
    outputSchemaDescription: wheelCurrentSchemaDescription(),
    create: () => wheelCurrentConverter,
  });
}
