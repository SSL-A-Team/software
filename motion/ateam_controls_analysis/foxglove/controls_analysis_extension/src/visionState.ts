// Copyright 2026 A Team
//
// Pure, stateless helpers to convert an `ateam_msgs/VisionStateRobot` (the
// software-side vision estimate published on `/{color}_team/robot{id}`) into the
// flat `{x, y, theta, visible}` shape the position plots overlay.
//
// This is the *fresh* vision estimate the software uses to make decisions. The
// cyan `pos_vision` curve on the same plots is the *same* measurement after it
// has round-tripped to the robot and come back in ExtendedTelemetry (delayed,
// with jitter); overlaying the two shows the round-trip delay.
//
// theta is derived here (yaw from the pose quaternion) because Foxglove message
// paths can't compute it, so a plain topic alias wouldn't expose a theta series.

import { MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

// The two team colors and the per-robot vision-state topic prefixes they map to
// (see ateam_common/topic_names.hpp: kYellowTeamRobotPrefix / kBlueTeamRobotPrefix).
export const TEAM_COLORS = ["blue", "yellow"] as const;
export type TeamColor = (typeof TEAM_COLORS)[number];

const TEAM_ROBOT_RE = /^\/(blue|yellow)_team\/robot(\d+)$/;

// Parse a per-robot vision-state topic name into its color and robot id, or
// undefined if it doesn't match the expected pattern.
export function parseTeamRobotTopic(
  topic: string,
): { color: TeamColor; id: number } | undefined {
  const m = TEAM_ROBOT_RE.exec(topic);
  return m ? { color: m[1] as TeamColor, id: Number.parseInt(m[2], 10) } : undefined;
}

// Yaw (rad) from a geometry_msgs/Quaternion, for the planar theta series.
function quaternionToYaw(q: any): number {
  if (q == undefined) {
    return NAN;
  }
  const x = Number(q.x);
  const y = Number(q.y);
  const z = Number(q.z);
  const w = Number(q.w);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

// Structure of the converted message, for message-path autocomplete.
export function visionStateSchemaDescription(): MessageSchemaDescription {
  return {
    x: "number",
    y: "number",
    theta: "number",
    visible: "bool",
  };
}

// Convert one VisionStateRobot into the flat overlay shape. When the robot is not
// visible, x/y/theta are NaN so the curve gaps rather than drawing a stale point.
export function buildVisionState(msg: any): Record<string, unknown> {
  const visible = Boolean(msg.visible);
  const pos = msg.pose?.position;
  if (!visible || pos == undefined) {
    return { x: NAN, y: NAN, theta: NAN, visible };
  }
  return {
    x: Number(pos.x),
    y: Number(pos.y),
    theta: quaternionToYaw(msg.pose?.orientation),
    visible,
  };
}

// Determine our (friendly) team color from a referee message by matching the
// configured team name against the two teams' names. Returns the matching color,
// or undefined if neither matches (leave the previously-detected color in place).
export function friendlyColorFromReferee(
  refereeMsg: any,
  teamName: string,
): TeamColor | undefined {
  if (refereeMsg?.blue?.name === teamName) {
    return "blue";
  }
  if (refereeMsg?.yellow?.name === teamName) {
    return "yellow";
  }
  return undefined;
}
