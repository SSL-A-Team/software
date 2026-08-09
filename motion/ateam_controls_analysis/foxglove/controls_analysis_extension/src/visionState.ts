// Copyright 2026 A Team
//
// Pure, stateless helper to compute the friendly robot's vision heading. The
// software-side vision estimate is published as `ateam_msgs/VisionStateRobot`
// with a quaternion orientation; the controls plots want a planar heading in
// radians, which a Foxglove message path can't derive. A *topic* converter (fed
// by the vision selection alias) adds it (yaw), alongside position/visible, so
// the plots can overlay the fresh vision estimate against the round-tripped one
// from telemetry.

import { MessageEvent, MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

// Source vision-state topic schema (unused for topic converters, kept for
// reference) and the converted analysis schema name.
export const VISION_FROM_SCHEMA = "ateam_msgs/msg/VisionStateRobot";
export const VISION_TO_SCHEMA = "ateam_controls_analysis/VisionStateAnalysis";

// Structure of the converted message, for message-path autocomplete.
export function visionSchemaDescription(): MessageSchemaDescription {
  return { x: "number", y: "number", theta: "number", visible: "bool" };
}

// Yaw (rad) from a geometry_msgs/Quaternion.
export function quaternionToYaw(q: any): number {
  if (q == undefined) {
    return NAN;
  }
  const x = Number(q.x);
  const y = Number(q.y);
  const z = Number(q.z);
  const w = Number(q.w);
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

// Convert one VisionStateRobot into `{x, y, theta, visible}`. When the robot is
// not visible, x/y/theta are NaN so the curve gaps rather than drawing a stale
// point.
export function buildVisionAnalysis(msg: any): Record<string, unknown> {
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

// Topic-converter entry point: (messageEvent) => converted message.
export function visionConverter(event: MessageEvent): Record<string, unknown> {
  return buildVisionAnalysis(event.message);
}
