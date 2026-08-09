// Copyright 2026 A Team
//
// Pure, stateless helper to convert an `ateam_msgs/VisionStateRobot` (the
// software-side vision estimate published on `/{team}_team/robot{id}`) into the
// flat `{x, y, theta, visible}` shape the position plots overlay.
//
// This is the *fresh* vision estimate the software uses to make decisions. The
// cyan `pos_vision` curve on the same plots is the *same* measurement after it
// has round-tripped to the robot and come back in ExtendedTelemetry (delayed,
// with jitter); overlaying the two shows the round-trip delay.
//
// theta is derived here (yaw from the pose quaternion) because Foxglove message
// paths can't compute it. The converter takes a single input (the vision
// selection alias, driven by the `robot`/`team` variables), so it neither parses
// the topic name nor introspects the referee -- it just converts each message.

import { MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

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
