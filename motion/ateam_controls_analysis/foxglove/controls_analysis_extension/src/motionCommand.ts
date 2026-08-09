// Copyright 2026 A Team
//
// Pure, stateless helpers to convert an `ateam_msgs/RobotMotionCommand` (the
// software-side command published to `/robot_motion_commands/robot{id}`, before
// it is sent to the robot) into the flat per-derivative command shape the plots
// overlay.
//
// This is the *pre-send* software command. The `pos_cmd` / `vel_cmd` /
// `accel_cmd` curves sourced from `ControlsAnalysis` (i.e. the firmware
// `cmd_echo`) are the *same* command after a full round trip to the robot and
// back through telemetry (delayed, with jitter); overlaying the two shows the
// round-trip delay.
//
// The command is routed onto the derivative implied by the active body control
// mode, exactly like the ControlsAnalysis `cmd_echo` routing, and emitted with
// the same nested `{x,y,theta}.{pos_cmd,vel_cmd,accel_cmd}` field names so the
// layout paths line up. Only global-frame modes route a command; local-frame
// commands (`BCM_LOCAL_*`) are intentionally NOT plotted, matching the
// ControlsAnalysis side (this avoids a brittle local->global rotation, since the
// command message carries no robot heading). Global-frame modes therefore
// overlay the robot-side curve exactly.

import { MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

// Body control mode values (mirror RobotMotionCommand.msg; identical to the
// telemetry enum). Only the global-frame modes are used -- local-frame commands
// are intentionally not plotted.
const BCM_GLOBAL_POSITION = 10;
const BCM_GLOBAL_VELOCITY = 11;
const BCM_GLOBAL_ACCEL = 13;

const DIMS = ["x", "y", "theta"] as const;
const CMD_FIELDS = ["pos_cmd", "vel_cmd", "accel_cmd"] as const;

// mode -> [RobotMotionCommand Twist2D field to read, output command field].
// Global-frame modes only; local-frame modes (`BCM_LOCAL_*`) are absent, so their
// commands are not plotted (no local->global rotation).
const MODE_ROUTE: Record<number, [string, string]> = {
  [BCM_GLOBAL_POSITION]: ["pose", "pos_cmd"],
  [BCM_GLOBAL_VELOCITY]: ["velocity", "vel_cmd"],
  [BCM_GLOBAL_ACCEL]: ["acceleration", "accel_cmd"],
};

// Structure of the converted message, for message-path autocomplete. Mirrors the
// ControlsAnalysis command fields so the plots can share path suffixes.
export function motionCommandSchemaDescription(): MessageSchemaDescription {
  const dim: MessageSchemaDescription = {};
  for (const f of CMD_FIELDS) {
    dim[f] = "number";
  }
  const desc: MessageSchemaDescription = { body_control_mode: "number" };
  for (const d of DIMS) {
    desc[d] = dim;
  }
  return desc;
}

// Convert one RobotMotionCommand into the flat overlay shape. Every command field
// is NaN except the one the active mode routes to (and only for modes with a
// direct x/y/theta mapping; OFF, ESTOP, pivot and line modes leave all NaN).
export function buildMotionCommand(msg: any): Record<string, unknown> {
  const mode = Number(msg.body_control_mode);
  const out: Record<string, unknown> = { body_control_mode: mode };
  for (const d of DIMS) {
    out[d] = { pos_cmd: NAN, vel_cmd: NAN, accel_cmd: NAN };
  }
  const route = MODE_ROUTE[mode];
  if (route != undefined) {
    const [srcField, outField] = route;
    const src = msg[srcField];
    if (src != undefined) {
      for (const d of DIMS) {
        (out[d] as Record<string, number>)[outField] = Number(src[d]);
      }
    }
  }
  return out;
}
