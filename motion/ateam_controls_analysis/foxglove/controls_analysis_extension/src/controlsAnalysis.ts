// Copyright 2026 A Team
//
// Pure, stateless conversion from `ateam_radio_msgs/ExtendedTelemetry` into the
// flat, plot-friendly `ControlsAnalysis` shape the controls-analysis layouts
// bind to. This is the single place that knows how to read ExtendedTelemetry
// fields and route them onto the correct derivative/color, and it runs entirely
// inside the Foxglove extension -- there is no ROS republisher node or derived
// message type any more.
//
// Design constraints:
//   * Stateless: one telemetry message maps to one output message; a message is
//     never compared against a previous one (no robot-time reconstruction, no
//     reboot tracking, no mode-transition gap logic, no heartbeat).
//   * NaN marks "no data at this instant", which Foxglove renders as a gap.
//   * The software command (`cmd_echo`) is routed onto the derivative implied by
//     the active body control mode. Only global-frame modes route a command;
//     local-frame commands (`BCM_LOCAL_*`) are intentionally NOT plotted (this
//     avoids a brittle local->global rotation). The per-mode-colored trajectory
//     curves still indicate the active mode.
//   * The reference trajectory and the software command are additionally emitted
//     as per-mode split series (non-NaN only while that mode is active) so each
//     body control mode is drawn in its own color, changing color as the active
//     control mode changes.

import { MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

// Body control mode values (mirror BodyControlExtendedTelemetry.msg).
export const BCM_GLOBAL_POSITION = 10;
export const BCM_GLOBAL_VELOCITY = 11;
export const BCM_LOCAL_VELOCITY = 12;
export const BCM_GLOBAL_ACCEL = 13;
export const BCM_LOCAL_ACCEL = 14;
export const BCM_HEADING_PIVOT = 20;
export const BCM_POINT_PIVOT = 21;
export const BCM_HEADING_LINE = 30;
export const BCM_POINT_LINE = 31;

// Dimensions (in telemetry float-array index order) and wheel motors (in
// CcmTelemetry order: front-left, back-left, back-right, front-right).
const DIMS = ["x", "y", "theta"] as const;
const WHEELS = ["front_left", "back_left", "back_right", "front_right"] as const;
const WHEEL_MOTOR_ATTR: Record<string, string> = {
  front_left: "front_left_motor",
  back_left: "back_left_motor",
  back_right: "back_right_motor",
  front_right: "front_right_motor",
};

// Short mode names used to build per-mode split-series field names.
const MODE_SUFFIX: Record<number, string> = {
  [BCM_GLOBAL_POSITION]: "global_pos",
  [BCM_GLOBAL_VELOCITY]: "global_vel",
  [BCM_LOCAL_VELOCITY]: "local_vel",
  [BCM_GLOBAL_ACCEL]: "global_acc",
  [BCM_LOCAL_ACCEL]: "local_acc",
  [BCM_HEADING_PIVOT]: "heading_pivot",
  [BCM_POINT_PIVOT]: "point_pivot",
  [BCM_HEADING_LINE]: "heading_line",
  [BCM_POINT_LINE]: "point_line",
};

// Modes for which the firmware computes trajectory setpoints (bang-bang); used
// to color the reference-trajectory curves on the position/velocity plots.
const TRAJ_MODES = [
  BCM_GLOBAL_POSITION,
  BCM_GLOBAL_VELOCITY,
  BCM_LOCAL_VELOCITY,
  BCM_HEADING_PIVOT,
  BCM_POINT_PIVOT,
  BCM_HEADING_LINE,
  BCM_POINT_LINE,
];

// mode -> derivative name for the software command. Only global-frame modes are
// listed: local-frame commands (`BCM_LOCAL_*`) are intentionally not plotted, so
// there is no local->global rotation. The per-mode-colored trajectory curves
// still show the active mode.
const CMD_ROUTE: Record<number, string> = {
  [BCM_GLOBAL_POSITION]: "pos",
  [BCM_GLOBAL_VELOCITY]: "vel",
  [BCM_GLOBAL_ACCEL]: "accel",
};

type Vec = ArrayLike<number> | undefined;

function at(a: Vec, i: number): number {
  return a != undefined && i < a.length ? Number(a[i]) : NAN;
}

// Every value field in a DimensionAnalysis, so paths always resolve (to NaN
// when inactive) rather than reading `undefined`.
function dimensionFieldNames(): string[] {
  const names = [
    "pos_estimate", "pos_traj", "pos_vision", "pos_cmd",
    "vel_estimate", "vel_traj", "vel_cmd", "vel_gyro",
    "accel_u", "accel_u_fric_comp", "accel_imu", "accel_cmd",
  ];
  for (const deriv of ["pos", "vel"]) {
    for (const m of TRAJ_MODES) {
      names.push(`${deriv}_traj_${MODE_SUFFIX[m]}`);
    }
  }
  names.push(
    "pos_cmd_global_pos",
    "vel_cmd_global_vel",
    "accel_cmd_global_acc",
  );
  return names;
}

const DIM_FIELDS = dimensionFieldNames();

// Pull the active maneuver's echoed command as [a, b, c], or undefined for modes
// that are not plotted (OFF, ESTOP, the local-frame modes, pivot and line). Only
// global-frame modes have a command routed onto a derivative.
function extractCmdNative(bct: any): [number, number, number] | undefined {
  switch (bct.body_control_mode) {
    case BCM_GLOBAL_POSITION: {
      const e = bct.maneuver_global_pos.cmd_echo;
      return [e.global_x, e.global_y, e.global_theta];
    }
    case BCM_GLOBAL_VELOCITY: {
      const e = bct.maneuver_global_vel.cmd_echo;
      return [e.global_xd, e.global_yd, e.global_omega];
    }
    case BCM_GLOBAL_ACCEL: {
      const e = bct.maneuver_global_acc.cmd_echo;
      return [e.global_xdd, e.global_ydd, e.global_alpha];
    }
    default:
      return undefined;
  }
}

// Build one WheelAnalysis from a CcmTelemetry motor. The measured current is the
// mean of the per-cycle current-sample buffer (unsigned magnitudes); an empty
// buffer yields NaN so the curve gaps rather than reading a spurious 0, and the
// sign of the (signed) commanded setpoint is applied to recover direction.
function wheelFromCcm(ccm: any): Record<string, number> {
  const setpoint = Number(ccm.current_telemetry.current_setpoint_ma);
  const samples: ArrayLike<number> = ccm.current_telemetry.current_samples_ma ?? [];
  const n = samples.length;
  let current = NAN;
  if (n > 0) {
    let sum = 0;
    for (let i = 0; i < n; i++) {
      sum += Number(samples[i]);
    }
    current = sum / n;
    if (setpoint < 0) {
      current = -current;
    }
  }
  return {
    vel: Number(ccm.velocity_telemetry.wheel_vel_rads),
    vel_setpoint: Number(ccm.velocity_telemetry.vel_setpoint_rads),
    current,
    current_setpoint: setpoint,
  };
}

// Structure of the converted message, so Foxglove can offer message-path
// autocomplete on the output topic. Every value field is a plain number; the
// shape mirrors what buildControlsAnalysis() returns.
export function controlsAnalysisSchemaDescription(): MessageSchemaDescription {
  const dim: MessageSchemaDescription = {};
  for (const name of DIM_FIELDS) {
    dim[name] = "number";
  }
  const wheel: MessageSchemaDescription = {
    vel: "number",
    vel_setpoint: "number",
    current: "number",
    current_setpoint: "number",
  };
  const desc: MessageSchemaDescription = {
    body_control_mode: "number",
    theta_estimate: "number",
  };
  for (const d of DIMS) {
    desc[d] = dim;
  }
  for (const w of WHEELS) {
    desc[w] = wheel;
  }
  return desc;
}

// Convert one ExtendedTelemetry message into the flat ControlsAnalysis object.
export function buildControlsAnalysis(msg: any): Record<string, unknown> {
  const bct = msg.body_control_telemetry;
  const mode: number = bct.body_control_mode;
  const thetaEst = at(bct.kf_body_pos_estimate, 2);

  // Resolve the software command (global-frame modes only; shared across dims).
  const cmdDeriv = CMD_ROUTE[mode];
  const cmdNative = extractCmdNative(bct);
  let cmd: [number, number, number] | undefined;
  if (cmdDeriv != undefined && cmdNative != undefined) {
    cmd = [Number(cmdNative[0]), Number(cmdNative[1]), Number(cmdNative[2])];
  }
  const trajActive = TRAJ_MODES.includes(mode);
  const suffix = MODE_SUFFIX[mode];

  const out: Record<string, unknown> = {
    body_control_mode: mode,
    theta_estimate: thetaEst,
  };

  DIMS.forEach((d, i) => {
    const f: Record<string, number> = {};
    for (const name of DIM_FIELDS) {
      f[name] = NAN;
    }

    f["pos_estimate"] = at(bct.kf_body_pos_estimate, i);
    f["pos_traj"] = at(bct.body_traj_pos, i);
    f["pos_vision"] = at(bct.vision_pose, i);
    f["vel_estimate"] = at(bct.kf_body_vel_estimate, i);
    f["vel_traj"] = at(bct.body_traj_vel, i);
    f["accel_u"] = at(bct.body_accel_u, i);
    f["accel_u_fric_comp"] = at(bct.body_accel_u_fric_comp, i);

    // Measurements on their physically correct derivative / dimension only.
    if (d === "theta") {
      f["vel_gyro"] = at(bct.imu_gyro, 2);
    } else {
      f["accel_imu"] = at(bct.imu_accel, i);
    }

    // Per-mode reference-trajectory split series (color-by-mode).
    if (trajActive && suffix != undefined) {
      f[`pos_traj_${suffix}`] = at(bct.body_traj_pos, i);
      f[`vel_traj_${suffix}`] = at(bct.body_traj_vel, i);
    }

    // Software command: single-series + per-mode split series (global modes only).
    if (cmd != undefined && cmdDeriv != undefined && suffix != undefined) {
      f[`${cmdDeriv}_cmd`] = cmd[i];
      f[`${cmdDeriv}_cmd_${suffix}`] = cmd[i];
    }

    out[d] = f;
  });

  for (const w of WHEELS) {
    out[w] = wheelFromCcm(msg[WHEEL_MOTOR_ATTR[w]]);
  }

  return out;
}
