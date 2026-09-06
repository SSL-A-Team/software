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

// Reconstruct the robot's own microsecond clock -- split across two uint32s in
// the telemetry (`timestamp_us_lo` / `timestamp_us_hi`) -- as float seconds.
// This lets a Plot panel use robot time as its X axis (X-Axis = "message path"
// -> `/analysis_telem.robot_time_s`) instead of PC receive time, so samples land
// on the robot's own timeline regardless of radio latency/jitter. The value is
// monotonic within a boot and resets to ~0 on reboot. `hi * 2^32 + lo` is exact
// in float64 far beyond any match runtime.
function robotTimeSeconds(msg: any): number {
  const lo = Number(msg.timestamp_us_lo);
  const hi = Number(msg.timestamp_us_hi);
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
    return NAN;
  }
  return (hi * 4294967296 + lo) / 1e6;
}

// Same robot clock as `robotTimeSeconds`, split into a ROS-style `{sec, nanosec}`
// stamp (with a `nsec` alias for ROS1-style readers) so the converted message
// carries a `header.stamp`. A Plot panel can then stay in the normal Timestamp
// X-axis mode with `timestampMethod: "headerStamp"` -- which keeps `isSynced`
// (cross-panel X sync + shared hover cursor) working -- while plotting against
// robot time. Missing/invalid timestamps map to 0. Resets to ~0 on reboot.
function secNanosFromSeconds(t: number): { sec: number; nanosec: number; nsec: number } {
  if (!Number.isFinite(t)) {
    return { sec: 0, nanosec: 0, nsec: 0 };
  }
  const sec = Math.floor(t);
  const nsec = Math.round((t - sec) * 1e9);
  return { sec, nanosec: nsec, nsec };
}

// Raw robot-boot-relative header stamp (seconds since boot). Used as a fallback
// when no PC receive time / clock anchor is available.
function robotHeaderStamp(msg: any): { sec: number; nanosec: number; nsec: number } {
  return secNanosFromSeconds(robotTimeSeconds(msg));
}

// Anchor that maps the robot's boot-relative clock onto the PC (Unix) timeline.
// The offset (pc_time - robot_time) is captured once from the first valid packet
// and then held, so the emitted `header.stamp` lands on the same absolute
// timescale as the data source's timeline. That is what lets a Timestamp-mode
// Plot with `timestampMethod: headerStamp` line its X axis (and the synced hover
// cursor) up with the global timeline, instead of sitting ~10^9 s away at
// seconds-since-boot. After the one-time anchor the stamp advances on the robot's
// own (jitter-free) clock, not PC receive time, so radio latency jitter on later
// packets doesn't move the X axis. One converter instance owns one anchor and
// re-anchors when recreated (e.g. the selected robot changes). A reboot (robot
// clock jumps back to ~0) shows as a jump on the X axis.
export type RobotClockAnchor = { offsetSec?: number };

function syncedHeaderStamp(
  msg: any,
  receiveTime: { sec: number; nsec: number },
  anchor: RobotClockAnchor,
): { sec: number; nanosec: number; nsec: number } {
  const robotSec = robotTimeSeconds(msg);
  if (!Number.isFinite(robotSec)) {
    return { sec: 0, nanosec: 0, nsec: 0 };
  }
  const pcSec = receiveTime.sec + receiveTime.nsec * 1e-9;
  if (anchor.offsetSec == undefined) {
    anchor.offsetSec = pcSec - robotSec;
  }
  return secNanosFromSeconds(robotSec + anchor.offsetSec);
}

// Every value field in a DimensionAnalysis, so paths always resolve (to NaN
// when inactive) rather than reading `undefined`.
function dimensionFieldNames(): string[] {
  const names = [
    "pos_estimate", "pos_ekf", "pos_traj", "pos_vision", "pos_cmd",
    "vel_estimate", "vel_ekf", "vel_traj", "vel_cmd", "vel_gyro",
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
    robot_time_s: "number",
    header: { stamp: { sec: "number", nanosec: "number", nsec: "number" } },
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
// `receiveTime` + `anchor` (from the converter) enable the PC-anchored robot-time
// `header.stamp` (see `syncedHeaderStamp`); when omitted, the header falls back to
// the raw robot-boot-relative stamp.
export function buildControlsAnalysis(
  msg: any,
  receiveTime?: { sec: number; nsec: number },
  anchor?: RobotClockAnchor,
): Record<string, unknown> {
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

  // `vision_pose` only carries a fresh measurement on cycles where the firmware
  // applied a vision update; otherwise it's stale, so gate the curve on the bit.
  const visionUpdated = Boolean(bct.vision_update);

  const stamp =
    receiveTime != undefined && anchor != undefined
      ? syncedHeaderStamp(msg, receiveTime, anchor)
      : robotHeaderStamp(msg);

  const out: Record<string, unknown> = {
    body_control_mode: mode,
    theta_estimate: thetaEst,
    robot_time_s: robotTimeSeconds(msg),
    header: { stamp },
  };

  DIMS.forEach((d, i) => {
    const f: Record<string, number> = {};
    for (const name of DIM_FIELDS) {
      f[name] = NAN;
    }

    f["pos_estimate"] = at(bct.kf_body_pos_estimate, i);
    // EKF body position state (delivered on the `kf_body_pos_prediction` field).
    f["pos_ekf"] = at(bct.kf_body_pos_prediction, i);
    f["pos_traj"] = at(bct.body_traj_pos, i);
    f["pos_vision"] = visionUpdated ? at(bct.vision_pose, i) : NAN;
    f["vel_estimate"] = at(bct.kf_body_vel_estimate, i);
    // EKF body velocity state (delivered on the `kf_body_vel_prediction` field).
    f["vel_ekf"] = at(bct.kf_body_vel_prediction, i);
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
