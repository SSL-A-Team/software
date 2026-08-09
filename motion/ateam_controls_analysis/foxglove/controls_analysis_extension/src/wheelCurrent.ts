// Copyright 2026 A Team
//
// Pure, stateless helper to compute per-wheel measured current for the current
// plots. `ExtendedTelemetry` reports each motor's current as a per-cycle buffer
// of unsigned sample magnitudes (`current_samples_ma`); the plots want a single
// signed value per cycle, which a Foxglove message path can't compute. A *topic*
// converter (fed by the telemetry selection alias) reduces the buffer to its
// mean and applies the sign of the commanded setpoint to recover direction, for
// all four motors.

import { MessageEvent, MessageSchemaDescription } from "@foxglove/extension";

const NAN = Number.NaN;

// Source telemetry schema (unused for topic converters, kept for reference) and
// the converted wheel-current schema name.
export const TELEM_FROM_SCHEMA = "ateam_radio_msgs/msg/ExtendedTelemetry";
export const WHEEL_CURRENT_TO_SCHEMA = "ateam_controls_analysis/WheelCurrentAnalysis";

// Structure of the converted message, for message-path autocomplete.
export function wheelCurrentSchemaDescription(): MessageSchemaDescription {
  const wheel: MessageSchemaDescription = { current: "number", current_setpoint: "number" };
  return {
    front_left: wheel,
    back_left: wheel,
    back_right: wheel,
    front_right: wheel,
  };
}

// Wheel name -> the ExtendedTelemetry CcmTelemetry motor field it comes from.
const WHEEL_MOTOR_ATTR: Record<string, string> = {
  front_left: "front_left_motor",
  back_left: "back_left_motor",
  back_right: "back_right_motor",
  front_right: "front_right_motor",
};

// Mean measured current (mA) for one CcmTelemetry motor. The samples are unsigned
// magnitudes; an empty buffer yields NaN (gap), and the sign of the (signed)
// commanded setpoint is applied to recover direction.
export function wheelCurrent(ccm: any): { current: number; current_setpoint: number } {
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
  return { current, current_setpoint: setpoint };
}

// Convert one ExtendedTelemetry into `{front_left, back_left, back_right,
// front_right}`, each `{current, current_setpoint}`.
export function buildWheelCurrent(msg: any): Record<string, unknown> {
  const out: Record<string, unknown> = {};
  for (const wheel of Object.keys(WHEEL_MOTOR_ATTR)) {
    out[wheel] = wheelCurrent(msg[WHEEL_MOTOR_ATTR[wheel]]);
  }
  return out;
}

// Topic-converter entry point: (messageEvent) => converted message.
export function wheelCurrentConverter(event: MessageEvent): Record<string, unknown> {
  return buildWheelCurrent(event.message);
}
