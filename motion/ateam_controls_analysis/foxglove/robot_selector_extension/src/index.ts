// Copyright 2026 A Team
//
// Foxglove extension: controls-analysis robot selector.
//
// Aliases the per-robot ControlsAnalysis topic selected by the `robot` global
// variable onto a single stable topic, `/controls_analysis_selected`. The
// bundled layouts bind to that alias, so changing the `robot` variable in
// Foxglove's Variables tab switches which robot every panel displays -- with no
// bag reload. Because a topic alias operates at Foxglove's data-source layer, it
// works identically for a loaded bag (full backfill) and a live connection.

import { ExtensionContext } from "@foxglove/extension";

// The stable topic every layout subscribes to.
export const SELECTED_TOPIC = "/controls_analysis_selected";
// Foxglove global variable that selects the robot id (defaults to 0).
export const ROBOT_VARIABLE = "robot";

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
  extensionContext.registerTopicAliases((args) => {
    const robot = robotIdFrom(args.globalVariables[ROBOT_VARIABLE]);
    return [
      {
        name: SELECTED_TOPIC,
        sourceTopicName: `/controls_analysis/robot${robot}`,
      },
    ];
  });
}
