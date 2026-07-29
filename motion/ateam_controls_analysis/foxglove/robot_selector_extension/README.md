# Controls Analysis Robot Selector (Foxglove extension)

A tiny [Foxglove extension](https://docs.foxglove.dev/docs/visualization/extensions/introduction)
that lets the bundled controls-analysis layouts switch which robot they display
**without reloading the bag**.

## What it does

The converter and node publish one topic per robot,
`/controls_analysis/robot{id}`. Foxglove message paths can use the `$robot`
global variable in field/index/filter positions but **not** in the topic-name
position, so a variable alone can't retarget a per-robot topic.

This extension registers a **topic alias**: it maps
`/controls_analysis/robot{robot}` (where `{robot}` is the `robot` global
variable, default `0`) onto a single stable topic `/controls_analysis_selected`.
The layouts subscribe to `/controls_analysis_selected`, so changing the `robot`
variable in Foxglove's **Variables** tab instantly re-points every panel.

Because a topic alias works at Foxglove's data-source layer, it applies to a
loaded bag exactly like a live connection — a file source has full backfill, so
the newly selected robot's plots immediately show the whole timeline.

## Build & install

Requires Node.js. From this directory:

```bash
npm install
npm run local-install   # builds and installs into your local Foxglove
```

Restart Foxglove (or reload) to pick up the extension. To produce a shareable
`.foxe` package instead, run `npm run package`.

## Usage

1. Open a combined converted bag (`<bag>_controls_analysis`) or connect live.
2. Import a layout from `../` (`controls_analysis.json` or
   `controls_analysis_singleview.json`) — both bind `/controls_analysis_selected`.
3. In the **Variables** tab, set `robot` to the id you want (e.g. `2`).
   All panels switch to that robot with no reload.
