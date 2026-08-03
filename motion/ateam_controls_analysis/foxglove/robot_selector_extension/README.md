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

`local-install` targets the **Foxglove desktop app** only (it copies into
`~/.foxglove-studio/extensions/`); it does **not** work for the browser app at
app.foxglove.dev. For the web app, run `npm run package` to produce a `.foxe`
and drag it onto the window, or install the desktop app.

Restart Foxglove (or reload with the app menu → View → Reload) after installing
to pick up the extension.

## Usage

1. Open a combined converted bag (`<bag>_controls_analysis`) or connect live.
2. Import a layout from `../` (`controls_analysis.json` or
   `controls_analysis_singleview.json`) — both bind `/controls_analysis_selected`.
3. Open the **Variables** tab (right sidebar in current Foxglove — toggle with
   the `]` key) and set `robot` to the id you want (e.g. `2`). All panels switch
   to that robot with no reload.

## Verifying / troubleshooting

- After installing, confirm Foxglove lists it under **Settings → Extensions**
  (or the Extensions sidebar) as "A-Team Controls Analysis Robot Selector".
- If plots are empty, check the **Topics** list for `/controls_analysis/robot{id}`
  entries in your data source. The alias only works if those source topics
  exist. `/controls_analysis_selected` is a virtual alias — it won't appear in
  the raw topic list, but message paths referencing it will resolve.
- The extension only takes effect after a reload; a freshly installed extension
  won't apply to an already-open session until you reload.

