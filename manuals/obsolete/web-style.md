# amr_web style makeover — coding plan

Prepared 2026-09-17 against branch `slam-roadmap` @ `55d400a`.

**Status (2026-09-17): W1–W8 implemented on top of `dac7a0a`, offline-verified only.**
Deviations from the text below:
- The operator callout class is `.warn-box` (and `.warn-box.stop`), not `.warn`: `warn` is
  already the level modifier on `.tel`, `.chip` and `.hero-mode`, and one class name for
  both would restyle every warning tile as a callout.
- The Maps jog pad sits in the survey panel, under the controls, not under the live map.
- W8 got two extra tests: `test_every_element_id_a_page_script_uses_exists_on_that_page`
  (the id-rename risk in §6) and `test_jogpad_js.py`, which runs the real `jogpad.js` in
  node against a fake DOM with deferred press responses (skipped when node is absent).
- Still open: §5 items 2–8. These are the sim walk-through at three sizes, the forced
  stale states, the jog regression in a real browser, the editor flow, the offline font
  check, the glyph check (`■ ▲ ◀ ↖` fall back to a system font) and P7.
Target: the unified-service web app `amr_ws/src/amr_web` (port 5001).
Style spec: [design_language.json](design_language.json).
Reference implementation of that spec: the legacy app, `app/static/app.css` and
`app/templates/base.html`. It already follows the spec and was reviewed for contrast
(see the `--ink-3` comment there).

## 1. Scope

**In scope:** everything in `amr_web/templates/` and `amr_web/static/`, plus the
`package_data` line in `setup.py` and one test file.

**Out of scope, do not change:**

- The API: any route in `server.py`, the `adapter.py` state payload, and any message.
- Jog safety behaviour in `jogpad.js`: session, ticket, refresh period, and the
  release on blur/pagehide/hidden/pointercancel/focused field. Only its markup and
  one presentational class toggle change (W4).
- MapView coordinate maths in `mapview.js`: `worldToPixel`, `pixelToWorld`, pan/zoom.
- The legacy app `app/`. It is retired in U11; copy from it, never import from it.
- New data sources. The spec's Wi-Fi indicator, battery tile and lidar zone lamps have
  no data in `/api/state` today. Leave them out; do **not** draw placeholders that look
  live. They can come later as their own change with an adapter subscription.
- Two-step "armed" confirmation on danger buttons. That changes behaviour; Abort on
  the Run page must stay one press. Only the danger *look* is applied.

## 2. Gap audit (current `amr.css` vs spec)

| Spec | Current amr_web | Fix |
|---|---|---|
| Light only, paper `#EFF1F5`, semantic colour only | Dark theme (`--bg:#14171c`), blue/green/orange hardcoded | New token set copied from `app.css` `:root` |
| Archivo / Plex Sans / Plex Mono, self-hosted | `system-ui`, `ui-monospace` | Copy `app/static/fonts/*` (8 woff2 + LICENSE.md) into `amr_web/static/fonts/` |
| Square corners, no shadows | `border-radius` 4/8/12px on cards, pills, inputs, buttons | Radius 0 everywhere except `.lamp` (50%) and `code` (2px) |
| Header 2px ink rule, uppercase mono nav, one status pill right | 1px rule, sans nav with underline, **six** pills | One link pill; the mode, selector, drives, localisation and run pills move into the rail as tiles |
| Fixed-height shell: header + 300px rail + main; only the log scrolls | Document scrolls; log in a footer | `.shell` grid, `#telemetry` rail, log well in the rail |
| Telemetry tiles: label / big mono value / unit | Multi-line `pre-wrap` strings in `.state` divs | Render tiles / definition rows (W5) |
| Status = wash + 3–4px left bar + text label | Filled pills (`.pill.ok` dark green fill) | `.tel.warn/.bad`, `.chip`, `.al-row.lv-*` |
| Stale must look stale, never calm | Only the header pills say "stale" | `.stale` dimming on tiles/lamps, and a red `bad` state for panel/drives older than 0.5 s |
| Tables: white box, rule-soft hairlines, mono key column | Generic `table` with dark borders | `.list` / `.row` grid pattern (from `.pm-list`, `.io-list`, `.al-log`) |
| Lamps: DI = accent-2, DO = hazard | I/O shows `ON`/`off` pills | `.lamp` / `.lamp.do` |
| Jog pad: 3×3 square cells, glyph + NAME + key, pressed = accent-2 fill | 64px dark buttons with glyphs only, no pressed state | W4 |
| Canvas drawn with UI variables | `#0d0f12` ground, `#3d8bfd`/`#2ecc71`/`#f0ad4e`/`#fff` overlays, 12px system-ui text | W6 |
| Voice: short uppercase mono labels | Sentence-case nav ("Maps & survey", "Route editor") | W2 nav labels |

## 3. Decisions

1. **Fonts are copied into `amr_web`, not shared.** The package has to stand on its
   own once `app/` is removed. Keep the "why self-hosted" comment. `setup.py` must
   become `["templates/*.html", "static/*.css", "static/*.js", "static/fonts/*"]`:
   `static/*` does not pick up a subdirectory in a non-symlink install.
2. **`amr.css` is rewritten, not patched.** Take the token block, font faces, base,
   header, pill, `button.big`, `button.danger`, `.tel*`, `.chip`, `.warn`, `.lamp`,
   `.io-*`, `.al-*`, `.pm-*`, `.pad`/`button.dir`, `kbd`, `#log` and the 820px media
   query from `app.css`, with their comments. Drop legacy-only blocks (`.track`,
   `.lcp`, `.rfid-*`, `.blind-*`, `.panel-strip`, `.auto-*`). Add the canvas-page layout.
   Target is under 450 lines.
3. **Map pages get the full main column.** The 900px max-width is for stacked
   sections. `editor`, `run` and `maps` set `<main class="wide">` (no max-width) so the
   canvas keeps its space. At 1280px wide that is 300 rail + 320 tools + ~620 canvas.
4. **Class vocabulary is `ok` / `warn` / `bad`** (the spec's names). Rename every
   `err` in the templates and JS (`log(..., 'err')`, `MODE_CLS`, pill classes).
   `log()` ignores its class today; keep that signature.
5. **Buttons map by meaning, not by the current class name:**
   - `primary` → `button.big.run` (steel blue): Save map revision, Save revision,
     Use this map on the vehicle, Hold plan, Load (READY), Confirm: scans align.
   - `danger` → `button.danger` (quiet outline, red on hover and focus): Abort survey,
     Abort, Clear steps, Clear / abort, Recover (FAULT → IDLE).
   - Everything else → `button.big`, or a compact `button.tool` for the editor turn
     grid and Undo/Redo (same look, padding 7px 10px, 11px).
   - Editor tool toggles (`.tool.on`) use the active nav look: accent-wash ground,
     accent-2 border and text.
   - Nothing is `go` (green fill). Nothing in this app arms the drives.
6. **Safety sentences become callouts where they are instructions to the operator.**
   `.warn` (hazard bar): the Run page's "Motion starts only from the physical panel",
   the commissioning "runs only on a fresh press of Start" text, and the Manual page's
   "Arming is the physical panel's job". Explanatory text stays `.note`
   (13px sans, ink-2, 74ch). Wording is kept; key words stay bold.
7. **Status text becomes structured, but every value in the old strings stays
   visible.** The old `.state` strings were the only diagnostics on some pages;
   dropping a field is a regression.
8. **Do the makeover before the U10 vehicle session.** K1 spot-checks the web UI and
   keyboard focus loss on the accepted build, and a restyle after acceptance would
   move the accepted build. JS changes are presentational, so the sim P7 test plus
   the jog checks in §5 are enough evidence to proceed.

## 4. Work breakdown

Each item ends with the app still working. Commit points are marked; the user makes
the commits.

### W1 — Assets and tokens

- Copy `app/static/fonts/` → `amr_ws/src/amr_web/amr_web/static/fonts/`.
- Fix `package_data` in `setup.py` (§3.1).
- New `amr.css`: font faces, `:root` tokens (verbatim from `app.css`, including the
  `--ink-3` contrast comment), base `html, body` (paper, 15.5px/1.62 Plex Sans,
  `touch-action: manipulation`), `h1`–`h3` plus the `small` eyebrow, `code`, `kbd`,
  `.note`, `.warn` (plus a `.warn.stop` variant with the stop-red bar), buttons (§3.5),
  form fields (`input`, `select`, `textarea`: 13px mono, white, 1px rule, square,
  2px accent-2 focus outline), disabled opacity .32, and user-select off only on
  `button, nav a, .pad, .lamp, .pill, kbd`.
- No `prefers-color-scheme`, `box-shadow`, gradient, or transition longer than 80ms.

Acceptance: pages load light, in the right fonts, with no requests outside the host
(DevTools network tab, offline).

### W2 — Shell: header, rail, log (`base.html`, `amr.js`)

Header: `<h1>AMR control</h1>`, then the nav, then one `#pill-link` pushed right.
Nav labels: `Status · Manual · Maps · Routes · Run · Monitor · I/O · Alarms · Params ·
Commission`. CSS uppercases them. The `href`s and `page` keys are unchanged. Ten tabs
do not fit at 1280px in 11.5px mono with 14px padding: use `padding: 6px 10px` and
let `header` wrap (`flex-wrap: wrap` is already in the reference).

Rail `<aside id="telemetry">`, with tiles in a 2-column grid:

| Tile id | Label | Value | Unit/note line | Level |
|---|---|---|---|---|
| `tel-mode` | MODE | `mode_name` | `phase` or `—` | IDLE/MAPPING/NAVIGATION plain; STARTING/TRANSITIONING warn; FAULT/STOPPING bad; no supervisor → `–` + bad |
| `tel-selector` | SELECTOR | AUTO / MANUAL | `age_s` in s | stale > 0.5 s or invalid → value `STALE`/`INVALID`, bad |
| `tel-drives` | DRIVES | ARMED / OFF | `left_state · right_state` | not operational → warn; stale → bad |
| `tel-source` | COMMAND | mux source name | `inhibited` or `—` | inhibited → warn |
| `tel-loc` | LOCALISATION | `state_name` or `–` | `stale` when `localization_stale` | READY plain, CHECKING warn, LOST bad, stale dims the tile |
| `tel-run` | RUN | `state_name` or `–` | `step_id` | BLOCKED/PAUSED warn, FAULT bad |
| `tel-wheels` | WHEELS | `left_rad_s / right_rad_s` (2 dp) | `rad/s · L / R` | none; stale mux dims |
| `tel-gen` | GENERATION | `mode.generation` | `lease` present / `no lease` | none |

Below the tiles is `#log` as the well that fills the rest of the rail.

`amr.js`: replace `pill()` for everything except the link pill with
`tile(id, value, note, level, stale)`. It sets `b`/`i` text and the `warn`/`bad`/`stale`
classes. Keep `request`, `api`, `apiGet`, `log`, `onState`, `lastState`, `poll`,
`operation`, `followOperation` and their behaviour exactly. `followOperation`'s
in-progress phase write goes to `tile('tel-mode', …, 'warn')`. A failed poll also sets
the whole rail `.stale` so disconnected never looks like calm.

Log: newest first as today, 60 lines. Add a mono time column (already in the string).

Acceptance: every page shows the same rail. When the web server is killed, the pill
turns bad and the rail dims within 1 s.

### W3 — Stacked pages: Status, Alarms, I/O, Monitor, Params

Main uses a max-width of 900px, and sections are `h2` + eyebrow + content.

- **Status:** a hero summary panel (spec `hero_summary_panel`) with the mode name at
  `clamp(30px,4vw,58px)` Archivo and the reason under it. Then a 3-column overview:
  active map `id rev N`, last saved survey, operation id. `FAULT code: reason` is a
  `.warn.stop` callout. A readiness grid of tiles (base ready, panel, drives, command
  source, localisation, manual available, autonomous available); booleans are `YES`/`NO`,
  and a `NO` on base ready is bad. Then the button bar: Survey a new map, Use a saved map
  (both links styled as `button.big`, and replace the `<a><button>` nesting with
  `<a class="big">`), Return to idle, Recover (danger). Then the existing note.
- **Alarms:** "Standing now" uses `.al-row` rows (level label + text, bar colour by
  level; the panel-stale and FAULT lines are `error`, inhibited/drives-off are `warn`).
  If none, `.mon-none` shows "nothing standing". Events are `.al-log` with `al-row` grid
  `time · LEVEL · source · code · text`. Keep the 200-row cap and the `since` cursor.
- **I/O:** two `.io-list` panels. DI rows are `ch · lamp · name`. DO rows are
  `ch · requested lamp · readback lamp · name`, with a header row reading
  `REQ` / `READ` in 10px mono, and `.lamp.do` on both. When `!comms_ok` or
  `age_s > 1.0`, the panels get `.stale` and a `.warn.stop` banner shows "COMMS LOST —
  lamps are last known". Link counters go in a tile row (image age, scan age, scans,
  errors, writes).
- **Monitor:** `.mon-grid` with one `.mon-node` per diagnostic name. The `h3` is the
  name, and a chip carries the level (`OK`/`WARN`/`ERROR`; `STALE` = error chip when
  `age_s > 3`). `hardware_id · N.N s ago` is the eyebrow. Values are `.pm-row`-style
  rows, key mono left and value right; `not available` stays in ink-3.
- **Params:** `.pm-id` header (profile name in Archivo accent, path in ink-3). The
  filter is `input[type=search]` in `.al-filters`. Runtime and profile sections are
  `.pm-list` rows: key mono / value with unit `i` / note as `<details class="pm-note">`
  when present.

### W4 — Jog pad (`jogpad.js`, CSS)

Markup: nine `button.dir` cells, each `<span class="glyph">`, `<span class="name">`
(`FWD-L FWD FWD-R / LEFT STOP RIGHT / REV-L REV REV-R`) and `<span class="key">` (`↑` …,
`Space` on stop). The stop cell is `button.dir.stopbtn`. Keep the `data-dir` values.
Speed select plus a status line as a `.tel` tile (`JOG` / `holding f` / `v · w`). The
explanatory `<p>` becomes a `.legend.keys` with `kbd`.

Behaviour addition, presentational only: `press()` adds `.active` to the held
direction's button, and `release()` removes it from all. Stop gets `.active` for 150 ms
on press. Nothing else in the file changes. Diff-review this file line by line.

Sizes: `.pad` max-width 520px on Manual. `.pad.compact` for the Maps and Run side
columns uses a fixed 56px row height, 22px glyph, and hides `.key`.

### W5 — Canvas pages: Maps, Routes (editor), Run

Layout: `main.wide`, with `.editor` = `grid-template-columns: 320px minmax(0,1fr)` and
`height: 100%` inside the fixed shell. That replaces `calc(100vh - 190px)`, which assumed
the old header + footer. The tools aside is a white panel (1px rule, padding 14px,
`overflow:auto`). Sections inside it are separated by 1px rule-soft instead of `<hr>`,
with `h2` + eyebrow. The canvas wrap is a white surface with a 1px rule and no radius.
The hint overlay is 11px mono ink-3 on a white strip with a 1px rule.

- **Maps:** the Survey session panel shows `state_name` as a key tile and `map_id rev`
  as a tile; `message` is a note. Closure is three tiles, `DX m / DY m / DYAW °`. They are
  warn when the value is available but unreviewed; that is presentational, and the
  threshold is not changed. Buttons: Start / Returned `big`, Save `run`, Abort `danger`.
  The saved revisions list is `.pm-list`-style rows per map with an `h3` map id; the
  bundle sha is in `code`. A per-revision error row is `lv-error`.
- **Routes:** keep the `id`s the JS reads (`ed-map`, `ed-route-id`, `ed-load`,
  `tool-start`, `tool-line`, `.turn`, `btn-*`, `ed-repeat`, `ed-speed`, `ed-result`,
  `ed-steps`, `ed-canvas`, `ed-hint`). The turn buttons go in a 4×2 grid of `button.tool`.
  In `ed-result`, `✓ valid` becomes an ok chip, each issue becomes an `al-row lv-error`,
  and the compiled summary becomes tiles (LENGTH m, TURNS °, CLOSES yes/no). `ed-steps`
  becomes `.list` rows `id · kind · detail`, with a bad row as `lv-error`.
- **Run:** the active map is a key tile (`ACTIVE` / `id rev N`, or `NO MAP` in warn).
  The localisation state is a tile row (state, can_confirm, scan_match, scan_long) plus
  a compact 3×2 of `cov_*` and `*_age_s`; ages over 1 s are warn. The run state is tiles
  (state, run id, step, cross-track m, remaining turn °) plus the reason as a note. The
  panel-motion sentence is a `.warn` callout (§3.6). Buttons: Pause/Prepare resume
  `big`, Abort `danger`, Acknowledge fault `big`.
- **Commissioning** (stacked, not canvas): plan textarea in 12.5px mono with the kinds
  line as an eyebrow. The job phase is a key tile; segment, progress L/R m, v m/s,
  encoder x/y/heading and gyro heading are tiles. The evidence path is a `code` line.
  Results keep their current rendering, restyled as `.list`.

`fmt()` in `run.js` and the string builders in the templates are replaced by small
`tiles(container, [[label, value, unit, level], …])` helpers in `amr.js`. Put it in
one shared function so four pages don't each re-implement it.

### W6 — Canvas graphics (`mapview.js`, `liveview.js`, `editor.js`, `run.js`)

Read the colours once from CSS so the canvas uses the UI tokens (spec
`canvas_graphics`):

```js
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const INK = { paper: css('--paper'), rule: css('--rule'), ink3: css('--ink-3'),
              route: css('--accent-2'), pose: css('--ok'), scan: css('--stop'),
              turn: css('--hazard-ink'), preview: css('--hazard') };
```

| Drawing | Old | New |
|---|---|---|
| canvas ground | `#0d0f12` | `--paper`. The map PNG is 254 free / 205 unknown / 0 occupied, so a 1px `--rule` frame around the image extent keeps its edge visible |
| route straight | `#3d8bfd` | `--accent-2`, 3px |
| invalid step | `#e74c3c` | `--stop` |
| turn marker + label | `#f0ad4e` | `--hazard-ink` dot/arrow, label in mono |
| start pose / footprint | `#2ecc71`, `#2ecc7188` | `--ok`, footprint `--ok` at 55% alpha |
| drag preview | `#f0ad4e` | `--hazard` |
| live vehicle pose (run/editor) | `#fff` | `--accent` arrow + footprint 55% |
| live scan | `#e74c3c`, stale `#8b1a1a` | `--stop`; stale = `--ink-3` at 50% alpha **and** a `STALE SCAN` label top-left in the stop colour (spec: stale must never look like clear) |
| live pose stale | `#777` | `--ink-3` + the same `STALE POSE` label |
| step-id text | `#9cc`, `12px system-ui` | `--ink-3`, `500 11px "IBM Plex Mono"` |

Wait for `document.fonts.ready` before the first `draw()` so labels don't come out in
a fallback font.

### W7 — Responsive and fixed viewport

- `html, body { height:100%; overflow:hidden }`; `main { overflow:auto }` as the
  safety valve. Keep the reference comment about DISARM out of reach, reworded for
  Abort/Stop.
- `@media (max-width: 820px)`: the rail stacks above main with tiles in auto-fit, the
  log is capped at 110px and the document may scroll. `.editor` becomes one column with
  the canvas at `height: 60vh`. `.io-cols` and `.cols` become one column. `button.dir`
  gets `aspect-ratio: 1/1`.
- `MapView._resize()` already reads the bounding rect on `resize`. Check it after the
  layout change, because the canvas container height now comes from the grid row
  instead of a calc.

### W8 — Guard test (`test/test_server.py`)

Extend `test_state_and_footprint` or add `test_pages_and_style`:

- All ten pages return 200 (today only `/maps`, `/editor` and `/run` are checked).
- `/static/fonts/plex-mono-400.woff2` returns 200 with a woff2 content type.
- Static-text checks on `amr.css` and every template/JS file: no `http://` or
  `https://` URLs, no `prefers-color-scheme`, no `box-shadow`, and `border-radius`
  only with `50%` or `2px`.

These checks are about the mechanism, not visual tuning. Pixel checks stay manual (§5).

## 5. Verification

1. `colcon build --symlink-install --packages-select amr_web` and
   `pytest amr_ws/src/amr_web/test` pass; ruff has nothing new to flag (JS/CSS aren't
   linted).
2. Run the sim stack (`nav.launch.py sim:=true web:=true`, domain 20) and walk every
   page at **1920×1080, 1280×800 and 390×844**. In each case: no document scroll above
   820px, no horizontal scroll at 390, and the rail and log are visible.
3. Stale/unknown states, forced in sim: stop the fake panel (selector tile bad, alarm
   row standing), kill `web_node`'s upstream (rail dims, pill bad), and switch modes
   (tiles warn during TRANSITIONING, stale localisation tile dimmed).
4. **Jog regression**, because W4 touches `jogpad.js`: in sim with selector MANUAL, check
   hold → moves and `.active`; release → stops and `.active` cleared; hold then Alt-Tab →
   released (`blur`); hold with focus in a text field → refused; Space → stop. Do this on
   Manual, Maps and Run.
5. Route editor: draw start + straight + CCW 90 + straight, then validate, save, create
   a mission, load it on Run and preview it. Colours are per W6 and labels are in Plex Mono.
6. Offline: disconnect the network on the viewing laptop, reload, and check the fonts
   still render (served from :5001).
7. Glyph check: the bundled fonts are a Latin subset. The arrows `↖ ▲ ◀`, `✓` and `×` fall
   back to a system font; confirm they render on the vehicle's browser, or replace them
   with ASCII/`kbd` labels.
8. Before U10: `AMR_SIM_TESTS=1 pytest src/amr_bringup/test/test_unified_sim.py` (P7)
   still passes, since it exercises the web API through the same server.

## 6. Risks

| Risk | Mitigation |
|---|---|
| A string-to-tile rewrite drops a field an engineer relied on (e.g. `operation_id`, `cov_yaw`) | §3.7. Diff the field list of every old `.state` string against the new tiles during review |
| An `id` rename silently breaks a handler (no JS tests) | Keep every `id` the JS reads; §5.2–5.5 exercise every button |
| Ten nav tabs + pill wrap onto two header lines and push the fixed-height layout | Header is `flex: 0 0 auto`; the shell takes the remainder, so wrapping costs height but never hides main |
| The canvas gets a 0px height in the new grid | `.editor { min-height: 0 }` and `.canvas-wrap { min-height: 300px }`; checked in §5.2 |
| Restyle lands after U10 and invalidates the acceptance build | §3.8: land and sim-verify before the vehicle session |
