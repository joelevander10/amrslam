---
name: jis-electrical
description: Draw JIS-style electrical diagrams (power circuits and control/ladder circuits) as HTML/SVG sheets from a photo of an existing drawing or a pen-on-paper sketch. The deliverable is the HTML file; PNG/PDF only on request. Use when the user gives an electrical schematic photo or sketch, or asks for an electrical sheet in this project.
---

# JIS electrical sheets

You turn a **photo of an existing drawing** or a **hand sketch** into a clean A4 electrical sheet in JIS drafting style: line-number gutter 00–21 on the left, revision/cross-reference grid, title block at the bottom, black-on-white.

The drawing engine already exists. Your job is **reading the source correctly and filling in data**, not designing graphics. You should almost never write new geometry.

## Folder map (all paths relative to `diagrams/`)

| Path | What it is | May you edit it? |
|---|---|---|
| `kit/jis-sheet.js` | Engine: frame, title block, grid, every symbol, export | Only to add a missing symbol (see "Missing symbol") |
| `kit/jis-sheet.css` | Page chrome, print and `#bare` screenshot mode | No |
| `sheets/_template-power.html` | Template for power-circuit sheets (breakers → contactors → motors) | Copy it, never edit it |
| `sheets/_template-control.html` | Template for control sheets (rungs between two rails) | Copy it, never edit it |
| `sheets/fw30hs-t-sheet-04.html`, `-07.html` | Finished reference sheets (three-phase AC) | Read as examples |
| `input-sketch/proj01/proj01-sheet-01.html`, `-02.html` | Finished reference sheets (two-wire DC: breakers, converters, terminal blocks, drivers) | Read as examples |
| `export/` | PNG + PDF output | Written by the export script, only on request |
| `agent/reference/*.md` | The documents listed below | Read them |
| `agent/scripts/preview.ps1` | Renders a throw-away PNG to `%TEMP%\jis-preview\` for the visual check | Run it on every build |
| `agent/scripts/export.ps1` | Renders sheets in `sheets/` to PNG/PDF in `export/` | Only when the user asks for PNG/PDF |

## Read before starting (in this order)

1. `agent/reference/jis-conventions.md` — how these drawings are organised and what the device names and notations mean.
2. `agent/reference/sketch-reading.md` — how to read a photo or sketch, and the interpretation step.
3. `agent/reference/api.md` — every engine function, the coordinate system, and the layout plan.
4. `agent/reference/worked-example.md` — one complete source → interpretation → code example.
5. `agent/reference/checklist.md` — what to check on the rendered output.

## Hard rules

1. **Never invent a value.** Ratings, wire sizes, device names, cross-reference numbers and names in the title block come from the source or from the user. If you cannot read something, write it with a leading `?` (for example `'?2.2A'`) — it renders red and is counted. If it is missing entirely, leave the `«placeholder»` in place. Both show up red so a human sees them.
2. **Interpret first, draw second.** Before writing any sheet code, produce the interpretation (step 2 below) and **stop for the user's confirmation**, unless the user explicitly said to proceed without it.
3. **At most 3 questions**, only about things that change the drawing. Everything else becomes a stated assumption.
4. **Use the templates and the engine.** Copy a template, fill data, use `s.row(n)` for vertical placement and the column plan in `api.md` for horizontal placement. Do not hand-position every element; do not restyle.
5. **One sheet per HTML file**, named `<model>-sheet-<NN>.html` (lower-case model, two-digit sheet number). Default folder is `sheets/`; if the user names another place (e.g. "same directory as the source"), put it there and fix the relative `kit/` paths (`../../kit/` from `input-sketch/<proj>/`).
6. **HTML is the deliverable.** Do not write PNG or PDF files into the project during a run. Use `preview.ps1` for checking; run `export.ps1` only when the user asks for PNG/PDF.
7. **Keep JIS style**: black on white, A4 portrait, **English only**. If the source has Japanese text, translate it to English and list each translation in the interpretation for confirmation. No colour, no modernised layout, unless the user asks.
8. **Never publish or upload** the sheets or source photos anywhere. They may be proprietary drawings.

## Workflow

### Step 1 — Intake
- Look at every image. Note the orientation (phone photos of binders are often rotated 90°; the title block is always at the bottom of the true sheet and the line numbers run down the left).
- Decide the sheet type: **power** (three-phase lines, breakers, contactors, motors) or **control** (two rails, contacts and coils on horizontal rungs). A sheet can mix both; use the power template and add rungs.
- Collect the title-block data if the source is an existing drawing.

### Step 2 — Interpretation (then stop)
Fill in `agent/templates/interpretation.md` and show it to the user. It lists the title block, buses and continuations, every branch or rung with its devices and values, unreadable items, assumptions, and at most 3 questions. Wait for confirmation or corrections.

### Step 3 — Build
- Copy the matching template to `sheets/<model>-sheet-<NN>.html` (or the folder the user named).
- Replace the `META` block, then the data arrays (`BRANCHES` / `RUNGS`), then adjust `draw()` only as far as the structure differs.
- Use the row numbers from the source: a device on line 07 of the original goes at `s.row(7)`.

### Step 4 — Render and check
- Run `pwsh -File diagrams/agent/scripts/preview.ps1 <path-to-sheet.html> [...]` (Windows PowerShell also works: `powershell -File ...`). It prints the path of a throw-away PNG in `%TEMP%\jis-preview\`.
- Open that PNG and go through `agent/reference/checklist.md`.
- Fix what fails, preview again, check again. **Maximum two fix rounds**; if problems remain, report them instead of looping.
- Do not run `export.ps1` unless the user asked for PNG/PDF.

### Step 5 — Report
Tell the user, briefly:
- the HTML file path(s);
- every item still red (uncertain `?` or `«placeholder»`), with its line number;
- assumptions you made and any symbol you approximated.

## Missing symbol

If the source uses a symbol the kit does not have (see the list in `api.md`):
1. First choose the nearest existing symbol and label it correctly, and mention the approximation in the report. This is the default.
2. Only if the user wants the exact symbol: add a new `case` to `P.item` (control) or a new `P.<name>` function (power) in `kit/jis-sheet.js`, in the same style (line widths 0.2–0.3 mm, `knock()` the conductor first, label above at size 1.9). Document it in `api.md`, then preview **every** finished sheet (`preview.ps1` with all sheet paths) and check that the existing sheets did not change.

## Effort guide
A typical sheet is: read source → one interpretation message → one build → one or two preview checks. If you find yourself computing many coordinates by hand, stop and use the row and column plan in `api.md` instead.
