# Engine API — `kit/jis-sheet.js`

A sheet page calls:

```js
JIS.render(document.getElementById('sheet'), {
  meta: META,               // title block + grid refs (see below)
  file: 'FW30HS-T-00-91-07',// base name for the in-page PNG download
  label: 'One sentence describing the sheet',
  draw(s, X) { ... },       // s = sheet object, X = default branch columns
});
```

Everything is in **millimetres** on an A4 portrait sheet (`viewBox 0 0 210 297`). y grows downward.

## Sheet geometry (`JIS.G`)

| Item | Value |
|---|---|
| Frame | x 14 → 202.8, y 8 → 291 |
| Line numbers | `s.row(r)` = 13 + 10.7 × r, r = 0…21. Fractions are fine (`s.row(7.3)`). |
| Top of drawing area | y ≈ 11.6 (bus labels sit just above this) |
| Bottom of drawing area | `JIS.G.gridY` = 240.5 (buses end here) |
| Revision / cross-ref grid | y 240.5 → 269, 24 columns from x 20 |
| Title block | y 269 → 291 |
| Usable text right edge | x ≈ 201 |

## `META` (title block)

```js
{
  customer,                    // top-left cell, upper half: customer name
  designer,                    // top-left cell, lower half: designer company
  content,                     // CONTENT row, e.g. 'POWER CIRCUIT'
  approval, checked, designed, // names
  date, scale, drawn,
  title,                       // 'ELECTRIC DIAGRAM'
  model, dwg,                  // 'FW30HS-T', '-00-91-'
  sheet, of,                   // 7, 57
  note,                        // vertical margin note ('' = none)
  refs: [{ name: 'CB2', ref: '1201' }],  // cross-refs shown in the grid, left to right
}
```
Empty string = cell left blank. `«…»` = placeholder (renders red). Keep the `of` placeholder short (`'«XX»'`): the sheet-total cell is narrow and `'«TOTAL»'` runs past the frame.

## Column plan — power sheets

| x (mm) | Use |
|---|---|
| 23.8 / 27.9 / 31.8 | Incoming 3-phase bus (R1 S1 T1 or R S T) |
| 35.9 – 38.5 | Cable spec mark on a tap |
| 52.5 – 56 | Feeder breaker (`mccb`) |
| 70.6 / 75.2 / 79.7 | Sub-bus after the feeder breaker (R3 S3 T3 …) |
| `X.win` 85 | Cable spec into the branch |
| `X.ms` 99.5 | Contactor |
| `X.ol` 112.2 | Overload relay |
| `X.sk` 120.9 | Surge killer |
| `X.wout` 127 | Cable spec to the motor |
| `X.term` 140.1 | Motor terminals |
| `X.mcx` 148.4, `X.mr` 5.2 | Motor circle centre and radius |
| `X.desc` 163 | Load description (one line, optional second line) |
| ~150–153 | End of outgoing conductors, then `endLabels` |

Phase spacing inside a branch is 4.2 mm: `s.phases(y)` → `[y, y+4.2, y+8.4]`.
A motor branch needs about **2 rows** of height; the original drawings usually space branches **3 rows** apart.
Description text: ≤ 28 characters per line, or it runs past the frame. Longer → split over `en` / `en2`.

## Column plan — control sheets

| x (mm) | Use |
|---|---|
| `RAILS[0]` 30 | Left rail (supply) |
| 38 – 125 | Contacts, push-buttons, fuses — keep ≥ 8 mm between items |
| `RAILS[1] − 12` ≈ 138 | Coil or lamp (the load sits next to the right rail) |
| `RAILS[1]` 150 | Right rail (return) |
| 154 | Rung description |

One rung per line number. A parallel (OR) branch runs 6 mm below its rung, so leave the next line free when you use one.

## Drawing functions (`s.`)

### Primitives
| Call | Draws |
|---|---|
| `row(r)` | y of line number r |
| `phases(y, gap = 4.2)` | `[y, y+gap, y+2·gap]` |
| `line(x1, y1, x2, y2, w = 0.3, dash?)` | Conductor or rule |
| `path(d, w = 0.25, fill = 'none')` | Free SVG path |
| `rect(x, y, w, h, sw = 0.25)` | White box with outline |
| `dot(x, y)` | Junction dot (T-junction). Crossing conductors get **no** dot. |
| `ring(x, y, r)` | Terminal circle |
| `knock(x1, y1, x2, y2)` | White-out a gap in a conductor (symbols do this themselves) |
| `text(x, y, str, { size, anchor, weight, sans, rotate })` | Text. Default size 2.2 mm mono. Prefix `?` → red "uncertain". `«…»` → red placeholder. |
| `desc(x, y, en, en2?)` | Load description, optional second line |
| `endLabels(x, ys, labels)` | Labels at the end of outgoing conductors |

### Power symbols
| Call | Symbol |
|---|---|
| `bus(xs, y1, y2, labels?, end?)` | Vertical bus lines; `end` = `'tee'` (continues on next sheet) or `'arrow'` |
| `wire(x, ys, spec, colour, pre?)` | Cable spec mark: crossing line with arrowheads + label (`'KIV2mm²'`, `'BLACK'`, optional `'(AC200V)'` above) |
| `mccb(x, ys, name, rating?, aux?)` | Moulded-case breaker with thermal element, horizontal flow. `ys` may be 2 or 3 lines. `aux` = small note above the rating |
| `mccbV(xs, y, name, rating?)` | Same breaker on vertical conductors |
| `contactor(x, ys, name, ref?)` | Main contacts `—| |—`; `ref` (coil location) printed under the bottom line |
| `overload(x, ys, name, amps?, poles?)` | Thermal overload heaters on the outer phases |
| `surge(x, ys, name)` | Surge killer (CR absorber) across 3 phases |
| `noSwitch(x, y, name)` | Normally-open switch on one conductor (door switch etc.) |
| `earth(x, y)` | Earth symbol |
| `motor(b, ys, x?)` | Terminals + motor circle + earth + rating (normally called by `branch`) |
| `branch(b, ys, from, xo?)` | Complete motor branch (see below) |

`branch(b, ys, from, xo)`:
```js
b = {
  ms: 'MS3L', ref: '3311',          // contactor + coil cross-ref
  ol: 'OL3L', amps: '2.2A',         // omit ol → no overload drawn
  sk: 'SK3L',                        // omit sk → no surge killer
  w: 'KIV2mm²', wc: 'BLACK',         // cable (wc defaults to BLACK); omit w → no cable marks
  wout: 'KIV2mm²',                   // optional different cable to the motor
  t: ['U3L', 'V3L', 'W3L'],          // terminal names
  m: 'M3L', kw: '0.4kW-4P',          // motor name + rating
  en: 'L. SPINDLE LUB. PUMP MOTOR',  // description; en2 = optional second line
}
from = { xs: [x1, x2, x3], tap: true } // where the three conductors start; tap → junction dots
xo   = { ms: 102 }                      // optional per-branch column overrides
```
**Draw order matters:** draw conductors first, then breakers/symbols that sit on them (they white-out the conductor under themselves). `branch()` already does this internally; for feeder breakers call `wire()` / `mccb()` after the conductors exist.

### Control symbols
| Call | Draws |
|---|---|
| `rails(xL, xR, y1, y2, [labelL, labelR])` | Two vertical rails |
| `rung(y, [xL, xR], items, { en, en2 }?)` | Conductor between the rails with junction dots, items on it, description right of the right rail |
| `seg(y, x1, x2, items)` | Plain conductor segment with items (no rail dots) |
| `parallel(y, yb, x1, x2, items)` | OR branch: drops at x1, runs at yb, rises at x2 |
| `item(it, y)` | One item (normally called by rung/seg/parallel) |

Item kinds (`{ k, x, ... }`):

| `k` | Symbol | Props |
|---|---|---|
| `no` | Normally-open contact `| |` | `name`, `ref` |
| `nc` | Normally-closed contact `|/|` (also use for OL trip contacts) | `name`, `ref` |
| `pbno` | Push-button, normally open | `name` |
| `pbnc` | Push-button, normally closed | `name` |
| `coil` | Relay/contactor coil (circle, name inside) | `name`, `refs` (array of contact locations) |
| `lamp` | Pilot lamp (circle with X) | `name`, `color` |
| `fuse` | Fuse | `name`, `rating` |
| `wno` | Wire number above the conductor | `text` |

Multi-pole manual switches (functions, not item kinds; draw after the conductors, they knock their own gaps):

| Call | Symbol |
|---|---|
| `estop(x, ys, name?)` | Emergency-stop push-button: one NC pole per y (top first), dashed mechanical link, mushroom head, latching mark (turn to release). Head sits 4.4 mm above the top pole |
| `selector(x, poles, name?)` | Maintained selector switch: `poles = [{ y, k: 'nc' \| 'no' }]`, dashed link, turn actuator + latching mark 4 mm above the top pole |
| `pushbutton(x, poles, name?)` | Momentary push-button with any mix of poles `[{ y, k: 'nc' \| 'no' }]` (1NO, 2NO, 1NO1NC …), dashed link, push actuator 2.8 mm above the top pole; name to the right of the actuator. Use instead of `pbno`/`pbnc` when the button has more than one contact or needs terminal numbers |
| `switchPoles(x, poles, head, nums = true)` / `detent(x, y)` | Building blocks of the two above (pole row + link + stem; latching notch) |

All three print contact terminal numbers per IEC 60947-5-1 / JIS C 8201-5-1 beside each pole's rings (over an NC pole, under an NO pole): first digit = pole order (top pole = 1), second = function, NC `1–2`, NO `3–4`. A 2NC E-stop reads 11-12 / 21-22; a 1NC1NO selector with the NC on top reads 11-12 / 23-24. Pass `{ flip: true }` to swap each pole's pair (14 left, 13 right) when the terminal wired as 14 sits on the left of the drawing; `{ nums: false }` hides them (e.g. when the real part is marked differently).

### Not in the kit yet
Timer coils and timed contacts, limit/pressure/float switches, solenoid valves, transformers, terminal blocks, PLC I/O modules, DC power supplies. Approximate with the nearest symbol plus a correct label, and say so — or extend the kit (see SKILL.md, "Missing symbol").

### Two-wire DC sheets (pattern from `input-sketch/proj01/`)
The kit is built for three-phase AC but handles DC with primitives. Local helpers go inside the sheet's `draw()`; `kit/` stays unchanged.
| Need | How it was done |
|---|---|
| +/− bus | `s.bus([23.8, 29.8], 23, JIS.G.gridY, ['P48','N48'], 'tee')` plus a start bar at y 23; 6 mm spacing leaves room for 4-character labels |
| 2-pole main breaker | `s.mccbV(bus, y, 'MCB-01', '2P 32A')` |
| 1-pole feeder breaker | `s.mccb(x, [y], name)` and the rating drawn separately at `x + 5.4, y - 1.6` (the built-in rating position touches the thermal jog on a single conductor) |
| Branch from the + bus | Conductor from the + bus with `s.dot`; it crosses the − bus with **no** dot (sketch "hop" = not connected) |
| Cable mark on vertical conductors | local `wireV(xs, y, l1, l2)`: horizontal crossing line, arrowheads pointing onto each conductor, label to the right (see sheet 01) |
| DC/DC converter, motor driver | `s.rect` box drawn after the conductors (white fill hides them), `s.ring`/`s.dot` terminals on the edge, name / rating / model centred inside |
| Terminal-block terminal (TB) | `s.ring(x, y, 0.8)` + a 45° slash `s.line(x-1.2, y+1.2, x+1.2, y-1.2, 0.2)`, name above at size 1.7 |
| Wire labels (cable markers) | `s.text(x, y - 0.6, '1P24', { size: 1.9 })` on every segment: x 56 (bus → TB), `TBX + 4` (TB → device), and `DX - 1.5` with `anchor: 'end'` at the device end. Terminal names (A1, 20/NET-VIN) go inside the box at `DX + 1.5` |
| Bus origin from another sheet | small sheet+line ref above the bus label, e.g. `'0105'` at y 19.4, size 1.5 |

## Review markers
- `'?value'` → rendered red without the `?`, listed as *uncertain*.
- `'«text»'` → rendered red, listed as *placeholder*.
- The toolbar shows the count; the browser console lists every item. Exported files show them red — a sheet is final only when nothing is red.

## Preview and export
The HTML file is the deliverable. During a run, check renders with the preview script only:
- `pwsh -File diagrams/agent/scripts/preview.ps1 <path-to-sheet.html> [...]` → `%TEMP%\jis-preview\<name>.png` (2480 × 3508). Works for sheets in any folder; writes nothing to the project.

PNG/PDF files only when the user asks:
- `pwsh -File diagrams/agent/scripts/export.ps1 [name ...]` → `export/<name>.png` (2480 × 3508, A4 at 300 dpi) and `export/<name>.pdf` (vector, one A4 page). No names = every non-template sheet. It reads `sheets/` only; for sheets elsewhere, run the same Edge commands on that path.
- In the browser: **Download PNG** and **Print / PDF** (choose "Save as PDF", A4, margins none).
- `sheets/<name>.html#bare` shows the sheet alone at 1240 × 1754 px (used by the script).
- AutoCAD: `PDFIMPORT` on the PDF gives editable lines and text.
