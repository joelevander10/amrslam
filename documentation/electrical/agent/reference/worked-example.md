# Worked example — FW30HS-T sheet 7 (power circuit)

Source photo: `elec-diagram-reference/photo_2026-09-29_18-22-03.jpg` (project root).
Result: `diagrams/sheets/fw30hs-t-sheet-07.html` (checked with `preview.ps1`; PNG/PDF in `export/` were made on request).
Sheet 4 (`photo_2026-09-29_18-22-04.jpg`, photographed rotated 90°) → `sheets/fw30hs-t-sheet-04.html` shows a second pattern: incoming supply, a vertical main breaker (`mccbV`), 2-pole breaker, door switch, lamp, outgoing feeders with `endLabels`, and grid cross-refs.

A second, hand-sketch example with two-wire DC circuits (battery → 2-pole MCB → 1-pole feeders and buck converters; terminal blocks and motor drivers) is `input-sketch/proj01/page01.jpg` / `page02.jpg` → `input-sketch/proj01/proj01-sheet-01.html` / `-02.html`. The patterns it uses are listed in `api.md`, "Two-wire DC sheets".

## 1. What the photo shows
- Portrait sheet, upright. Line numbers 00–21 on the left, title block at the bottom.
- Three bus lines R1 S1 T1 run down the full left side and stop at the grid (they continue on later sheets).
- Line 01: a tap off R1/S1/T1 with cable mark "(AC200V) KIV1.25mm² BLACK" → breaker CB3 50AF/50AT. After CB3 the three lines are R3/S3/T3: they continue straight into the first motor branch **and** drop down the page as a sub-bus.
- Six identical motor branches on lines 01, 04, 07, 10, 13, 16 (every 3 lines). Each: KIV2mm² BLACK → MSx → OLx (setting) → SKx → KIV2mm² BLACK → Ux Vx Wx → motor Mx with rating, description on the right. The number under each MS is its coil location (3311 … 3316).
- R3/S3/T3 turn right at lines 18–20 and leave as "AC200V CONTROL POWER".

## 2. Interpretation (as sent to the user, shortened)

| Line | Taps from | Devices | Load |
|---|---|---|---|
| 01 | R1 S1 T1 | ?KIV1.25mm² BLACK (AC200V) → CB3 50AF/50AT | R3 S3 T3 |
| 01 | (straight from CB3) | KIV2mm² → MS3L (3311) → OL3L 2.2A → SK3L → KIV2mm² → U3L V3L W3L | M3L 0.4kW-4P · L. SPINDLE LUB. PUMP MOTOR |
| 04 | R3 S3 T3 | MS3R (3312) · OL3R 2.2A · SK3R | M3R 0.4kW-4P · R. SPINDLE LUB. PUMP MOTOR |
| 07 | R3 S3 T3 | MS4 (3313) · OL4 0.52A · SK4 | M4 75W-2P · CLAMP LUB. PUMP MOTOR |
| 10 | R3 S3 T3 | MS5 (3314) · OL5 0.11A · SK5 | M5 20W · AUX. HYDRAULIC FAN COOLER |
| 13 | R3 S3 T3 | MS6L (3315) · OL6L 0.11A · SK6L | M6L 20W · L. SPINDLE LUB. FAN COOLER |
| 16 | R3 S3 T3 | MS6R (3316) · OL6R 0.11A · SK6R | M6R 20W · R. SPINDLE LUB. FAN COOLER |
| 18–20 | R3 S3 T3 | — | AC200V CONTROL POWER |

Doubtful: the wire size on line 01 (1.25 mm²?). Assumption: surge-killer internals drawn with the kit's standard symbol.

## 3. Code — the whole `draw()`

```js
const BRANCHES = [
  { ms: 'MS3L', ref: '3311', ol: 'OL3L', amps: '2.2A', sk: 'SK3L', w: 'KIV2mm²',
    t: ['U3L', 'V3L', 'W3L'], m: 'M3L', kw: '0.4kW-4P',
    en: 'L. SPINDLE LUB. PUMP MOTOR' },
  // … five more rows, same shape
];

draw(s) {
  const busA = [23.8, 27.9, 31.8];            // R1 S1 T1
  const busB = [70.6, 75.2, 79.7];            // R3 S3 T3
  const busBEnd = [212.9, 217.9, 222.3];      // where R3/S3/T3 turn right
  const top = s.phases(s.row(1));

  s.bus(busA, 11.6, JIS.G.gridY, ['R1', 'S1', 'T1'], 'tee');

  busB.forEach((x, i) => {
    s.line(x, top[i], x, busBEnd[i]);
    s.line(x, busBEnd[i], 153, busBEnd[i]);
    s.dot(x, top[i]);
    s.text(x + 1.3, top[i] - 0.6, ['R3', 'S3', 'T3'][i], { size: 1.9 });
    s.text(154.2, busBEnd[i] + 0.7, ['R3', 'S3', 'T3'][i], { size: 2 });
  });
  s.desc(163, 212.6, 'AC200V CONTROL POWER');

  // first branch runs straight on from CB3; the others tap the R3/S3/T3 sub-bus
  BRANCHES.forEach((b, k) =>
    s.branch(b, s.phases(s.row(1 + 3 * k)), { xs: k === 0 ? busA : busB, tap: true }));

  // feeder cable + breaker last, so they cut the conductors already drawn
  s.wire(35.9, top, '?KIV1.25mm²', 'BLACK', '(AC200V)');
  s.mccb(52.5, top, 'CB3', '50AF/50AT');
}
```

## 4. Lessons from building it
- Most of the effort is reading; the code is short because every branch is one data row.
- Put the feeder breaker **after** the conductors — it whites-out the line under itself.
- Labels that sit close together (cable colour vs. terminal names, breaker rating vs. bus names) are where overlaps happen — check them first on the render.
- Items that could not be read with certainty are prefixed `?` so they stay red until someone checks the paper original.
