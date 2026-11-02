# Reading JIS-style electrical drawings

The reference drawings are machine-tool electrical manuals drawn to JIS conventions (e.g. IZUMI FW30HS-T, 57 sheets). Our sheets follow the same conventions, in English only.

## Sheet anatomy
- **Portrait A4/A3.** Title block across the bottom; revision / cross-reference grid directly above it.
- **Line numbers 00–21** down the left edge. Every device location is "sheet + line": `3311` = sheet 33, line 11.
- **Current flows top → bottom and left → right.** Supply buses run down the left; loads sit on the right with their description text at the far right.
- **Descriptions are in English** (capitals). Source drawings may add a second language underneath; translate, do not copy it.
- Proprietary note printed vertically in the margin.

## Power circuits
Typical chain for one motor, left to right:

`bus tap → cable mark → MS (contactor) → OL (overload) → SK (surge killer) → cable mark → U/V/W terminals → motor M`

- The motor circle shows its name; the rating is printed next to its earth lead: `0.4kW-4P` (kW, poles), `75W-2P`, `20W`.
- Breaker rating `50AF/50AT` = frame 50 A / trip 50 A. `225AF/125AT` = 225 A frame, 125 A trip.
- Overload setting next to the heater: `2.2A`, `0.52A`.
- The small number under a contactor (e.g. `3311`) is where its **coil** is drawn.
- Numbers in the grid under a breaker name (e.g. `CBSPL / 1201`) are where its **auxiliary contacts** are drawn.
- Bus names carry across sheets: `R S T` incoming → `R1 S1 T1` after the main breaker → `R2 S2 T2`, `R3 S3 T3` after feeder breakers. A bus ending at the grid (tee or arrow) continues on a later sheet.

## Control circuits
- Two vertical rails (often named after the control transformer secondary or R3/S3), horizontal rungs between them, one rung per line number.
- The load (coil, lamp, solenoid) sits at the right end of the rung; contacts are to its left.
- Under a coil: list of where its contacts are used. Under a contact: where its coil is.
- Parallel branches (self-hold, OR conditions) are drawn directly under the rung they belong to.

## Device prefixes (common, confirm from the drawing)
| Prefix | Device |
|---|---|
| CB, MCCB, CP, CPO | Circuit breaker / circuit protector |
| ELB | Earth-leakage breaker |
| MS | Magnetic starter (contactor + overload) |
| MC | Magnetic contactor |
| OL, THR | Thermal overload relay |
| SK | Surge killer (CR surge absorber) |
| M | Motor |
| CR, R, X, K | Control relay |
| T, TR, TM | Timer (or transformer — check context) |
| PB | Push-button |
| SW, SS, COS | Switch / selector switch |
| LS, PS, FS | Limit / pressure / float switch |
| DSW | Door switch |
| PL, GL, RL, WL, OL(lamp) | Pilot lamps (green / red / white) |
| FU, F | Fuse |
| FL | Fluorescent lamp (panel lighting) |
| SOL, SV | Solenoid valve |
| TB | Terminal block |

## Wire and cable notation
Written next to a small crossing mark with arrowheads on each conductor, e.g. `KIV2mm²` over `BLACK`.
- **KIV**: PVC-insulated flexible panel wire. **IV**: solid panel wire. **MLFC**: flame-retardant flexible cable for large currents. **VCT**: cab-tyre cable to field devices.
- Size in mm² (2, 5.5, 14, 60 …).
- Colours commonly used in these panels (the drawing always wins):
  - black — main AC power;
  - red — AC control;
  - blue — DC control;
  - green or green/yellow — earth;
  - yellow — circuits that stay live when the main breaker is off (e.g. panel lighting fed ahead of CB1).

## Symbols as drawn in these manuals
| Symbol | Looks like |
|---|---|
| Breaker (MCCB) | Two small circles bridged by an arc, then a small square jog (thermal element); dashed line links the poles |
| Contactor main contact | Conductor broken by two short parallel bars `—| |—` |
| NC contact | Same bars with a diagonal slash |
| Overload heater | Square jog on the outer two phases |
| Surge killer | Small boxes on vertical links between the phases |
| Motor | Circle with name; three terminal circles on the left; earth lead with `E` terminal at lower right |
| Junction | Filled dot. Lines that cross without a dot are **not** connected. |
