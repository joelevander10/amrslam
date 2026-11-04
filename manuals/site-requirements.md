# Site requirements: 103 Tyre Building

Product-neutral decisions kept from the tracked site layout (agreed with the operator
2026-10-17) when the tracked product was removed (`manuals/trackless-only-removal-plan.md`
D20). The tape network, junction orders and stop list went with it (tag `tracked-final`).
**Scope now: AGV-01 only.** Two-AGV traffic is deferred until AGV-02 starts.

## Decisions

| # | Decision | Source |
|---|---|---|
| D1 | Two AGVs, fixed halves split along the middle line ("103 TYRE BUILDING"). No AGV serves the other's machines | operator |
| D2 | AGV-01: BTU9–24 and MRU1–4 (20 machines). AGV-02: BTU1–8, STU1–10 (18 machines) | operator |
| D3 | A trip serves **at most 2 machines, both from one group**. AGV-01 groups: **MRU** (MRU1–4) and **BTU** (BTU9–24). Never mixed | operator |
| D4 | The AGV waits at HOME until two machines of one group have called. Cycle time and capacity are out of scope for now | operator |
| D5 | Calls are entered on the AGV's own display (or the web page from a laptop). No call buttons, no central system | operator |
| D6 | Facing machines share one stop point; the AGV stops in the same place for both | operator |
| D7 | **BTU:** at a stop the AGV waits; the operator swaps the trolley; panel Start continues | operator |
| D8 | **MRU:** westbound, the AGV stops at each called machine and the trolley is **released**. It drives on empty to the end of the MRU line, turns, and on the eastbound pass stops again at the same machines to **take the trolley back** | operator |
| D10 | There is no payload detection. A rear lidar may later detect the trolley; ignored for now | operator |
| D11 | AGV-01's **home position is the north bay** of the HOME area, where it parks **empty**. The **attach point** is on the departure lane after the turn; the exact stop is taught on the floor. **Trolleys are released at each AGV's own home position** (AGV-01: north bay, AGV-02: south bay) | operator, 2026-10-17 mark-ups |
| D13 | AGV-01 only for now: no collision or priority logic | operator |
| D14 | **Leaving home needs a physical Start.** A call (or a formed trip) on the display never moves the vehicle (no web page starts the vehicle; autonomous motion only under AUTO + a fresh panel Start). When a trip forms, the HMI shows it; the operator presses Start at the bay | safety invariants (CLAUDE.md) |

## HOME-area pillars

From the HOME-area projection drawing (operator mark-up, 2026-10-17; dimensions about
±200 mm, verify on site). Projection coordinates in mm, origin at the left merge datum;
the projection is rotated against the site plan (its up = east, left = north).

| Pillar | x (mm) | y (mm) | Size |
|---|---|---|---|
| LL | 0 – 240 | 3220 – 3460 | 240 × 240 |
| UL | 0 – 240 | 4235 – 4475 | 240 × 240 |
| LR | 3764 – 4004 | 3220 – 3460 | 240 × 240 |
| UR | 3764 – 4004 | 4235 – 4475 | 240 × 240 |

The two pillar pairs are 3524 mm apart (inner faces) and the pillars in each pair 775 mm
apart. This is the input for placing the GLS label in the HOME area.
