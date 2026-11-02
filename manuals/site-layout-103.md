# Site layout: 103 Tyre Building (tape network, stations, trip flow)

Status: agreed with the operator 2026-10-17. **Scope now: AGV-01 only.** AGV-02 is recorded for
completeness; two-AGV traffic (zone lock, priority) is deferred until AGV-02 starts.

The current `missions/line-a.json` and branch config do NOT implement this layout. RFID trials
gave poor results, so the stops and junction orders here are to come from tape-graph localization,
not from RFID.

Localization plan: `manuals/tracked-plan-3-tape-localization.md`; procedures: `manuals/tape-localization-runbook.md`.

Sources: the simplified site plan (operator image, 2026-10-17, not in the repo; partly outdated)
and the HOME-area projection `documentation/layout-reference/homepos-area-projection/layout-projection.svg`.
Where they disagree, the projection wins. The operator's correction of the same day removes the
site plan's outer trunk: there is **one trunk**, entering the HOME complex at merge L (from the
north) and merge R (from the south).

Compass: site-plan north = the MRU line at the top. The projection is rotated: its up = east,
left = north, right = south, down = west. Projection coordinates below are mm, as printed on it.

## 0. Decisions

| # | Decision | Source |
|---|---|---|
| D1 | Two AGVs, fixed halves split along the middle line ("103 TYRE BUILDING"). No AGV serves the other's machines | operator |
| D2 | AGV-01: BTU9–24 and MRU1–4 (20 machines). AGV-02: BTU1–8, STU1–10 (18 machines) | operator |
| D3 | A trip serves **at most 2 machines, both from one group**. AGV-01 groups: **MRU** (MRU1–4) and **BTU** (BTU9–24). Never mixed: the track and flow are too complicated | operator |
| D4 | The AGV waits at HOME until two machines of one group have called. Cycle time and capacity are out of scope for now | operator |
| D5 | Calls are entered on the AGV's own display (or the web page from a laptop). No call buttons, no central system | operator |
| D6 | Facing machines share one stop point; the AGV stops in the same place for both | operator |
| D7 | **BTU:** at a stop the AGV waits; the operator swaps the trolley; panel Start continues | operator |
| D8 | **MRU:** westbound, the AGV stops at each called machine and the trolley is **released**. It drives on empty to the end of the MRU line, U-turns, and on the eastbound pass stops again at the same machines to **take the trolley back** | operator |
| D9 | **The AGV cannot U-turn with a payload, at either U-turn.** The MRU-end U-turn is a pivot at the tape end (as `line-a`'s `u_turn`), so every release must be done before it. The HOME U-turn R717 is driven empty: the trolley is attached after it (D11) | operator |
| D10 | There is no payload detection. A rear lidar may later detect the trolley; ignored for now | operator |
| D11 | AGV-01's **home position is the north bay** (left at J-SPLIT), where it parks **empty**. The **attach point** is on the departure lane after the U-turn. The operator's mark-up puts it at ≈ projection (2474, 3250–5190), from about 0.8 m to 2.8 m before J-N; the exact stop is taught on the floor. **Trolleys are released at each AGV's own home position** (AGV-01: north bay, AGV-02: south bay) | operator, 2026-10-17 mark-ups |
| D14 | **Leaving home needs a physical Start.** A call (or a formed trip) on the display never moves the vehicle (safety invariant: no web page starts the vehicle; autonomous motion only under AUTO + a fresh panel Start). When a trip forms, the HMI shows it; the operator presses Start at the bay, and the AGV drives empty through the U-turn to the attach point | safety invariants (CLAUDE.md) |
| D12 | Tape is laid today **only as far as MRU** (the MRU circuit, `line-a`). Row 2, the left vertical and the middle line are not laid yet | operator |
| D13 | AGV-01 only for now: no collision or priority logic | operator |

## 1. Tape network (AGV-01's half and HOME)

| Segment | Description |
|---|---|
| **HOME complex** | Per the projection. Arrival stem → bay split → two parallel bays (~1750 mm, 1150 mm apart; home positions) → bay merge → U-turn R717 → departure lane heading west (attach point, then three exits: R2434 north, R1570 south, straight on to the middle line). Two crossings: X1, X2 |
| **Trunk** | One tape (the site plan's outer trunk does not exist). North of HOME: bidirectional, AGV-01 only, from merge L to the MRU line's NE corner; row 2 joins it from the west. South of HOME: from merge R, AGV-02 (deferred) |
| **Middle line** | From the departure lane west to the left vertical. Westbound only |
| **Left vertical** | x≈45 on the site plan. From the middle line, north leads to row 2 (AGV-01) and south to row 3 (AGV-02) |
| **Row 2** | BTU17–24 above / BTU9–16 below. Driven eastbound, from the left vertical to the trunk |
| **MRU line** | MRU1–4 (single-sided). From the trunk's NE corner west to a tape end, where the empty AGV pivots and comes back east on the same tape |

## 2. Stop positions (AGV-01: 16 at machines, plus home and attach)

Listed in the order the AGV meets them on its circuit. Each is a separate taught position: the
eastbound MRU stops face the other way and need not coincide with the westbound ones.

| Position | Machines | Circuit | Pass | Operator action |
|---|---|---|---|---|
| HOME-N | – | both | north bay, end of the trip | release the trolleys; park (trip ends) |
| ATT | – | both | departure lane, after the U-turn | attach the trolleys, Start |
| R2-1 | BTU24 / BTU16 | BTU | row 2 eastbound | swap |
| R2-2 | BTU23 / BTU15 | BTU | | swap |
| R2-3 | BTU22 / BTU14 | BTU | | swap |
| R2-4 | BTU21 / BTU13 | BTU | | swap |
| R2-5 | BTU20 / BTU12 | BTU | | swap |
| R2-6 | BTU19 / BTU11 | BTU | | swap |
| R2-7 | BTU18 / BTU10 | BTU | | swap |
| R2-8 | BTU17 / BTU9 | BTU | | swap |
| MW-1 … MW-4 | MRU1 … MRU4 | MRU | MRU line westbound | release |
| ME-4 … ME-1 | MRU4 … MRU1 | MRU | MRU line eastbound | take back |

## 3. Trip flow (AGV-01)

1. The AGV is parked empty in the north bay. Operators enter calls on the display.
2. A trip forms when **two calls of the same group** are pending. Calls of the other group keep
   waiting. A duplicate call for a machine already pending is refused.
3. The HMI shows the formed trip; the operator presses panel Start at the bay (D14). The AGV drives
   **empty** out of the bay, through the bay merge and the U-turn R717, and stops at the attach point.
4. The operator attaches the trolleys and presses panel Start.
5. The AGV drives its group's circuit and serves the calls **in circuit order**, not call order.
6. **BTU trip:** at each called point (R2-1 first, R2-8 last) the operator swaps the trolley, then
   presses panel Start. If both calls are at one shared point (e.g. BTU24 and BTU16), the AGV makes
   **one stop** and both swaps happen there.
7. **MRU trip:**
   - **Westbound:** at each called MW stop the trolley is released, then panel Start.
   - **At the tape end:** the empty AGV pivots.
   - **Eastbound:** it stops at the same machines, in reverse order, to take the trolleys back,
     then panel Start at each.
8. After the last stop the AGV finishes the circuit to the north bay without stopping. The
   operator releases the trolleys there, so the AGV is empty before its next trip's U-turn.

## 4. Circuits and junction orders (AGV-01)

Junctions in travel order. Sides are as seen in the direction of travel. "Merge" means another
tape joins and there is no choice to make.

| Id | Where | Kind |
|---|---|---|
| J-BMERGE | bay merge, projection (1040, 8450) | merge |
| J-N | departure lane × R2434, (2474, 2434) | diverge: R2434 on the right (to the north trunk) |
| J-S | departure lane × R1570, (2474, 1570) | diverge: R1570 on the left (to AGV-02's south trunk) |
| X1 | R2624 × R2434, (1995, 984) | crossing, non-90° |
| X2 | R2624 × departure lane, (2474, 522) | crossing, non-90° |
| J-ML | merge L, (40, 0) | southbound: diverge R1380 (left, tighter) / R2434 (right). Northbound on R2434: merge |
| J-STEM | R2624 joins the stem, (1420, 2624) | merge from the right |
| J-SPLIT | bay split, (1040, 5175) | diverge: north bay on the left, south bay on the right |
| J-LV | middle line × left vertical (site plan) | T: north on the right, south on the left |
| J-R2E | row 2 × trunk (site plan) | eastbound: T (north left, south right). On the trunk: row 2 joins from the west side |
| U-MRU | MRU line west end | tape end: pivot U-turn, empty only |

**C-BTU:** bay → J-BMERGE → U-turn R717 (empty) → ATT → J-N **straight** → J-S **straight** → X2 **straight** →
middle line west → J-LV **right** → row 2 east (R2-1…R2-8) → J-R2E **right** → trunk south →
J-ML **left** → J-STEM → J-SPLIT **left** → north bay.

**C-MRU:** bay → J-BMERGE → U-turn R717 (empty) → ATT → J-N **right** → X1 **straight** → J-ML (merge) → trunk
north → J-R2E **straight** → MRU line west (MW stops) → U-MRU **pivot** → MRU line east (ME stops)
→ trunk south → J-R2E **straight** → J-ML **left** → J-STEM → J-SPLIT **left** → north bay.

J-R2E and J-LV appear only once row 2 is laid (D12). Until then C-MRU meets no tape at J-R2E.

Consequences:

- **Only one branch decision depends on the job: J-N**, and it depends on the group, not the
  machine (MRU → right, BTU → straight). Every other order is fixed for AGV-01.
- **A single standing default cannot drive this site.** C-BTU alone needs straight, straight,
  right, right, left, left. Every diverge needs its own order.
- **J-ML is a wrong-way hazard.** Taking R2434 instead of R1380 southbound drives the AGV
  backwards up a departure curve. It must be an explicit LEFT, never "nearest track".
- **J-S is a territory hazard.** Taking R1570 sends AGV-01 into AGV-02's half.
- **X1 and X2** are oblique crossings. Following straight through them needs a floor check
  (as in MLS bench test B7) before any trip uses them.
- **Both U-turns have a payload guard.**
  - **U-MRU:** the pivot may start only when every called MW release is recorded as done. A
    missed release means a trolley is still attached, so the AGV HOLDs before the tape end for the
    operator. It never pivots.
  - **HOME U-turn R717:** a trip may leave the bay only if the previous trip ended with its
    trolleys detached at home.
  - Without payload detection (D10), "released" and "detached" are records of the stops having been
    served; a rear-lidar check can replace them later.

## 5. What this needs from the code (today vs site)

| Area | Today | Site needs |
|---|---|---|
| Destinations per job | one (`TagTable.destination`) | up to 2 machines per trip, served in circuit order; one shared stop can serve two calls; MRU machines are served twice (release, take back) |
| Branch orders | `branch_default` plus RFID `branch_latch` pairs | an order at every diverge (section 4); the J-N order comes from the trip's group |
| U-turn | triggered by RFID 0180/0190; pivot until the tape is back | triggered by position ahead of the MRU tape end; the pivot is gated on all releases done (payload guard) |
| Missed stop | detected when the U-turn tag is read (`TagTable.note_u_turn`) | BTU: end of section at J-R2E → report. MRU release missed → HOLD before U-MRU. MRU take-back missed → report on arrival at HOME |
| Call queue | none: the job is chosen directly | calls queue on the display; a trip forms at 2 calls of one group |
| HOME | RFID 0010, role `home` | the north bay at the end of the stem (park, empty); a separate attach stop after the U-turn; leaving the bay needs a panel Start (D14) |
| Position source | RFID tags | 16 machine stops + home + attach + 6 junction orders + the U-turn trigger: tape-graph localization (the localization plan, after P0) |

## 6. Open items

1. **AGV-02 (deferred):** the operator gives the row 3 group as BTU1–8 + STU1–4, but the site plan
   shows STU1–6 on row 3. Reconcile when AGV-02 starts.
