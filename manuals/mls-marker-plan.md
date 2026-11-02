# MLS marker plan: read marker codes 1-3 (tracked / LINE mode)

Status: IMPLEMENTED in software 2026-10-09 (steps 0, 2, 3 and 6.1; section 11). The bench
session B0 and B2-B7 is open as a formal record, **but codes 1 and 2 work on the floor: the
speed zones (outer 2, inner 1, 100 mm gap between the strips of a pair) were tested the week of 2026-10-09 (operator,
2026-10-18)**. Code 3 has not been run on the floor. **2026-10-23: markers are always on; `mls.markers_enabled`
is retired (a profile with it is refused), so the bench-switch wording below is history.**
Whether a marker changes a run is the mission's choice (`high_zone`, `marker_run`). Aligned with
the live sensor and re-audited against the whole manual and the code the same day (section 10). Scope is **reading** codes 1-3 reliably and
handing them to the tape engine as events. What each code makes the vehicle do is a separate,
later decision (section 8); this plan wires no behaviour to a marker.

Reference: SICK MLS operating instructions 8021642.1OC7/2024-10-25
(`manuals/obsolete/operating_instructions_mls_en_im0076568.pdf`, the primary reference despite
the folder): §8.1 CANopen objects pp. 28-40, §8.4.5 detection levels pp. 46-49, §8.5 markers
pp. 51-53. Drawing: `documentation/layout-reference/mls-marker-codes-standard.html`.

## 0. Decisions

| # | Decision | Source |
|---|---|---|
| D1 | Sensor MLSE-0200: reads ±100 mm | operator 2026-10-09 |
| D2 | Track tape 30 mm (north up); marker tape **15 mm** wide, 100 mm long, south up | operator 2026-10-09 |
| D3 | **Standard mode, codes 1-3 only**: 1 = 30 mm alone, 2 = 60 mm alone, 3 = 60 + 30 mm. Codes 4-5 need a 90 mm marker with 2.5 mm to spare (centred only), 6-7 are outside the sensor | operator, drawing |
| D4 | FailSafe ON. Without it a skewed pass over 60 + 30 shows code 2 or 1 first; the decoder refuses markers when the sensor does not report FailSafe on | this plan |
| D5 | The sensor decodes; the PC never reconstructs a code from LCPs. Sensor parameters are written by the bench tool only; drive_node only READS and compares. **Every bench write goes through `guard.py`** (a node-10 allow-list, section 3), and `set-variant` is brought under it too | existing rule (read_mls docstring); CLAUDE.md "every SDO write goes through guard.py" |
| D6 | A fourth cue, if needed, is direction (the mirrored layout). Its encoding is unknown until bench B6; until then `direction` is reported as 0 (unknown) and nothing may depend on it | this plan |
| D7 | Markers are not in the safety path. ~~Until section 8 is decided, they change nothing about motion~~ **2026-10-09 (operator): the markers decide speed, RFID decides where to stop.** The mission's `high_zone` outer/inner codes (line-a: outer 2, inner 1) replace the RFID zone tags; no marker changes the run's state | safety invariants; operator |

**Principle (same as the speed plan): a missed marker must be harmless.** Any later use of a
code must be one where not seeing it is safe (the vehicle carries on as if no marker were
laid) or is cross-checked against RFID. A marker never grants anything on its own.

## 1. Does this need code?

Yes, in three layers. The sensor already decodes the code; today nothing turns it on, checks
it, or uses it.

| Layer | Today | Needed |
|---|---|---|
| Sensor configuration | 2028h at factory default (markers off, so byte 7 bits 3-7 are always 0: confirmed by the 2026-10-08 bag, 7,846 samples, marker 0) | one bench write (section 3), and a read-only check at every start |
| Reading | `read_mls.decode_marker` decodes byte 7; drive_node publishes `LineTrack.marker`/`marker_intro` | turn per-sample codes into **events** at sample rate (a FailSafe code may last a single 10 ms frame), publish them reliably |
| Engine | `amr_line` ignores the fields | an intake like the RFID encounter stream; events logged and shown, no action |

## 2. MLS parameters (communication chapter, checked against the live sensor 2026-10-09)

Read from node 10 with `amr.service` stopped (`read_mls snapshot` plus read-only SDO uploads of
every object below; nothing written). **Bold** = written by the bench tool `set-markers`; the
rest stays as it is and is only verified.

**Identity:** order number 2019h = **1104468 = MLS 200 mm** (D1 confirmed), vendor 0x01000056,
product 0x1100, revision **7** (the manual documents firmware V5 features; all objects below
answered except where noted), serial 0x018E8F7E. 1008h/100Ah are segmented strings our
expedited `sdo_read` cannot fetch: the order number is the identity check.

| Object | Name | Manual default | **Live 2026-10-09** | Target | Why |
|---|---|---|---|---|---|
| **2028h:01** | Use markers | 0 | 0 | **1** | "1 = detected markers displayed in TPDO1". Live 0 explains the all-zero marker field in the 2026-10-08 bag |
| **2028h:02** | Marker style | 0 | 0 | **1** (SICK standard) | D3. 2 = extended buys nothing: the usable positions are still 30 and 60 |
| **2028h:03** | FailSafe mode | 0 | 0 | **1** | D4. The code is output once the whole marker frame has passed |
| **202Dh:05** | Tape polarity | 0 (both) | 0 | **1** (north track, south markers), **only after B0** | p. 49 "increases reliability"; also stops auto-detection being fooled at power-up over a marker (p. 52). **Risk:** with 1 the sensor no longer sees south-up track at all (p. 49 "only tracks with north polarity are detected"), and p. 20 offers south-up tape as a ZONE marker. Any south-up piece on the route becomes a gap in the line (track hold). B0 proves the whole route is north first; markers work with 0 too, so this is a reliability option, not a prerequisite |
| **2029h** | Lock teach | 0 | 0 | **1** | the capacitive keypad on the front end cap can invert the range, teach a zero point or run an offset calibration (pp. 9, 41-42); markers depend on side and offset, and p. 44 warns the offset calibration must not run over tape. Consequence: a future commissioning step that uses the keypad must first write 2029h = 0 (RUNBOOK note) |
| 202Dh:01 | First detection level | 30 % | 30 | unchanged | a 15 mm marker's peak must clear 30 % of the track's (B3). Lower only if B3 fails and B7 then shows no false markers |
| 202Dh:02 | Last detection level | 30 % | 30 | unchanged | |
| 202Dh:03 | Averaging magnetic | 1 | 1 | unchanged | more averaging delays the output (p. 49); at 1 a 100 mm marker spans ~12 samples at 0.8 m/s |
| 202Dh:06 | Filter strength (improved diverter detection, p. 51) | - | **SDO abort: object absent** | none | the manual mentions it; this unit does not have it. Noted so nobody tries to tune it |
| 2025h | Min. level | 200 | **300** (not default: set before this work; the 2026-10-02 calibration suggested 792) | unchanged | B3 records whether it also gates markers. Raising it towards 792 could hide 15 mm markers: re-run B3 after any change |
| 2006h:01 | Variant TPDO1 | 3 | 3 (set 2026-10-07) | 3 | marker decoding is independent of it. **Found:** drive_node's `mls_track_variant` parameter still expects 0, so every start logs a mismatch. Fixed in step 2 (moved to the profile) |
| 2006h:02 | Variant MEMS (IMU) | 1 | 1 | unchanged | |
| 2026h | Offset (zero point) | 0 | 0 | unchanged | |
| 2027h | Sensor flipped | 0 | 0 | unchanged | p. 52: markers are decoded from positions AFTER inversion, so it decides which physical side is "Marker 1" (the far marker of the positive direction). The manual does not tie that side to the cable outlet: B6 records it |
| 1800h | TPDO1 COB-ID / type / timer | 0x18A / 0xFF / 10 ms | 0x18A enabled / 0xFF / 10 ms | unchanged | markers ride in TPDO1 byte 7; no other TPDO needed. 0.8 m/s × 10 ms = 8 mm per sample |
| 1801h-1806h | TPDO2-7 | disabled | all disabled (1801h timer 10) | unchanged | the IMU is read by SDO (read_imu), so no TPDO competes for the 4-active limit |
| 2021h:04 | #LCP (SDO) | | read only | | "tracks **and markers**": the SDO fallback carries the marker bits, but at ~10 Hz it can miss a one-frame code, so SDO samples never produce events |
| 1010h:01 | Store | | reads **2** | not used | it refused a store write on 2026-10-07 (abort 06010002) yet kept the value after an NMT reset. 2 likely means "saves autonomously" (CiA 301 bit 1); B2 proves persistence across a power cycle |
| 1017h | Producer heartbeat | 0 | **SDO abort** | none | the MLS sends no heartbeat; stream liveness stays the TPDO rate check |
| 1F80h | NMT start-up | 8 | 8 | unchanged | the sensor waits Pre-operational; drive_node sends NMT Start as today |
| 202Fh:02 | Bitmap mode | 0 | 0 | unchanged | |
| 202Ah | Set param. to default | | never | never | wipes all of the above; listed so nobody runs it as a fix |

Live measurement during the read: no tape under the sensor (field level 152 < min. level 300,
#LCP 0), so nothing about marker reception could be observed; that is bench work (section 4).

**Detection-level caution (p. 47).** A north track has a positive main peak with negative
overshoots beside it; a south marker is also a negative peak. The negative detection level
must sit below the overshoots or the track's own edge reads as a marker. With 30 mm tape the
overshoots lie close to the 30 mm position (code 1). B4 and B7 are the tests for this.

## 3. Step 0 - bench tooling (`agv_core/drivers/canbus/read_mls.py`)

No ROS. All behind the existing owner lock: refused while `amr.service` holds can0.

| Item | Change |
|---|---|
| `decode_marker` | also return `raw` (the 5-bit field, 0-31) next to `intro`/`code`. The manual types the field INT5 (signed) and says bit 0 = intro, bits 1-4 = code 1-15: the raw value is what B6 needs to settle direction. **Breaks** `tests/test_mls.py` "byte 6 bit 3 is the marker intro" (compares the whole dict): update that check to include `raw`; the check count is unchanged |
| `snapshot` | add 2028h:03, 202Dh:05, 2029h, 202Dh:01/02; print the order number as the sensor model |
| `markers` (new, read-only) | stream TPDO1 (needs `--nmt` as `stream` does) and print one line per CHANGE of byte 7 bits 3-7 or status bit 6 (reading code): time, raw bits as binary, intro, code, reading-code bit, LCP2, #LCP, polarity. Ends with a per-code count, the number of "reading code" episodes that ended with no code, and the polarity seen. This is the instrument for B0 and B4-B7. Line levels are SDO objects and are not read while streaming (one client on node 10's SDO channel at a time) |
| `calibrate --step marker` (new) | SDO-sample the line levels (2021h:0B-0D) with a marker under the sensor beside the track; report marker peak / track peak against 202Dh:01 (30 %) |
| `set-markers` (new, the second write) | same shape as `set-variant`: dry run unless `--go`; writes 2028h:01 = 1, 2028h:02 = 1, 2028h:03 = 1, 2029h = 1, and 202Dh:05 = 1 only with `--polarity-lock` (after B0); NMT reset of node 10 only (0x81); reads every value back and prints now/want/after. Exit non-zero on any mismatch. No 1010h store (guard forbids it and this sensor refuses it anyway) |
| `guard.py` (safety-listed file) | today the bench path `drive_forward.sdo_write` bypasses guard: `set-variant` wrote 2006h:01 and attempted 1010h (FORBIDDEN "store parameters") with no check. Add `check_sensor(index, sub)`: the MLS allow-list {2006h:01, 2028h:01-03, 202Dh:05, 2029h}, FORBIDDEN still applied first, everything else refused. `read_mls` writes only through a wrapper that calls it; `set-variant` drops its 1010h store. Node-specific by construction: the drive `check()` is untouched, so no drive object becomes writable. **Failure mode:** a wrong entry would let the bench tool reconfigure the MLS (track reading), never a drive; **recovery:** `snapshot`, then `set-variant`/`set-markers` back, or the values in section 2. Test asserts the exact set and that 1010h, 202Ah, 202Bh, 202Ch and every drive object are refused |

Tests (`tests/test_mls.py`, `tests/test_guard*.py`, EXPECTED_CHECKS +N): raw marker bits for
0x08|(9<<4) and a negative-looking 0xF8; `set-markers` dry run lists exactly the writes and
sends no frame (fake bus); `--polarity-lock` adds 202Dh:05; the guard allow-list as above;
snapshot formatting with the new rows. **EXPECTED_CHECKS:** `tests/run_all.py` has uncommitted
edits from the RFID work in progress (git status 2026-10-09): rebase the count on whatever is
committed when this lands, never on a guess.

## 4. Step 1 - bench session (operator, vehicle on stands or slow on the floor)

Results go in a short note appended to this file. Steps 2-3 can be coded in parallel, but
nothing is enabled in the profile until B0-B7 pass.

| # | Test | Pass |
|---|---|---|
| B0 | Before any write: one full lap of the site route with the current settings, `markers` tool or a run bag recording polarity | polarity is `north` everywhere (no south-up zone tape anywhere). Only then `--polarity-lock` |
| B1 | `snapshot` | DONE 2026-10-09: order number 1104468 (200 mm), 2006h:01 = 3, 2027h = 0, marker objects all 0 (section 2). `set-markers` dry run lists the 4 writes and changed nothing |
| B2 | `set-markers --go`, power-cycle the sensor, `snapshot` | all five values survive a power cycle (not only an NMT reset) |
| B3 | `marker-level --baseline floor`, sensor over the track with a 15 mm strip at 30 and then 60 mm (section 11: the bare-floor profile is already saved) | marker peak ≥ 1.5 × the 30 % level (≥ 45 % of the track peak: margin for wear and height) |
| B4 | 15 mm strip at 30 mm under a stationary sensor | LCP2 moves ≤ 2 mm vs no strip; #LCP stays 2 (the marker is not reported as a diverter) |
| B5 | codes 1, 2, 3 laid on a straight; 20 passes each at 0.3, 0.5 (NORMAL) and 0.85 m/s (HIGH, tracked-speed plan), centred and 25 mm off-centre | 100 % read, correct code, no other code ever shown (FailSafe); zero "reading code without a code" episodes. Record where along the marker the code appears (FailSafe reports after the frame has passed: ~100 mm + filter t90 35 ms late) |
| B6 | each code driven over both ways; and laid mirrored | record raw bits per case. Decides D6: does direction appear (sign bit, other code, nothing)? Does a lone marker on the Marker 2 side decode at all? |
| B7 | full lap of the site route with markers enabled and **no** markers laid; then over every diverter and crossing | zero marker events (no false reads from track overshoot, diverters, floor steel) |

## 5. Step 2 - events at the bus owner (`amr_base`, `amr_interfaces`, profile)

### 5.1 Profile (`profiles/agv-01.json`, `agv_core/config.py`)

The schema has no generic nested blocks (only `drivers.ramp` is special-cased) and its `list`
type is a list of STRINGS, so the block is flat and the codes get their own reader (like
`zone_bytes`). A new section is required in every profile (`_parse` refuses a missing
top-level key); agv-01 is the only profile today.

```json
"mls": {
  "variant": 3,
  "markers_enabled": false,
  "marker_codes": [1, 2, 3],
  "polarity_lock": false,
  "teach_lock": true
}
```

| Key | Config name | Validation (refused at load) |
|---|---|---|
| `mls.variant` | `MLS_VARIANT` | int 0-7. Replaces drive_node's `mls_track_variant` parameter (default 0, wrong since 2026-10-07: every start logs a mismatch today) |
| `mls.markers_enabled` | `MLS_MARKERS` | bool. The bench switch: `true` only during a bench session or after B0-B7 pass, otherwise `false` |
| `mls.marker_codes` | `MLS_MARKER_CODES` | own reader: non-empty list of unique ints, each 1-7 (standard mode), no bools |
| `mls.polarity_lock` | `MLS_POLARITY_LOCK` | bool: whether the check expects 202Dh:05 = 1. `false` until B0 |
| `mls.teach_lock` | `MLS_TEACH_LOCK` | bool: whether the check expects 2029h = 1 |

Refusal tests in `tests/test_config.py` for each (variant 8, codes empty/duplicate/0/8/"2"/true,
non-bool flags, missing block, unknown key in the block).

### 5.2 Configuration check (`mls_track.py`, read-only, never blocking)

**Not in `start()`.** `start()` reads synchronously with the 0.4 s SDO timeout, and it also runs
from `_rediscover()` inside `poll()` on the bus thread while the drives may be moving. Five more
blocking reads there could hold the wheel loop for up to 2 s if the sensor stops answering. So
the check is a cycle like the SDO track poll: at most ONE read per bus tick, 0.05 s timeout
(`PROBE_TIMEOUT_S`), reading 2028h:01-03, 202Dh:05, 2029h, and only while the MLS answers.

When it runs: after every `start()`; again whenever the TPDO stream comes back after a gap
(`_nmt_start` path: the sensor rebooted or was reset); and every 60 s. A sensor reboot does NOT
pass through `_rediscover()` (that only follows an incomplete start), so "re-check on
rediscovery" alone would miss a sensor swapped or reset to defaults.

Result, when markers are enabled: any value different from the profile, or unread, makes
`markers_ok = False`, warns once naming the value and the fix (`read_mls set-markers --go`),
and suppresses every marker event until a later cycle reads them right. Disabled in the
profile: no reads, `markers_ok = False` silently. Diagnostic rows: `markers` (ok / off /
misconfigured: ...), `marker_events`, `marker_rejects`, `marker_aborts`.

Tests (`test_mls_track.py`, fake link): one read per poll; a timeout costs one probe timeout,
not five; a wrong 2028h:03 gives misconfigured; a stream gap re-arms the check; disabled reads
nothing.

### 5.3 Pure event detector (`amr_base/marker_events.py`, clock-fed)

Fed every decoded sample in the bus thread, at TPDO rate (not the follower's 50 Hz, which
could miss a one-frame code):

- Only `source == "tpdo"` samples count. An SDO sample resets nothing and emits nothing.
- An event is the code changing from 0 to c, or from c to c' (c' ≠ 0). A code that stays is
  one event. FailSafe makes the reported code final, so no extra debounce: one frame is enough.
- A code not in `MLS_MARKER_CODES` is a **reject** (counted, warned once per code per minute),
  never an event.
- Each event carries: `seq` (per process), `generation` (bumped when the MLS is rediscovered
  or the bus owner restarts), `code`, `raw`, `direction` (0 until D6 is decided), `lcp2_mm`,
  `nlcp`, `line_good`, `t_mono`.
- Status bit 6 "reading code" (p. 38) going 1 → 0 with no code reported is an **abort**:
  counted as `marker_aborts`. A marker the sensor began to read but could not decode is the
  early warning of a worn, off-centre or too-fast marker, before a full miss.
- Not `markers_ok` → nothing.
- Runs in the bus thread inside the existing `on_sample` callback and publishes with the same
  `_safe_publish` as `/amr/line_track`: no new thread, no lock.

Tests (`amr_base/test/test_marker_events.py`): 0→1→0 one event; 0→2→2→0 one event; 0→3→2
two events (only possible without FailSafe: still reported); code 5 rejected; SDO sample
ignored; seq monotonic; generation bump; disabled → no output.

### 5.4 Transport (`amr_interfaces`)

New `MarkerEvent.msg` on `/amr/line_marker`, RELIABLE, depth 50, volatile (a replayed old
marker must never look new):

```
builtin_interfaces/Time stamp
uint32 seq
uint32 generation
uint8 code
uint8 raw                # byte 7 bits 3-7, for diagnosis and B6
int8 direction           # +1 / -1 once D6 is decided; 0 = unknown
int16 lcp2_mm
uint8 nlcp
bool line_good
bool markers_ok          # also sent as a 1 Hz heartbeat with code 0
bool heartbeat           # true: no marker passed, this only carries the status below
string status            # "ok" | "off" (profile) | "misconfigured: 2028h:03 = 0" | "no stream (sdo)"
uint32 rejects           # codes outside mls.marker_codes since start
uint32 aborts            # "reading code" episodes that ended with no code (5.3)
```

A 1 Hz heartbeat (code 0) carries `markers_ok` and the current seq, so a consumer can tell
"no markers passed" from "no marker stream". `LineTrack` keeps its per-sample `marker` field.
Needs `colcon build` (amr_interfaces).

## 6. Step 3 - engine intake (`amr_line`)

| Item | Change |
|---|---|
| `line_follow_node._on_marker` | mirror `_on_rfid`: a deque of `(seq, code, direction)`, reset on a generation change; a `_markers(now)` snapshot `{"ok", "seq", "generation", "encounters"}` with `ok` false when the heartbeat is older than 2.5 s |
| `job.Inputs` | new field `markers: dict \| None = None` at the END (frozen dataclass, defaults after the required fields): every existing caller and `test/harness.py` stay valid |
| `amr_line/marker_reader.py` (pure) | cursor over the snapshot like the RFID scan: returns the new events since the last tick, counts seq gaps as `missed` |
| `FollowJob` | drains events every tick in every state; emits an operator event `marker 2 at +3 mm` (info) and keeps `last_marker` / `marker_count`. **No state change, no speed change** |
| `LineState.msg` (append) | `uint8 last_marker`, `uint32 marker_count`, `bool markers_ok` |
| HMI | Home: a **Marker** tile beside the RFID tile (6.1). No new endpoint, no command |
| `tools/run_log.sh` | add `line_marker` to the `TOPICS` regex, or run bags will not contain the events |
| `tools/bag_report.py` | add `marker`, `marker_intro` to `line_track.csv`; new `line_marker.csv`. The file has uncommitted edits from the RFID work (git status 2026-10-09): change it after those land |
| `amr_web/adapter.py` | `_on_line` already converts LineState generically (`_msg_to_dict`), so the new fields arrive without adapter code; the LINE tile reads them in `amr.js`. The existing `_on_track` copy (5 Hz) keeps showing the instantaneous `marker` |
| Sim (`amr_sim/tape_plant.py`) | optional: a marker list along the tape so the unified sim can emit codes. Mechanism only |

### 6.1 Home page: marker code beside the RFID tag

Today Home shows three live tiles (2026-10-08, `templates/home.html` `.home-live`): **Speed** and
**RFID tag** side by side, **Line position** full width under them. The marker tile goes in the
first row, right of RFID, so the RFID tag and the marker code read as a pair:

| Row | Tiles |
|---|---|
| 1 | Speed · RFID tag · **Marker** |
| 2 | Line position (full width, unchanged) |

The tiles stay the same height, so Home still fits at UI scale `l` (1280×720) without
scrolling. A row of three at ~1280 px leaves each tile about 380 px; the longest value
("0.50 m/s") is about 210 px at the tile's 2.7 rem mono, so it fits. Below 820 px the grid
already stacks to one column. The tile is rendered only when `mls.markers_enabled` is true
(server-side, from the profile), so Home looks exactly as it does today until the bench passes.

**What the tile shows**, mirroring the RFID tile's rules (`linetrack.js rfidReading`):

| Situation | Big value | Line under it | Style |
|---|---|---|---|
| Marker read within the last 2 s (RFID's `RF_HOLD_S`) | the code, e.g. `2` | `read now · +3 mm` (LCP2 at the read) | `ok` (filled, like RFID "reading now") |
| Stream up, no recent read | `––` | `last 2 · 14 s ago · 37 this run`, or `no marker read yet` | plain |
| Aborted reads since start > 0 | unchanged | ` · 3 aborted` appended | `warn` |
| `status` misconfigured | `––` | `sensor not set up: read_mls set-markers` | `warn` |
| Heartbeat older than 2.5 s, or no `/amr/line_marker` | `––` | `no marker data` | `off` |
| `status` "no stream (sdo)" | `––` | `MLS on SDO fallback: markers not read` | `off` |

Direction is not shown until D6 is decided (B6). It will appear as `2 ←` / `2 →`.

**Code:**

| File | Change |
|---|---|
| `amr_web/adapter.py` | subscribe `/amr/line_marker` with the RFID QoS (`RFID_QOS`: reliable, depth 50, volatile); keep the last heartbeat and the last 8 events (deque, like `_rfid_tags`); `_marker_state(now)` → `{"link": {status, markers_ok, rejects, aborts, age_s}, "events": [{code, raw, direction, lcp2_mm, age_s}]}`; add `"markers"` to `state()` |
| `amr_web/static/linetrack.js` | `markerReading(m)` beside `rfidReading`: the same up/last/reading rule, shared, not copied |
| `amr_web/templates/home.html` | `<div class="live" id="live-marker"><span>Marker</span><b>––</b><i>no marker data</i></div>` after `live-rfid`, inside `{% if markers_enabled %}`; `liveTiles()` fills it |
| `amr_web/static/amr.css` | `.home-live.three { grid-template-columns:repeat(3, minmax(0, 1fr)); }`, set on the container when the tile is present. `#live-line` keeps `grid-column:1 / -1`. No other style |
| `amr_web/server.py` | resolve `markers_enabled` the way `product` is resolved today: from `agv_core.config` (`MLS_MARKERS`) at app creation, `False` when no profile loads (dev box), and injectable as an app-factory argument so tests set it; render it into `home.html` only (not the context processor, which every page pays for) |

Tests (`amr_web/test/test_operator_ux.py`):
- `test_home_shows_speed_rfid_and_line_tiles...` stays: with markers disabled there is no `live-marker` and no `three` class.
- A new test with markers enabled checks that `live-marker` follows `live-rfid` and the `three` class is set.
- An adapter test feeds a heartbeat and two events and checks the `markers` state shape and order (newest first).

Tests: `amr_line/test/test_marker_reader.py` (cursor, gaps, generation reset);
`test_line_authority`-style job test proving a marker event changes neither state nor speed;
node contract test that `/amr/line_marker` QoS is reliable/volatile.

## 7. Failure modes and recovery

| Failure | Effect | Recovery |
|---|---|---|
| Sensor at factory defaults (replaced, 202Ah, power fault) | start check: `markers_ok` false, warning names the fix; no events; RFID unaffected | `read_mls set-markers --go` with the service stopped |
| FailSafe off | refused as misconfigured (D4), not debounced around | same |
| Marker missed (worn, off-centre > 32 mm, too fast) | no event | by the principle, harmless; `marker_count` per lap on the Monitor shows it |
| False marker (track edge overshoot, diverter, floor steel) | an event with a code in 1-3 | B4/B7 catch it before enabling; codes outside 1-3 are rejects, counted |
| LineTrack samples dropped | none: events are made in drive_node before transport | |
| `/amr/line_marker` message lost | seq gap counted as `missed` | |
| SDO fallback (no TPDO1) | no events; `markers_ok` stays true but the heartbeat reports source sdo | the existing `rate` hold already stops the follower |
| Bus owner restart / MLS rediscovered | generation bump; the engine drops its cursor | automatic |
| Direction misread before B6 | cannot happen: `direction` is 0 until decided | |

## 8. Later (not this plan)

- What codes 1, 2, 3 mean (zone, station confirmation, branch warning) and how a mission
  names them: needs operator decisions and a mission schema change like the tag table.
- Cross-checking RFID with markers (redundant station detection) is the most likely first use:
  it satisfies the principle because a miss on either side only raises a warning.
- Direction decoding once B6 has run; mirrored layouts as cues 4-6.

## 9. Order of work

1. Step 0 (bench tooling) - offline tests only.
2. Bench B0-B7 with the operator. Stop if B3, B4 or B7 fails: report and decide (narrower
   detection level, wider spacing, or no markers).
3. Step 2, then step 3 - profile `mls.markers_enabled` is on only for bench sessions until the
   bench note says PASS.
4. Enable on the vehicle, one lap with markers laid; the operator's observation is the
   acceptance.

## 10. Audit 2026-10-09 (whole manual + current code)

Manual read in full (pp. 1-68). What changed in this plan because of it:

| # | Finding | Source | Change |
|---|---|---|---|
| A1 | The bench write path bypasses `guard.py`; `set-variant` already wrote 2006h:01 and attempted 1010h (on guard's FORBIDDEN list) unchecked | `drive_forward.sdo_write`, CLAUDE.md invariant | D5; `guard.check_sensor` allow-list in step 0 with test, failure mode and recovery; `set-variant` loses its 1010h store |
| A2 | Polarity lock makes south-up track invisible; the manual offers south-up tape as a zone marker | pp. 20, 49 | 202Dh:05 only after B0 (full-lap polarity); profile default `false`; `--polarity-lock` opt-in |
| A3 | A blocking config read in `start()` would run on the bus thread mid-motion via `_rediscover()` (0.4 s timeout per read) | `mls_track.py` start/poll, `canopen.read` | 5.2 rewritten: one non-blocking read per tick, 0.05 s timeout |
| A4 | A sensor reboot never reaches `_rediscover()`; the old wording claimed it was caught | `mls_track.py` | re-check after a TPDO gap and every 60 s |
| A5 | Adding `raw` to `decode_marker` breaks an exact-dict check | `tests/test_mls.py` | noted in step 0 |
| A6 | The schema has no nested blocks and `list` means strings | `config._coerce`, `_parse` | 5.1 flattened; own int-list reader; missing-block refusal |
| A7 | `mls_track_variant` expects 0 against the live 3 | drive_node param, live read | moved to `mls.variant` |
| A8 | Run bags would not contain the new topic | `tools/run_log.sh` TOPICS | added to step 3 |
| A9 | HIGH speed is 0.85 m/s, not 0.8 | tracked-speed plan D1 | B5 tests 0.85 |
| A10 | FailSafe reports after the marker frame has passed, plus the filter delay (t90 35 ms at averaging 1, p. 43) | pp. 43, 53 | B5 records the lag; any later position-based use must allow for it |
| A11 | "Reading code" status bit (p. 38) can expose partial reads | p. 38 | `marker_aborts` counter |
| A12 | Uncommitted RFID work touches `tests/run_all.py` and `tools/bag_report.py` | git status | EXPECTED_CHECKS and bag_report changes wait for it |
| A13 | The teach pad is on the front end cap (p. 9); offset calibration must not run over tape (p. 44) | pp. 9, 44 | teach lock kept; RUNBOOK note to unlock deliberately |

Checked and unaffected:

- **TPDO1 layout:** byte 7 is the same in every variant, Standard or Combi (table 7), so the 2006h:01 = 3 setting is untouched. No TPDO is added, and with TPDO2-7 disabled nothing nears the 4-active limit (p. 36).
- **Persistence:** 2028h needs no restart according to the manual; only 2006h does (p. 31). set-markers resets node 10 anyway and B2 power-cycles the sensor.
- **Line following:** the track reading (LCP2, #LCP, line_good) gets no new consumer. `track.py` and the branch ladder are unchanged, and B4/B7 prove that markers don't disturb the LCPs.
- **Drives:** guard's drive `check()` and its tests are unchanged.
- **Sim:** sim and unified-sim tests have no `/amr/line_marker`, so `markers_ok` is false and nothing in the engine acts on it.
- **Web:** no new endpoint, no command and no new engineer action. The Marker tile on Home (6.1) is display-only, appears only once `mls.markers_enabled` is true, and keeps Home within 1280×720 at scale `l`.
- **Profile load:** with `markers_enabled` false a profile with the new block loads and runs exactly as today. With it true and the sensor not yet written, the check reports misconfigured: no marker events, Home shows "sensor not set up", motion unaffected.

## 11. Implementation notes (2026-10-09)

Code as planned unless listed here.

| Item | Where | Note |
|---|---|---|
| Guard | `guard.SENSOR_ALLOWED`, `check_sensor`, `is_sensor_allowed`; `read_mls._guarded_write` | `set-variant` no longer tries the 1010h store. `test_canmon` pins that read_mls holds exactly one `sdo_write(` call |
| Bench tool | `read_mls markers`, `marker-level`, `set-markers` | **`marker-level` replaces `calibrate --step marker`.** A marker is not a track, so the line levels (2021h:0B-0D) may never show it; the tool reads the raw Hall elements (2000h) instead |
| Hall elements | `HALL_ELEMENTS` | **Found on the vehicle:** 2000h answers all 168 sub-indices on the MLSE-0200, 36 real values then 132 zeros. The count comes from the manual's table (p. 44), not from reading until an abort |
| Standing field | `--save floor` / `--baseline floor` | **Found on the vehicle:** with no tape the mounted sensor reads about +150 falling to -30 digits across its length (manual p. 45, fig. 20). Subtracting a bare-floor profile leaves ±20 digits. Saved 2026-10-09: `~/.amr/mls_cal/hall-floor.json`. Re-record it after moving or remounting the sensor |
| No-track gate | `hall_profile(min_level=2025h)` | below the sensor's own min. level (300) no peak is judged: a percentage of noise is noise |
| Detector | `amr_base/marker_events.py` | an abort is counted only after `ABORT_GRACE_FRAMES` (5, 50 ms), because FailSafe may report the code a frame after the reading bit drops. A code already showing when trust returns is not reported |
| Config check | `mls_track.py` | as 5.2. A failed read is `unverified: ...` and retried after 5 s |
| drive_node | `marker_expectation()` | a test asserts it equals `read_mls.MARKER_SETTINGS`, so the checker and the writer cannot drift. `mls_track_variant` now defaults to `mls.variant` (3) |
| Messages | `MarkerEvent.msg`; `LineState` + `markers_ok`, `last_marker`, `marker_count`, `marker_missed` | as 5.4, with `heartbeat`, `status`, `rejects`, `aborts` |
| Engine | `amr_line/marker_reader.py`; `FollowJob.markers`, `drain_marker_events`, `marker_snapshot` | a test proves a marker changes neither the state nor the wheel command |
| Events | new catalogue code `LINE_MARKER` (info), RUNBOOK section 4 regenerated | the regeneration also brought in the 2026-10-08 `DRIVES_NOT_READY` text |
| Home | `.home-live.three`, `#live-marker`, `linetrack.js markerReading` | the code is shown for `MK_HOLD_S` = 2 s (the RFID tile's `RF_HOLD_S` is 1 s, not 2 as 6.1 said). Checked at 1280×720, scale `l`: one row of three tiles, no scroll |
| Recorder | `tools/run_log.sh` TOPICS + `line_marker` | edited on the vehicle only: `.gitignore` hid `*.sh` there until fix plan 001 A3; committing it is B1 |
| Report | `tools/bag_report.py` | (fix plan 001 A2) `marker`, `marker_intro` in `line_track.csv`; `line_marker.csv`; an MLS MARKERS section and `MARKER` timeline rows. Proven on a real bag in B3 |

Tests:

| Suite | Result |
|---|---|
| `tests/run_all.py` | 702 checks (+31) |
| amr_base | 284 |
| amr_line | 173 |
| amr_web | 295 |
| amr_mission, amr_navigation, amr_localization, amr_maps, amr_sim, amr_bringup | all pass |

The unified sim fails at `can_confirm` (trackless localisation). The same failure occurs on a clean HEAD worktree, so it predates this work. Fixed 2026-10-17 (fix plan 001 B5: near-zero negative AMCL covariance); the unified sim passes. `tests/test_alarms.py` flags `PP_VETOED`, which drive_node emits with no catalogue row; that is also already the case in HEAD. Fixed in `7fc226e` (fix plan 001 A1).

Read-only on the sensor, 2026-10-09:

- `snapshot` and the `set-markers` dry run: four writes pending, nothing changed.
- `marker-level`: 36 elements, no track.
- `markers --nmt` for 10 s: 1,005 frames, no codes.
