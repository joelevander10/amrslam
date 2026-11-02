# MLS diagnostics plan: event detection, temperature, tape health

Status 2026-10-18: M0-M3 BUILT (uncommitted), offline tests green. Bench step E0 OPEN:
it decides `mls.event_clear` (profile, -1 = no runtime write until E0.5 passes) and
confirms the E5 gate. Nothing has run on the vehicle yet.

| Built | Where |
|---|---|
| M0 | `read_mls`: `decode_event`, `decode_temp`, snapshot section [4], `clear-event --value 0/1 [--go]`; `guard.SENSOR_ALLOWED` += 2080h:01 |
| M1 | `mls_track`: one SDO job queue (marker check, event cause, temperature); `on_event`; diagnostic keys `events_*`, `event_clear`, `track_level_min`, `temp_c`, `temp_range_c`. `drive_node`: `MLS_FIELD_WEAK` (INFO, `RateLimit` 2 s), `MLS_TEMP` (`_edge("temperature")`) |
| M2 | `mls_track` writes 2080h:01 through `DriveLink.write_sensor` (guard.check_sensor, 50 ms) when `mls.event_clear` is 0 or 1 |
| M3 | `amr_line/tape_health.py` fed by `line_follow_node`; run file `~/.amr/reports/tape_health/<date>.jsonl` at DONE/FAULT/IDLE; `tools/tape_health_report.py` |
Source: SICK MLS operating instructions 8021642.1OC7/2024-10-25, §8.4 (track levels),
§8.7 (temperature), §8.8 (event detection), pp. 43, 57-58
(manuals/obsolete/operating_instructions_mls_en_im0076568.pdf). Sensor: MLSE-0200,
revision 7, firmware V5 features answer (mls-marker-plan §2); the MLS gyro already runs on them.

Nothing in this plan is in the safety path or changes motion. It is maintenance
information: where the tape is weak, and when the sensor says its own reading is out of spec.

## 1. Why

| Gap today | Consequence |
|---|---|
| `LineTrack.event_flag` (status bit 7) is published ([drive_node.py:981](../amr_ws/src/amr_base/amr_base/drive_node.py)) and read by nothing | The sensor's own statement "the unfiltered LCP is no longer within data-sheet spec" (§8.8, magnetic-field event) is thrown away |
| 2080h (event source and cause) is never read | Even a logged flag could not say field or temperature |
| `track_level` (0-7, every TPDO1 frame) feeds only `track.py` | Worn, lifted or cut tape is found when the vehicle loses the line, not before |
| 2070h:01 (temperature) is never read | Low value alone; needed to tell a temperature event from a field event |

## 2. Decisions

| # | Decision | Reason |
|---|---|---|
| E1 | Field events are **history**, never a standing alarm | A weak spot is a maintenance item. A standing WARN would become the top row on Home ("Nothing to do" would disappear) for something the operator cannot fix |
| E2 | **Clear the latched flag at runtime** by writing 2080h:01, through `guard.check_sensor`, only after the cause has been read | §8.8: the flag stays set until 2080h:01 is written. Without the clear, the first dip after power-up hides every later one. **Breaks** the rule "drive_node writes nothing to the MLS" ([mls_track.py:3](../amr_ws/src/amr_base/amr_base/mls_track.py)): one status object only, no configuration. Fallback if rejected: M1 alone, reports the first event per power-up |
| E3 | Tape health uses `track_level` from TPDO1, not the 2021h:0B-0D line levels | `track_level` arrives at TPDO rate with no extra bus traffic; an SDO level read once per second gives one sample per 0.85 m at HIGH. 2021h levels stay a bench tool (`read_mls`) |
| E4 | All new SDO reads share **one queue** with the marker check: at most one read per `poll()`, TPDO mode only, `PROBE_TIMEOUT_S` | Keeps the invariant in [mls_track.py:27](../amr_ws/src/amr_base/amr_base/mls_track.py): the wheel loop is never held up |
| E5 | Field events are counted only while a track is expected (previous sample `nlcp > 0`); E0 decides the exact gate | If the sensor raises the event every time it leaves the tape (U-turn, end of tape, gaps), an ungated counter is noise |

## 3. E0: bench (service stopped, read-only except the clear test)

The objects are named in the manual; their behaviour is not fully documented. E0 settles that before any code depends on it.

| Step | Do | Record |
|---|---|---|
| E0.1 | `read_mls snapshot` extended (M0) | 2080h:01-05, 2070h:01/06/07 answer; types; default thresholds |
| E0.2 | `markers` stream, sensor over tape, then lift it slowly (spacer blocks) | Height at which event bit 7 sets vs `line_good` drop vs `track_level` per step. Is the "critical strength" separate from `line_good`? |
| E0.3 | Sensor over bare floor; over a tape end; over a diverter | Does the event fire with no tape at all? (decides E5) |
| E0.4 | After E0.2, return to normal height | Bit 7 stays set (latched)? 2080h:04 value? |
| E0.5 | Write 2080h:01 (value 0 first, then 1 if 0 does nothing), via `read_mls clear-event --go` | Which value clears it; does it re-latch on the next dip; survives NMT reset / power cycle? |
| E0.6 | Ten passes at 0.3 / 0.5 / 0.85 m/s over a known weak or 3 mm-lifted piece | `track_level` minimum seen at each speed: is one 10 ms frame enough to see a dip? |

PASS for M2: E0.4 latched and E0.5 a value that clears it. If 2080h:01 is not writable,
M2 is dropped and E2 falls back to M1 only.

## 4. Work

### M0: bench tool (agv_core, offline-testable)

| File | Change |
|---|---|
| `profiles/agv-01.json`, `agv_core/config.py` | `mls.event_clear` (int: -1 off, 0 or 1 = the reset value E0.5 found); refusal test |
| `agv_core/drivers/canbus/read_mls.py` | `snapshot` reads 2070h:01/06/07 and 2080h:01-05; new `decode_event(src, field, temp)` -> `{"field": bool, "temp": bool, "field_code", "temp_code"}` (tables 22-24); new `clear-event` (dry run unless `--go`, refuses while the service holds can0, as `set-markers`) |
| `agv_core/drivers/canbus/guard.py` | `SENSOR_ALLOWED` += `(0x2080, 1): "reset event flag"` (E2). Safety-listed file: test below |
| `tests/test_mls.py` | `decode_event` on every table 22-24 value; unknown values reported, not guessed |
| `tests/test_canmon.py` (holds the `SENSOR_ALLOWED` checks) | assert the exact `SENSOR_ALLOWED` set incl. 2080h:01; 2070h:06/07 (thresholds) and 2080h:02-05 refused |
| `tests/run_all.py` | `EXPECTED_CHECKS` bumped deliberately |

### M1: runtime, read-only (amr_base)

[mls_track.py](../amr_ws/src/amr_base/amr_base/mls_track.py):

- **One SDO queue** (E4): the marker-check queue becomes `_sdo_q` of `(purpose, key)`. Purposes: `marker` (existing behaviour unchanged), `event`, `temp`. Same rules: one read per `poll()`, TPDO mode only, a timeout abandons that purpose and retries later.
- **Event edge:** in `_on_tpdo`, a rising `event_flag` (and E5 gate) queues 2080h:02, 2080h:04, 2080h:05. When all three are in, `on_event(dict)` fires with the decoded cause, `t_mono`, the last LCP2 and `track_level`.
- **Temperature:** 2070h:01 every `TEMP_PERIOD_S = 60`.
- **Counters** in `diagnostic()`: `events_field`, `events_temp`, `event_unread` (flag seen, cause read failed), `temp_c`, `track_level_min` since start.

[drive_node.py](../amr_ws/src/amr_base/amr_base/drive_node.py):

- `on_event` -> `self.event(level, code, text)`: an event per occurrence, not an `_edge` slot (each dip is its own record). Rate limit: at most one `MLS_FIELD_WEAK` per `FIELD_EVENT_MIN_S = 2` s; extra ones are counted into the next text ("+3 more").
- Temperature beyond its threshold uses `_edge("mls_temp", ...)`: it is a lasting condition.

[agv_core/alarms.py](../agv_core/alarms.py), two rows (RUNBOOK §4 regenerated with `--md`):

| Code | Severity | clears_by | Title |
|---|---|---|---|
| `MLS_FIELD_WEAK` | info | auto | Weak tape signal under the line sensor |
| `MLS_TEMP` | warn | engineer | Line sensor outside its temperature range |

Tests (`amr_ws/src/amr_base/test/test_mls_track.py`, fake link and clock):
- rising flag queues exactly three reads, one per poll, and only in TPDO mode;
- a marker check in progress and an event in the same window both finish, no poll does two reads;
- a timeout on 2080h:02 counts `event_unread` and does not block the marker check;
- a flag held high does not re-queue (edge, not level) while M2 is off;
- the E5 gate: a flag rising on a sample with `nlcp == 0` (after a no-track sample) is not counted.

### M2: clear the latch (E2, after E0.5 passes)

- After the cause is read, `mls_track` queues the 2080h:01 write: `link.write` through `guard.check_sensor`, rate-limited to once per `CLEAR_MIN_S = 1` s.
- A clear that fails is logged once and M2 stops for the rest of the run. M1 keeps reporting, but only the first event after power-up.
- Tests: the write happens only after all three cause reads; never in SDO mode; the guard refuses any other sensor object through this path; a refused or failed write turns M2 off and does not stop the marker check.

**Failure mode:** a wrong `SENSOR_ALLOWED` entry or a wrong index in the write would let drive_node change an MLS object at runtime. The guard limits the write to 2080h:01; any other object is refused before it reaches the bus.
**Recovery:** `read_mls snapshot`, then `set-variant` and `set-markers` back to the plan values. The drive_node startup check already refuses markers while 2028h is wrong.

### M3: tape-health map (after tape_loc P1 is enabled on the vehicle)

- New pure module `amr_line/tape_health.py`, clock-fed: per tape leg, bins of 0.25 m along `loc_s_m`: minimum and mean `track_level`, sample count, and field events placed by `s`. Fed from `/amr/line_track` (TPDO only) and `LineState.loc_*`. Only while `loc_health == HEALTHY`. A bin is written only when `loc_sigma_m` is below the bin width.
- At run end, written to `~/.amr/reports/tape_health/<date>.jsonl` (on the W3 disk budget).
- `tools/tape_health_report.py` merges runs and lists the legs whose minimum fell in the last N runs.
- Optional display: a list on the Teach page (engineer). No operator page change.
- Tests: binning; a sample with LOST or a large sigma is dropped; a leg change splits bins.

## 5. Order and size

| Step | Needs | Size |
|---|---|---|
| M0 | nothing | small: offline tests only |
| E0 | M0, the vehicle, service stopped (operator) | one bench session, ~1 h |
| M1 | E0.1-E0.3 | medium: `mls_track` queue refactor plus tests |
| M2 | E0.4-E0.5 PASS | small |
| M3 | tape_loc P1 running on the vehicle | medium |

Verification: focused tests per step (`tests/run_all.py`, `test_mls_track.py`, the new tape_health test). No unified sim run is needed; the fake MLS always publishes `event_flag` false.
On the vehicle: one lap with M1 on. Field events must be zero on good tape at NORMAL and HIGH, and the existing `MLS markers: sensor configured` line must still appear.

## 6. Not in this plan

| Item | Why not |
|---|---|
| Interference compensation (variants 2/4/5/7) | Only if marker plan B7 shows false tracks from floor steel |
| Combi track width | Cuts LCP to 10 bits for a diagnostic |
| MLS accelerometer / orientation | The yaw angle drifts (§8.6); the yaw rate is already fused |
| Slowing or stopping on a weak-field event | Software deciding motion from a diagnostic; tape loss is already handled by the line follower |
