# Remove the legacy nanoScan3 listener (`drivers/lidar.py`)

## Context

`drivers/lidar.py` is a passive UDP listener that binds `192.168.3.2:6060` on its own
thread and decodes the nanoScan3 telegram stream. It feeds exactly two things: a
non-critical `lidar` row in the health table, and the `/lidar` diagnostic page plus the
three-lamp zone rail in `app/templates/base.html`. **Nothing gates on it** — the real stop
is the scanner's OSSD pair into the FX3 in hardware, `canworker` never consults `_lidar`
on any motion path, and `tests/test_lidar.py:374` exists to keep it that way.

The SLAM stack now needs that port. `sick_safetyscanners2` and this listener both want
`192.168.3.2:6060` and only one can hold it; `amr_ws/README.md:135` records the
2026-09-15 decision that the ROS driver owns it and the legacy listener is to be removed,
and the spec (§3.4) says "do not run the legacy listener on that endpoint." Today the
consequence is that `/lidar` renders *assumed occupied* whenever `lidar.launch.py` runs —
a permanently wrong page kept alive by a thread that exists to lose a race.

Outcome: the legacy controller stops touching the scanner entirely. The telegram decoder
and the standalone bench tool survive, because SICK commissioning (§3.4: "CoLa2
data-output setup is an explicit commissioning operation") still needs a read-only way to
see what the scanner is actually emitting, and `drivers/lidar_scan.py raw` is the only
thing in the repo that can do it.

**Keep:** `core/lidarframe.py` (328 lines, pure stdlib, no socket) and
`drivers/lidar_scan.py` (281 lines, binds only when a human runs it) — same standing as
the `drivers/canbus/` bench tools the README already blesses.

## Scope

| Removed | Kept |
|---|---|
| `drivers/lidar.py` — the always-on thread and socket | `core/lidarframe.py` — decoder |
| `/lidar` page, `/api/lidar`, its template, JS and CSS | `drivers/lidar_scan.py` — bench tool |
| The zone rail on every page | `tests/fixtures/nanoscan3_telegram.bin` |
| The `lidar` health-table row | The 9 config keys the bench tool reads |
| 5 runtime-only config keys + 2 dead ones | |

## Work

### 1. Config and profile — change together

The loader is strict in both directions (unknown key fatal, missing key fatal), so
`config.py:_SCHEMA` and `profiles/agv-01.json` must move in one commit.

Delete from the `lidar` block in [config.py:385-409](config.py#L385-L409) and from the
`lidar` object in [profiles/agv-01.json](profiles/agv-01.json):

- `silent_warn_s`, `reconnect_period_s`, `decimate` — read only by the deleted listener.
- `expected_identity`, `expected_checksum` — **already dead**: declared in the schema,
  never read by any module in the repo. Verified by grep for `LIDAR_EXPECTED_*`.

Keep these nine, all read by `drivers/lidar_scan.py`: `enabled`, `host_ip`, `sensor_ip`,
`port`, `reassembly_timeout_s`, `zone_block`, `zone_bytes`, `zone_active_low`,
`zones_validated`. Reword the section comment at [config.py:386-388](config.py#L386-L388)
— it currently points at `drivers/lidar.py`; it should say these configure the bench tool,
and that ROS owns the live stream.

Then:
- Drop the validation checks for the removed keys at
  [config.py:782-798](config.py#L782-L798) (`silent_warn_s`, `reassembly_timeout_s` vs
  `silent_warn_s`, `reconnect_period_s`, `decimate`). Keep the port, zone-block and
  `host_ip != sensor_ip` checks. `reassembly_timeout_s` keeps its `> 0` check but loses
  the `< silent_warn_s` pairing.
- Delete `LIDAR_SCAN_CYCLE_S` ([config.py:468-475](config.py#L468-L475)) and its row in
  the derived table at [config.py:984-985](config.py#L984-L985). Its only two consumers
  are the `silent_warn_s` check and the `/lidar` page, both going.
- `_read_zone_bytes()` ([config.py:524](config.py#L524), [:622](config.py#L622)) stays —
  the bench tool calls `zone_spec()`.
- No tuning-note paragraphs to remove: the docstring has no `lidar.*` headings.

### 2. Controller

[canworker.py](canworker.py): delete `import lidar` (:59), the `LidarLink` construction
and `health.PullSource("lidar", …)` registration (:369-384, including the comment block),
`self._lidar.start()` (:414), `self._lidar.stop()` (:420), the whole `lidar_cloud()`
method (:488-496) and the `"lidar"` key in the state snapshot (:589).

Then `rm drivers/lidar.py`. The two `events.warn`/`events.info` calls it made
("lidar lost", "lidar connected") go with it.

### 3. Web tier

- [app/server.py](app/server.py): delete the `/lidar` route (:122-131) and `/api/lidar`
  (:375-385).
- `rm app/templates/lidar.html app/static/lidar.js`.
- [app/templates/base.html](app/templates/base.html): remove the nav entry (:16) and the
  whole `zone-rail` block with its comment (:44-56).
- [app/static/common.js](app/static/common.js): remove `zoneState()` (:126-135ish) and
  the zone-rail rendering at the end of the state callback (:204-226). Check the early
  `return` at :208 — it currently guards the rest of the function; removing the block must
  not orphan code that follows it.
- [app/static/alarms.js](app/static/alarms.js): remove the lidar rows (:49-57). The
  `hw.sensor_error` row above it is generic and stays.
- [app/static/app.css](app/static/app.css): remove the lidar section (~:449-504:
  `.zone-list`, `.zone-state`, `.lamp.zone.*`, `.scan-wrap`, `#scan`, `.scan-stale`,
  `.scan-range`), the shared-rail pips (~:527-548) and `#scan-toggle` (~:620-622). **Keep**
  the base `.lamp` rules (:432-440) and every `.io-*` rule — the `/io` page uses both.
  Remove `.zone-pip` from the shared transition list at :61.

### 4. Tests

`tests/test_lidar.py` splits cleanly at line 243: everything above is `lidarframe`,
everything below is `LidarLink`.

- Keep `test_header_and_reassembly`, `test_block_table_is_the_only_map`,
  `test_beam_count_comes_from_the_wire`, `test_geometry_is_read_not_assumed`,
  `test_zones_are_never_silently_clear` (all use `lf.` only) and
  `test_reader_never_writes_to_the_scanner` — trim its file list at
  [tests/test_lidar.py:414](tests/test_lidar.py#L414) to `drivers/lidar_scan.py` and
  `core/lidarframe.py`. This scan is the reason the rule survives the deletion: it is what
  stops someone re-adding a vendor library that writes the scanner's config.
- Delete `_link()` and the five `LidarLink` tests (:243-404), and
  `test_lidar_cannot_stop_the_vehicle` (:374) — it asserts against `canworker` source that
  no longer mentions `_lidar`. Update the module docstring and the `TESTS` list.
- [tests/test_web.py](tests/test_web.py): delete
  `test_lidar_page_is_read_only_and_never_reads_clear` (:100) and
  `test_the_scan_is_off_until_asked_for` (:218), and both entries in `TESTS` (:480+). In
  `test_the_shared_rail_is_on_every_page` (:173-195) drop `id="zone-rail"` from `OPERATOR`
  and `/lidar` from the page tuple. Add one check that `/lidar` now 404s, next to the
  existing `check("the retired auto page is not served", …)` at :82 — same pattern, same
  reason.
- [tests/test_health.py](tests/test_health.py) needs **no change**: it uses `"lidar"` only
  as an arbitrary source name in its own fixtures.
- [tests/test_config.py:241](tests/test_config.py#L241) keeps `("lidar", "zone_bytes")` —
  that key survives. `test_tuning_notes_are_parsed_not_restated` is unaffected.

### 5. Recount `EXPECTED_CHECKS`

[tests/run_all.py:41](tests/run_all.py#L41) pins `EXPECTED_CHECKS = 853`. It will drop:
the deleted tests, plus one file-scan check each in `test_layout.py` for
`drivers/lidar.py`, `lidar.html` and `lidar.js` (both scans glob the tree). Keep
`"test_lidar"` in `MODULES` — the decoder tests still live there.

Run the suite, read the reported total, set the constant to it. The README rule is "never
lower it to get a green run" — this is the deliberate exception, so the commit message
must state the old and new numbers and that the delta is deleted tests, not lost coverage.

### 6. Documentation

- [README.md](README.md): drop `lidar.py` from the layout block (:128); keep
  `lidarframe.py` (:121) but reword it as decoder-for-the-bench-tool. Remove `lidar` from
  the health table row (:248) — it becomes `rfid`, `dio`. Add a line to the SLAM status
  block saying the scanner is owned by the ROS stack. The check count at :30 says 824 and
  is already stale against 853; fix it to the new number while there.
- [manuals/slam-generalized-plan/hardware-reconciliation.md](manuals/slam-generalized-plan/hardware-reconciliation.md)
  and `install-list.md` mention the listener — check and update the lines that describe it
  as live.
- `manuals/obsolete/lidar_brief.md` is referenced by the surviving comments in
  `lidarframe.py` and `lidar_scan.py`. Leave it where it is; those references stay valid.

### 7. Two files inside `amr_ws` — do these last, or hand them over

`amr_ws/README.md:135` and `amr_ws/src/amr_bringup/config/nanoscan3.yaml:12` both describe
the listener in the present tense and should become past tense. **Another agent is
actively working T7/T8 in that tree**, and `amr_ws/README.md` is a likely conflict.
Recommend doing these two one-line edits last, after the root-repo work is committed, or
leaving them to whoever lands T7/T8.

## Verification

```bash
cd ~/agv_can
python3 tests/run_all.py            # green; note the new total for EXPECTED_CHECKS
python3 -c "import main"            # profile still loads under the trimmed schema
AGV_PROFILE=agv-01 python3 -c "import config; config.load(); print(config.describe.__name__)"
```

- **Nothing binds 6060.** Start the app, then `ss -lunp | grep 6060` → no rows. This is
  the whole point of the change; check it before and after.
- **`/params` still renders every schema key.** `test_config.py:230` asserts this
  generically and will catch a schema/profile mismatch, but load `/params` by hand and
  confirm the lidar section shows nine rows and no orphan.
- **`/io` lamps still look right.** The `.lamp` base rules are shared; a CSS over-delete
  shows up there, not on any page the tests render.
- **No page throws.** Open `/manual`, `/monitor`, `/io`, `/alarms`, `/params` with the
  console open — a leftover `getElementById('rail-z-0')` reference in `common.js` would
  kill the whole `onState` callback and silently freeze telemetry on every page. That
  failure mode is called out at `tests/test_web.py:92`; it is the main risk in step 3.
- **The bench tool still works standalone:**
  ```bash
  python3 drivers/lidar_scan.py --help        # imports cleanly with no listener
  sudo systemctl stop agv_controller          # nothing else may hold 6060
  python3 drivers/lidar_scan.py raw           # on the vehicle, with ROS stopped
  ```
- **Leftover grep:** `grep -rn -i "lidar" app/ core/ canworker.py main.py --include=*.py
  --include=*.js --include=*.html` should return only `lidarframe.py` and prose mentions
  of the hardware safety chain.
