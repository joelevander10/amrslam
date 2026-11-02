# Coding plan: software hardening against power loss and unexpected shutdown

Date 2026-09-22 · branch `slam-roadmap` · proposal, nothing implemented.
Companion to `2026-09-22-operator-ux-and-fault-hardness.md`.

## Baseline (verified on the PC today)

| Item | State |
|---|---|
| Root | ext4 on LVM `ubuntu-vg/ubuntu-lv`, 100 G of a 229.8 G PV → ~130 G unallocated in the VG |
| Swap | 4 G file `/swap.img`, unused |
| Journal | persistent (`/var/log/journal`), no size cap |
| Writers with fsync + rename | map bundles, route/mission store, `operations.jsonl` |
| Writers without fsync | `events.jsonl` (append), `web.json` (role), role logs `~/.amr/logs/*.log` (unbounded, 43 MB now) |
| Stale state | `costmap_footprint_gen*.yaml` accumulate per generation |
| Service | `Type=simple`, `Restart=on-failure`; `sdnotify.py` exists and the supervisor already calls `ready()`/`watchdog()`, but the unit has no `Type=notify` / `WatchdogSec`, so both are no-ops |
| Drives on PC loss | heartbeat lapses → 8130h latched → stopped; needs a drive power cycle after |
| Boot | 27 s to userspace; `amr.service` enabled |

Work items are ordered by value ÷ risk. W1–W6 are code; W7–W9 are configuration
files in `deploy/` that the user installs with sudo. W10 (read-only root) is last and
gated on a decision.

## W1 — Unclean-shutdown marker and boot verdict

Files: new `amr_bringup/amr_bringup/uptime.py`; `supervisor_node.py` (`_boot`,
`_set`, `_shutdown`); `test/test_uptime.py`; `test/test_supervisor_events.py`.

```python
# uptime.py
MARKER = "running.json"
def write(state_dir, instance, mode_name, operation_id, run_id="")  # atomic: tmp + fsync + os.replace
def clear(state_dir)                                                  # os.remove, ENOENT ignored
def verdict(state_dir, now_wall, boot_wall=/proc/stat btime) -> Verdict | None
#   Verdict(kind: "power_loss" | "service_crash", at: float, mode: str, operation_id: str, run_id: str)
#   kind = power_loss when the marker's written_at < boot_wall (the PC rebooted since),
#          service_crash otherwise (same boot, the supervisor died and systemd restarted it)
```

Supervisor:
- `_boot()` first: `v = uptime.verdict(...)`; if present, one event
  `UNCLEAN_SHUTDOWN` (WARN for service_crash, ERROR for power_loss) with text
  "Power was lost at 14:32 while NAVIGATION, mission m3. The drives need a power cycle."
  / "The service crashed at 14:32 while IDLE and restarted itself." Then
  `uptime.write(...)`.
- `_set()`: rewrite the marker on every mode change (cheap, one small atomic write).
  Run id from `snap.run_state` when present.
- `_shutdown()`: `uptime.clear()` as the last line after the children are stopped, so
  a kill during teardown still counts as unclean.
- `_boot_report()`: if the verdict was power_loss, prepend "after a power loss" so the
  "drives (press the safety reset …)" line explains itself.

Tests: marker round-trip; verdict kinds by btime; missing/corrupt marker → None;
supervisor source scan asserts `uptime.clear` is the last call in `_shutdown`;
event text for both kinds.

## W2 — Bounded role logs and stale-state sweep

Files: `amr_bringup/amr_bringup/logs.py` (new); `process_supervisor.py` (`Group.spawn`);
`supervisor_node.py` (`_loop` every 60 s, `_boot`); tests.

- `logs.rotate(path, keep=5)`: at each `spawn()` with a `log_path`, shift
  `base.log → .1 → … → .5` before opening. Every service start and every layer
  transition begins a fresh file; old ones are what a report needs.
- `logs.cap(path, max_bytes=50 MB)`: in the supervisor loop once a minute. Children
  write with `O_APPEND` (spawn opens `"ab"`), so `os.truncate(path, 0)` is safe while
  they run; a line "[truncated at 50 MB]" is written first. Never raises.
- `logs.sweep_generation_files(state_dir, current_gen)`: at boot, remove
  `costmap_footprint_gen*.yaml` (all are stale after a restart). Also in
  `_launch()` after the layer confirms ready, remove other generations.
- `reports.py`: include `*.log.1` as well as the live log.

Tests: rotate chain, cap truncates and appends the marker, sweep keeps only the
current generation, `Group.spawn` rotates before open (tmpdir).

## W3 — Disk-space guard

Files: `amr_bringup/amr_bringup/readiness.py` (`disk_status`), `supervisor_node.py`,
`amr_web/amr_web/adapter.py` (`state()["disk"]`), `alarms` catalogue row, tests.

- `readiness.disk_status(paths, warn_mb=1000, stop_mb=200) -> DiskStatus(free_mb, level)`
  over `state_dir` and `maps_dir` (min of both), `shutil.disk_usage`, once per 30 s.
- Supervisor: `DISK_LOW` WARN event on the warn edge, `DISK_FULL` ERROR on the stop
  edge, cleared edge → INFO. Survey start and map save are refused at `stop`
  (`SURVEY_START_REFUSED "disk full: N MB free"`), never navigation or LINE (a full
  disk must not stop a vehicle that is already running).
- Web: `/api/state.disk = {free_mb, level}`; standing alarm from the catalogue with the
  action "call the engineer: delete old maps or reports".

Tests: thresholds and edges with a fake `disk_usage`; refusal wording; adapter field.

## W4 — fsync where it pays

Files: `amr_web/amr_web/eventlog.py`, `amr_web/amr_web/role.py`, tests.

- `EventLog.append(event, sync=False)`; the adapter passes `sync=(level == "error")`.
  One fsync per error event, the events you want after a cut are exactly these.
- `role.py`: fsync before `os.replace` (it already writes tmp + replace; the fsync is
  missing).
- Nothing else: map bundle, store, operations already do it; the info-level events and
  logs are not worth a sync each.

## W5 — systemd watchdog and notify (unit change)

Files: `deploy/amr.service`, `deploy/validate.sh`, `RUNBOOK` §1.

```
Type=notify
NotifyAccess=main
WatchdogSec=30
TimeoutStartSec=180      # boot budget + drives at "switch on disabled" for 70 s (2026-09-22)
```
The supervisor already sends `READY=1` after boot and `WATCHDOG=1` each loop tick
(`sdnotify.py`), so this is a unit edit plus a check in `validate.sh` that
`WATCHDOG_USEC` reaches the process (`systemctl show -p WatchdogUSec amr.service`).
Verify the stop path still fits `TimeoutStopSec=55` with the watchdog armed.

Caveat: `READY=1` must be sent even when boot ends in `BASE_NOT_READY`, or systemd
kills the service at `TimeoutStartSec` and the "base ready after the boot budget"
path (2026-09-22) never runs. Check `_notify.ready()` sits at the end of `_boot()`
regardless of outcome; add a test that scans for it.

## W6 — Dirty-page window and swap (sysctl + fstab, user installs)

Files: `deploy/sysctl-amr.conf` (new), `deploy/install.sh`, `RUNBOOK` §5.

```
vm.dirty_expire_centisecs = 500      # unsynced data is at most ~5 s old (was 30 s)
vm.dirty_writeback_centisecs = 200
vm.swappiness = 0
```
User steps: `swapoff -a`, delete the `/swap.img` line from `/etc/fstab`, `rm
/swap.img`. 8 GB RAM and no swap use today; swap only lengthens the dirty window and
any boot fsck.

## W7 — Journald cap (config, user installs)

`deploy/journald-amr.conf` → `/etc/systemd/journald.conf.d/amr.conf`:
```
Storage=persistent
SystemMaxUse=300M
SystemMaxFileSize=50M
```
`install.sh` copies it; `validate.sh` checks `journalctl --disk-usage` < 400 M.

## W8 — Separate data volume (LVM, user runs; code: env wiring)

The VG has ~130 G unallocated, so no resize of `/` is needed.

User (sudo, one session, service stopped):
```
lvcreate -L 40G -n amr-data ubuntu-vg
mkfs.ext4 -L amr-data /dev/ubuntu-vg/amr-data
mkdir -p /data/amr
# /etc/fstab
LABEL=amr-data /data/amr ext4 defaults,data=journal,commit=5,nofail,x-systemd.device-timeout=10 0 2
mount /data/amr && install -d -o gvipc-evo-01 -g gvipc-evo-01 /data/amr/{state,maps}
rsync -a ~/.amr/ /data/amr/state/ && rsync -a ~/amr_maps/ /data/amr/maps/
ln -sfn /data/amr/state ~/.amr && ln -sfn /data/amr/maps ~/amr_maps   # tooling keeps working
```
Code/config:
- `deploy/amr.env`: `AMR_STATE_DIR=/data/amr/state`, `AMR_MAPS_DIR=/data/amr/maps`.
- `deploy/amr.service`: `RequiresMountsFor=/data/amr`.
- `validate.sh`: refuse when `AMR_STATE_DIR` is not on a mount with `data=journal`
  (warn, not fail, on the bench).
- `data=journal` is chosen because every important writer here is small and fsynced;
  throughput is irrelevant and it removes the last ordering assumption.

## W9 — Pull-the-plug acceptance steps

`RUNBOOK` §5 and the acceptance file: three cuts, each followed by a boot and a read of
the Home line and the Alarms history:
1. Cut mid-mission (NAVIGATION, vehicle moving in a clear aisle): drives stop
   within the heartbeat timeout; after boot: `UNCLEAN_SHUTDOWN power_loss` names the
   mission; drives need a power cycle; Recover/Restart path reaches IDLE.
2. Cut during a map save (press Save, cut within 1 s): no partial revision directory
   without a manifest; the previous revision loads; the staging dir is left, not a
   revision.
3. Cut while IDLE with the web open: boot to IDLE within the budget with no operator
   action; events history intact including the last error before the cut.

## W10 — Read-only root (decision needed, last)

`overlayroot` is installed (0.47). `overlayroot.conf: overlayroot="tmpfs:recurse=0"`
makes `/` read-only with a tmpfs overlay while `/data/amr` (a separate mount) stays
writable. It removes the last way a cut can stop the PC booting.

Cost: `~/agv_can` and `install/` are on the root, so every code change or colcon
build must be made inside `overlayroot-chroot`, and `/var/log/journal` must be
bind-mounted from `/data/amr/journal` or the journal is lost per boot. Recommend
deferring this until commissioning is over and the vehicle is with an operator, then
enabling it with a `deploy/release-mode.sh` (enable) / `dev-mode.sh` (disable via
`overlayroot=disabled` in the kernel command line) pair and a `validate.sh` line that
prints which mode the PC is in.

## Order and size

| Item | Kind | Size |
|---|---|---|
| W1 marker + verdict | code | ½ day |
| W2 logs + sweep | code | ½ day |
| W3 disk guard | code | ½ day |
| W4 fsync | code | 1 h |
| W5 watchdog unit | unit + check | 1 h + user install |
| W6, W7 sysctl / journald / swap | config | 1 h + user install |
| W8 data volume | LVM + env | 1 h user + ½ h code |
| W9 acceptance | vehicle | ½ day on the floor |
| W10 read-only root | config | deferred |

No message changes anywhere: symlink install plus `sudo systemctl restart amr.service`
after W1–W4; `daemon-reload` + restart after W5–W8.

## Verification
```bash
cd ~/agv_can/amr_ws && source install/setup.bash
env AMR_SIM_TESTS=0 ROS_DOMAIN_ID=89 python3 -m pytest -q src/amr_bringup/test src/amr_web/test
ruff check src && ruff format --check src
bash deploy/validate.sh
systemctl show -p WatchdogUSec,Type amr.service        # after W5
findmnt -no OPTIONS /data/amr                            # after W8: data=journal
```
