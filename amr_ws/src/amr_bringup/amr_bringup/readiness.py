"""Snapshots and readiness predicates for the supervisor (unified plan §3.2).

Snapshots records what the base and the current layer last said, with local
monotonic receipt times; generation-tagged layer state is dropped on a
transition (reset_layer_state) so a replaced layer's latched READY can never
satisfy a new layer's readiness. Predicates are pure functions over it.
"""

from __future__ import annotations

import os
import re
import threading
import time

from agv_core import disk

from amr_bringup import mode_fsm as fsm

MAP_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")

# drive_node's standby (2026-10-08): linked, power stage off on purpose. Not an emergency.
STANDBY_STATES = ("Ready to switch on", "Switched on")


def drives_ok(operational: bool, link_state: str, left_state: str, right_state: str) -> bool:
    """mode_fsm.Conditions.drives_ok: False means an emergency (E-stop / safety chain / fault).

    Since the drives are powered only on demand (amr_base.arm_policy), "not operational" is
    the normal idle state: standby with both drives Ready to switch on is fine. Switch on
    disabled (STO held by the FX3), a fault, a fully disarmed link or anything unknown is not.
    """
    if operational:
        return True
    return link_state == "standby" and left_state in STANDBY_STATES and right_state in STANDBY_STATES

# Disk thresholds live in agv_core.disk: the supervisor and the web both need the
# same answer, and the web must not import the supervisor's internals to get it.
DISK_WARN_MB, DISK_STOP_MB = disk.WARN_MB, disk.STOP_MB
DISK_OK, DISK_WARN, DISK_STOP, DISK_UNKNOWN = disk.OK, disk.WARN, disk.STOP, disk.UNKNOWN
DiskStatus = disk.DiskStatus
disk_status = disk.status


def check_map_id(map_id: str) -> str:
    """A map id is a plain directory name: no separators, no traversal, no shell."""
    if not MAP_ID_RE.match(map_id or ""):
        raise ValueError(f"invalid map id {map_id!r}: letters, digits, '_' and '-' only")
    return map_id


def resolve_map(maps_dir: str, map_id: str, revision: int) -> tuple[str, int, str]:
    """(map_id, revision, sha256) of a verified bundle. Exact revision required (>0)."""
    from amr_mission import map_bundle as mb  # noqa: PLC0415 - heavy, only on request

    check_map_id(map_id)
    if revision <= 0:
        raise ValueError("an exact map revision (> 0) is required")
    root = os.path.realpath(maps_dir)
    rev_dir = os.path.realpath(mb.revision_dir(maps_dir, map_id, revision))
    if not rev_dir.startswith(root + os.sep):
        raise ValueError("map path escapes the map store")
    if not os.path.isdir(rev_dir):
        raise ValueError(f"no such map revision: {map_id} rev{revision}")
    manifest = mb.load_manifest(rev_dir, map_id, revision)  # raises BundleError on any mismatch
    return map_id, revision, manifest.sha256


CONTINUE_FILES = ("posegraph.posegraph", "posegraph.data")
HINT_LIMIT_M = 1.0e5  # a pose hint beyond 100 km is a typo, not a map coordinate


def continuable(maps_dir: str, map_id: str, revision: int) -> str:
    """sha256 of a verified revision whose saved pose graph a survey can continue.
    ValueError otherwise: a merged or imported map has no pose graph to load."""
    import yaml  # noqa: PLC0415

    from amr_mission import map_bundle as mb  # noqa: PLC0415 - heavy, only on request

    try:
        _, _, sha = resolve_map(maps_dir, map_id, revision)  # verifies every hash once
        with open(os.path.join(mb.revision_dir(maps_dir, map_id, revision), mb.MANIFEST)) as fh:
            files = (yaml.safe_load(fh) or {}).get("files") or {}
    except (mb.BundleError, OSError) as e:
        raise ValueError(f"cannot continue {map_id} rev{revision}: {e}") from e
    missing = [f for f in CONTINUE_FILES if f not in files]
    if missing:
        raise ValueError(
            f"cannot continue {map_id} rev{revision}: no saved pose graph ({', '.join(missing)}); "
            "only a surveyed revision can be continued"
        )
    return sha


def check_hint(x: float, y: float, yaw: float) -> None:
    for v in (x, y, yaw):
        if not isinstance(v, (int, float)) or v != v or abs(v) > HINT_LIMIT_M:
            raise ValueError("the pose hint must be a finite pose on the map")


def bundle_sha(maps_dir: str, map_id: str, revision: int) -> str:
    try:
        return resolve_map(maps_dir, map_id, revision)[2]
    except Exception:  # noqa: BLE001 - diagnostic value only
        return ""


class Snapshots:
    """Thread-safe last-message store. Callbacks run on the executor; the loop reads."""

    def __init__(self, still_wheel_rad_s: float = 0.02) -> None:
        self._lock = threading.Lock()
        self.still_thr = still_wheel_rad_s
        self.mux_t = None
        self.mux_gen = 0
        self.mux_inhibited = False
        self.drives_t = None
        self.drives_ok = False
        self.wheels_t = None
        self.wheels_still_since = None
        self.panel_t = None
        self.panel_valid = False
        self.panel_manual = False
        self.mapping_gen = None
        self.mapping_state = None
        self.mapping_map_id = ""
        self.run_gen = None
        self.run_state = None
        self.run_mission = ""
        self.loc_gen = None
        self.loc_state = None
        self.line_gen = None
        self.line_state = None
        self.commissioning_phase = 0
        self.commissioning_t = None

    # -- callbacks --

    def on_mux(self, m) -> None:
        with self._lock:
            self.mux_t = time.monotonic()
            self.mux_gen = int(m.generation)
            self.mux_inhibited = bool(m.inhibited)

    def on_drives(self, m) -> None:
        with self._lock:
            self.drives_t = time.monotonic()
            self.drives_ok = drives_ok(
                bool(m.operational), str(m.link_state), str(m.left_state), str(m.right_state)
            )

    def on_wheels(self, m) -> None:
        with self._lock:
            now = time.monotonic()
            if not (m.left_valid and m.right_valid):
                self.wheels_still_since = None
                return
            self.wheels_t = now
            still = abs(m.left_vel_rad_s) <= self.still_thr and abs(m.right_vel_rad_s) <= self.still_thr
            if not still:
                self.wheels_still_since = None
            elif self.wheels_still_since is None:
                self.wheels_still_since = now

    def on_panel(self, m) -> None:
        with self._lock:
            self.panel_t = time.monotonic()
            self.panel_valid = bool(m.valid)
            self.panel_manual = not bool(m.mode_auto)

    def on_mapping(self, m) -> None:
        with self._lock:
            self.mapping_gen = int(m.generation)
            self.mapping_state = int(m.state)
            self.mapping_map_id = m.map_id

    def on_run(self, m) -> None:
        with self._lock:
            self.run_gen = int(m.generation)
            self.run_state = int(m.state)
            # Kept only so the unclean-shutdown marker can name the interrupted mission
            # (power-loss plan W1); nothing gates on it.
            self.run_mission = str(getattr(m, "mission_id", "") or "")

    def on_loc(self, m) -> None:
        with self._lock:
            self.loc_gen = int(m.generation)
            self.loc_state = int(m.state)

    def on_line(self, m) -> None:
        with self._lock:
            self.line_gen = int(m.generation)
            self.line_state = int(m.state)

    def on_commissioning(self, m) -> None:
        with self._lock:
            self.commissioning_phase = int(m.phase)
            self.commissioning_t = time.monotonic()

    def _commissioning_active_locked(self, now: float, fresh_s: float = 2.0) -> bool:
        return (
            self.commissioning_t is not None
            and now - self.commissioning_t <= fresh_s
            and self.commissioning_phase in (1, 2, 3)
        )

    def commissioning_active(self, now: float, fresh_s: float = 2.0) -> bool:
        """PREPARED / RUNNING / SETTLING (1..3) on a fresh state; a stale state is not a job."""
        with self._lock:
            return self._commissioning_active_locked(now, fresh_s)

    # -- reads --

    def reset_layer_state(self) -> None:
        with self._lock:
            self.mapping_gen = self.mapping_state = None
            self.mapping_map_id = ""
            self.run_gen = self.run_state = None
            self.run_mission = ""
            self.loc_gen = self.loc_state = None
            self.line_gen = self.line_state = None

    def mux_acknowledged(self, generation: int, now: float, fresh_s: float = 0.5) -> bool:
        with self._lock:
            return self.mux_t is not None and now - self.mux_t <= fresh_s and self.mux_gen == generation

    def mapping_state_for(self, generation: int) -> bool:
        with self._lock:
            return self.mapping_gen == generation and self.mapping_state is not None

    def run_state_for(self, generation: int) -> bool:
        with self._lock:
            return self.run_gen == generation and self.run_state is not None

    def loc_state_for(self, generation: int) -> bool:
        with self._lock:
            return self.loc_gen == generation and self.loc_state is not None

    def line_state_for(self, generation: int) -> bool:
        with self._lock:
            return self.line_gen == generation and self.line_state is not None

    def _line_active_locked(self, generation: int) -> bool:
        """ARMED / RUNNING / HOLD (1..3) of THIS generation: a job that holds the vehicle."""
        return self.line_gen == generation and self.line_state in (1, 2, 3)

    def conditions(self, now: float, generation: int, operation_pending: bool) -> fsm.Conditions:
        with self._lock:
            return fsm.Conditions(
                now=now,
                wheels_t=self.wheels_t,
                wheels_still_since=self.wheels_still_since,
                panel_t=self.panel_t,
                panel_valid=self.panel_valid,
                panel_manual=self.panel_manual,
                run_state=self.run_state if self.run_gen == generation else None,
                survey_state=self.mapping_state if self.mapping_gen == generation else None,
                operation_pending=operation_pending,
                commissioning_active=self._commissioning_active_locked(now),  # lock already held
                line_active=self._line_active_locked(generation),
                drives_ok=bool(self.drives_t is not None and now - self.drives_t <= 1.0 and self.drives_ok),
            )


def base_missing(s: Snapshots, now: float, fresh_s: float = 1.0) -> str:
    with s._lock:
        missing = []
        if s.mux_t is None or now - s.mux_t > fresh_s:
            missing.append("mux")
        if s.drives_t is None or now - s.drives_t > fresh_s:
            missing.append("drives")
        if s.wheels_t is None or now - s.wheels_t > fresh_s:
            missing.append("wheel feedback")
        if s.panel_t is None or now - s.panel_t > fresh_s:
            missing.append("panel")
    return ", ".join(missing)


def base_ready(s: Snapshots, now: float) -> bool:
    return base_missing(s, now) == ""
