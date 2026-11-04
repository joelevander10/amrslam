"""Recording floor-label poses into a map's label set (gls-wait-coding-plan §C.2, §C.3).

Pure: no ROS. The route executor ("Record label here") and the mapping session (HOME at
survey start) keep a buffer of Samples - one per /gls/reading, with the vehicle's map pose
at its arrival - and the time the wheels last came to rest; this module turns those into
ONE label observation and writes it:

    T_ML = T_MB (+) T_BC (+) T_LC^-1

T_MB is averaged over the same frames as T_LC. T_BC is the profile's mount (gls.mount_*).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from agv_core import config, glspose

from amr_navigation import store

ROLES = ("home", "dest")
HOME_NAME = "HOME"


@dataclass(frozen=True)
class Sample:
    t: float  # local receive time
    reading: object  # a GlsReading (or anything with its fields)
    t_mb: tuple[float, float, float] | None  # map -> base_footprint at receipt; None if TF had none


@dataclass(frozen=True)
class Observation:
    label_id: str
    t_ml: tuple[float, float, float]
    n_frames: int
    spread_xy_m: float  # of T_LC over the frames: the camera's own repeatability
    spread_yaw_rad: float


def mount() -> tuple[float, float, float]:
    return config.GLS_MOUNT_X_M, config.GLS_MOUNT_Y_M, config.GLS_MOUNT_YAW_RAD


def observe(samples: list[Sample], still_since: float | None, now: float, need_pose: bool = True):
    """(Observation, None) from the samples taken since the wheels came to rest, or
    (None, reason). The wheels must have been still for gls.stationary_s, the newest
    reading must be fresh, and the last gls.fix_frames readings must agree (glspose.observe).
    need_pose=False skips T_MB (the caller supplies it): t_ml is then T_BC (+) T_LC^-1."""
    if still_since is None:
        return None, "the wheels are moving"
    if now - still_since < config.GLS_STATIONARY_S:
        return None, f"stand still for {config.GLS_STATIONARY_S:g} s first"
    used = [s for s in samples if s.t >= still_since]
    if not used or now - used[-1].t > config.GLS_STALE_S:
        return None, "no floor-label reading: is a label under the camera?"
    obs, why = glspose.observe(
        [s.reading for s in used],
        config.GLS_FIX_FRAMES,
        config.GLS_MIN_CODES,
        config.GLS_FIX_SPREAD_XY_M,
        config.GLS_FIX_SPREAD_YAW_RAD,
    )
    if obs is None:
        return None, why
    used = used[-config.GLS_FIX_FRAMES :]
    if need_pose:
        if any(s.t_mb is None for s in used):
            return None, "no map pose (TF map -> base_footprint) for every label frame"
        t_mb = glspose.average([s.t_mb for s in used]).pose
    else:
        t_mb = (0.0, 0.0, 0.0)
    t_ml = glspose.label_in_map(t_mb, mount(), obs.t_lc.pose)
    return Observation(obs.label_id, t_ml, obs.t_lc.n, obs.t_lc.spread_xy_m, obs.t_lc.spread_yaw_rad), None


def at(obs: Observation, t_mb) -> Observation:
    """An observation made with need_pose=False, placed at vehicle pose `t_mb`."""
    return Observation(
        obs.label_id, glspose.compose(t_mb, obs.t_ml), obs.n_frames, obs.spread_xy_m, obs.spread_yaw_rad
    )


def record(
    maps_dir: str,
    map_ref: tuple[str, int, str],
    obs: Observation,
    role: str,
    name: str,
    method: str,
    note: str = "",
) -> tuple[int, str, str]:
    """Write a new label-set revision for map (id, revision, sha256): the newest set for that
    map revision (or an empty one) with `obs` recorded under role/name, replacing any entry
    with the same label ID or name (and the old HOME, for a home). Returns
    (label-set revision, the new entry's text for the operator, what it replaced or "")."""
    map_id, map_rev, map_sha = map_ref
    if role not in ROLES:
        raise store.StoreError(f"role must be one of {ROLES}")
    name = (name or "").strip() or (HOME_NAME if role == "home" else "")
    if not name:
        raise store.StoreError("a destination label needs a name")
    found = store.latest_labels(maps_dir, map_id, map_rev)
    old = found[0]["labels"] if found else []
    if found and found[0]["map"].get("sha256") != map_sha:
        old = []  # the newest set names another bundle of this revision: start afresh
    entry = store.label_entry(
        obs.label_id, role, name, obs.t_ml, method, obs.n_frames, obs.spread_xy_m, obs.spread_yaw_rad, note
    )
    new = store.with_label(old, entry)
    gone = [f"{e['name']} ({e['label_id']})" for e in old if e not in new]
    revision, _path, _sha = store.save_labels(maps_dir, map_id, map_rev, map_sha, new)
    x, y, yaw = obs.t_ml
    text = f"{name} = {obs.label_id} at ({x:.3f}, {y:.3f}) m, {math.degrees(yaw):.1f} deg"
    return revision, text, ", ".join(gone)
