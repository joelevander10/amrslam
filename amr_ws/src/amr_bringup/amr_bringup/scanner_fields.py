"""The nanoScan3 field set as the software knows it (config/scanner_fields.yaml).

    load(path=None)        -> validated dict (raises ValueError on anything off)
    mux_params(fields)     -> the cmd_mux_kinematics field parameters
    reach_m(rect)          -> the farthest corner, = the range /field_data reports

The scanner's verified configuration is the authority; this copy only tells the
software which /output_paths index is which field and what each does to AUTO.
"""

from __future__ import annotations

import math
import os

import yaml

ROLES = ("protective", "warning_1", "warning_2")
RECT_KEYS = ("x_min", "x_max", "y_min", "y_max")


def default_path() -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("amr_bringup"), "config", "scanner_fields.yaml")


def reach_m(rect: dict) -> float:
    return max(math.hypot(x, y) for x in (rect["x_min"], rect["x_max"]) for y in (rect["y_min"], rect["y_max"]))


def _inside(a: dict, b: dict) -> bool:
    return b["x_min"] <= a["x_min"] and a["x_max"] <= b["x_max"] and b["y_min"] <= a["y_min"] and a["y_max"] <= b["y_max"]


def validate(doc: dict) -> dict:
    if not isinstance(doc, dict) or not isinstance(doc.get("fields"), dict):
        raise ValueError("scanner_fields: expected a 'fields' mapping")
    f = doc["fields"]
    if set(f) != set(ROLES):
        raise ValueError(f"scanner_fields: expected exactly {list(ROLES)}, got {sorted(f)}")
    for role in ROLES:
        r = f[role]
        for key in ("path", "active", "scale", "rect"):
            if key not in r:
                raise ValueError(f"scanner_fields.{role}: missing {key}")
        if not isinstance(r["path"], int) or isinstance(r["path"], bool) or not 0 <= r["path"] < 20:
            raise ValueError(f"scanner_fields.{role}.path must be an output path index 0..19")
        if not isinstance(r["active"], bool):
            raise ValueError(f"scanner_fields.{role}.active must be true or false")
        if not isinstance(r["scale"], (int, float)) or not 0.0 <= r["scale"] <= 1.0:
            raise ValueError(f"scanner_fields.{role}.scale must be in [0, 1]")
        rect = r["rect"]
        if not isinstance(rect, dict) or set(rect) != set(RECT_KEYS):
            raise ValueError(f"scanner_fields.{role}.rect: expected {list(RECT_KEYS)}")
        if not (rect["x_min"] < rect["x_max"] and rect["y_min"] < rect["y_max"]):
            raise ValueError(f"scanner_fields.{role}.rect is empty")
    if len({f[r]["path"] for r in ROLES}) != 3:
        raise ValueError("scanner_fields: the three fields need three different output paths")
    if f["protective"]["scale"] != 0.0:
        raise ValueError("scanner_fields.protective.scale must be 0.0: the protective field is a stop")
    if not f["warning_2"]["scale"] <= f["warning_1"]["scale"] < 1.0:
        raise ValueError("scanner_fields: need warning_2.scale <= warning_1.scale < 1 (inner is the slower)")
    if not (_inside(f["protective"]["rect"], f["warning_2"]["rect"]) and _inside(f["warning_2"]["rect"], f["warning_1"]["rect"])):
        raise ValueError("scanner_fields: fields must nest protective in warning_2 in warning_1")
    stops = [r for r in ("warning_1", "warning_2") if f[r]["scale"] == 0.0]
    if stops != [r for r in ("warning_1", "warning_2") if "stop_m" in f[r]]:
        raise ValueError("scanner_fields: a warning field has stop_m exactly when its scale is 0.0 (a stop)")
    if stops:
        if stops != ["warning_2"]:
            raise ValueError("scanner_fields: only warning_2 (inner) may be a stop; warning_1 slows first")
        stop_m = f["warning_2"]["stop_m"]
        gap = f["warning_2"]["rect"]["x_max"] - f["protective"]["rect"]["x_max"]
        if not isinstance(stop_m, (int, float)) or isinstance(stop_m, bool) or not 0.0 < stop_m <= gap + 1e-9:
            raise ValueError(f"scanner_fields.warning_2.stop_m must be in (0, {gap:.2f}] (the gap to protective)")
    if len({f[r]["active"] for r in ROLES}) != 1:
        raise ValueError("scanner_fields: the mux takes one polarity for all paths; they disagree")
    return doc


def load(path: str | None = None) -> dict:
    with open(path or default_path()) as fh:
        return validate(yaml.safe_load(fh))


def mux_params(doc: dict) -> dict:
    f = doc["fields"]
    return {
        "protective_index": f["protective"]["path"],
        "warning_indices": [f["warning_1"]["path"], f["warning_2"]["path"]],
        "warning_scales": [float(f["warning_1"]["scale"]), float(f["warning_2"]["scale"])],
        "warning_active_level": f["warning_1"]["active"],
        "warning_stop_m": float(f["warning_2"].get("stop_m", 0.0)),
    }
