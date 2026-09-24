"""Which vehicle's geometry files to use: selected by AGV_PROFILE, like everything else.

    agv-01 (or AGV_PROFILE unset)  config/footprint.yaml, the xacro defaults (gvievo-01)
    any other profile <p>          config/vehicle.<p>.yaml (xacro args) and
                                   config/footprint.<p>.yaml - BOTH must exist

There is no fallback from another vehicle's files to gvievo-01's: a profile without its
own geometry is refused with the file it looked for, in the same spirit as config.py
refusing a missing profile. A footprint 30 cm too short passes route validation and
then meets a rack.
"""

from __future__ import annotations

import os

import yaml

DEFAULT_PROFILE = "agv-01"  # config.DEFAULT_PROFILE; not imported so this stays ROS-light


def profile_name() -> str:
    return os.environ.get("AGV_PROFILE") or DEFAULT_PROFILE


def _share() -> str:
    from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

    return get_package_share_directory("amr_description")


def config_path(kind: str, profile: str | None = None, share: str | None = None) -> str | None:
    """Path of the vehicle's `kind` file ("vehicle" or "footprint").

    None only for agv-01's "vehicle" (its values are the xacro defaults)."""
    if kind not in ("vehicle", "footprint"):
        raise ValueError(f"unknown vehicle file kind {kind!r}")
    profile = profile or profile_name()
    share = share or _share()
    if profile == DEFAULT_PROFILE:
        return os.path.join(share, "config", "footprint.yaml") if kind == "footprint" else None
    path = os.path.join(share, "config", f"{kind}.{profile}.yaml")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"AGV_PROFILE={profile} has no {kind} file: {path} (every vehicle but {DEFAULT_PROFILE} "
            f"needs its own; copy and MEASURE, do not borrow another vehicle's)"
        )
    return path


def xacro_args(profile: str | None = None, share: str | None = None) -> dict[str, str]:
    """xacro argument overrides for this vehicle ({} for agv-01)."""
    path = config_path("vehicle", profile, share)
    if path is None:
        return {}
    with open(path) as fh:
        doc = yaml.safe_load(fh) or {}
    args = doc.get("xacro") or {}
    if not isinstance(args, dict):
        raise ValueError(f"{path}: 'xacro' must be a mapping of argument -> value")
    return {str(k): str(v) for k, v in args.items()}
