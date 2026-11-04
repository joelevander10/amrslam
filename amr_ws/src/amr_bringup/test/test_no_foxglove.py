"""Foxglove was removed from the vehicle software on 2026-11-03 (manuals/foxglove-removal-plan.md).

The bridge exposed every ROS topic, unauthenticated, on 0.0.0.0:8765. No ROS import here, so
this guard runs on any machine: a code or deploy file that mentions it again fails the suite.
"""

import pathlib

REPO = pathlib.Path(__file__).resolve().parents[4]
SCANNED = ("amr_ws/src", "amr_ws/deploy", "agv_core")
SUFFIXES = {
    ".py", ".xml", ".yaml", ".yml", ".sh", ".env", ".service", ".conf", ".json", ".js", ".html", ".cfg",
}


def test_no_code_or_deploy_file_mentions_foxglove():
    this = pathlib.Path(__file__).resolve()
    hits = []
    for top in SCANNED:
        for path in (REPO / top).rglob("*"):
            if not path.is_file() or path.resolve() == this:
                continue
            if path.suffix not in SUFFIXES and path.name != "amr.env":
                continue
            if any(part in ("build", "install", "log", "__pycache__") for part in path.parts):
                continue
            text = path.read_text(encoding="utf-8", errors="replace").lower()
            if "foxglove" in text:
                hits.append(str(path.relative_to(REPO)))
    assert not hits, f"foxglove is back in: {hits}"
