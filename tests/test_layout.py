"""The layout itself: filesystem anchors and the package boundary.

Two failure modes this refactor introduced, both silent:

  * a module that computes a path from its own __file__ points somewhere new
    the moment it moves directory. Nothing raises - the profile is simply not
    found, on the vehicle. Only a check that pins the resolved path catches that.

  * the vehicle library is imported by the ROS nodes under amr_ws/ and by the
    bench tools, and must depend on neither ROS nor Flask. A stray `import
    rclpy` inside agv_core breaks a bench tool at import time, on the vehicle,
    not here. (Until U11, 2026-09-21, the legacy controller was the second
    runtime; the rule outlives it.)

Until 2026-09-20 this file pinned the opposite arrangement: core/, drivers/,
drivers/canbus/ and app/ went on sys.path rather than becoming packages, which
made module BASENAMES one flat namespace where two modules could never share a
name. That is gone. agv_core is a real package, imports state where a module
lives, and what needs pinning now is the package boundary instead.
"""
import ast
import pathlib
import sys

from helpers import ROOT, check

from agv_core import config

# Directories that must NOT come back as sys.path layers.
RETIRED_LAYER_DIRS = ["core", "drivers", "drivers/canbus"]
PKG = ROOT / "agv_core"


def test_path_anchors():
    """Directory-dependent paths must resolve to the repo root, not to the
    directory the module happens to live in today."""
    print("\nfilesystem anchors survive the move")

    check("config.PROFILE_DIR points at the repo root",
          pathlib.Path(config.PROFILE_DIR).resolve()
          == (ROOT / "profiles").resolve(), config.PROFILE_DIR)

    check("the profile resolves under profiles/",
          pathlib.Path(config.profile_path()).parent.resolve()
          == (ROOT / "profiles").resolve())


def test_the_library_is_a_package():
    """agv_core is imported by package path, from anywhere, exactly once."""
    print("\nthe vehicle library is a real package")

    for d in ["", "drivers", "drivers/canbus"]:
        sub = PKG / d if d else PKG
        check(f"agv_core/{d} is a package" if d else "agv_core is a package",
              (sub / "__init__.py").is_file(), str(sub))

    check("the retired layer directories are gone",
          not any((ROOT / d).exists() for d in RETIRED_LAYER_DIRS),
          str([d for d in RETIRED_LAYER_DIRS if (ROOT / d).exists()]))

    # A module reachable under two names is the hazard the flat namespace had
    # and the package was supposed to end: it would be imported twice, with two
    # copies of its module-level state (config's loaded profile, ownerlock's
    # handles). The repo root on sys.path is what makes agv_core importable;
    # the package's own directories must never join it.
    inside = {PKG.resolve(), (PKG / "drivers").resolve(),
              (PKG / "drivers" / "canbus").resolve()}
    on_path = [p for p in sys.path if p and pathlib.Path(p).resolve() in inside]
    check("no agv_core directory is itself on sys.path", not on_path, str(on_path))

    names = sorted(f.stem for f in PKG.rglob("*.py") if not f.stem.startswith("_"))
    shadowed = sorted(set(names) & set(sys.stdlib_module_names))
    check("no library module shadows a standard library module",
          not shadowed, str(shadowed))


def _imported_modules(path):
    """Every absolute module name imported anywhere in a file, lazy ones too."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                yield node.lineno, a.name
        elif isinstance(node, ast.ImportFrom) and node.module and node.level == 0:
            yield node.lineno, node.module


def test_the_library_depends_on_neither_runtime():
    """agv_core is shared by the ROS stack and the bench tools, so it may
    import neither ROS nor Flask - at any nesting level, lazily or not. The
    legacy names stay on the list so a revival is caught the same way."""
    print("\nthe vehicle library stays runtime-neutral")

    forbidden = {"rclpy", "rcl_interfaces", "geometry_msgs", "nav_msgs",
                 "sensor_msgs", "std_msgs", "amr_interfaces", "amr_base",
                 "flask", "werkzeug", "jinja2", "canworker", "app"}
    offenders = []
    for f in sorted(PKG.rglob("*.py")):
        for lineno, mod in _imported_modules(f):
            if mod.split(".")[0] in forbidden:
                offenders.append(f"{f.relative_to(ROOT)}:{lineno} {mod}")
    check("agv_core imports neither ROS nor Flask nor the legacy controller",
          not offenders, "; ".join(offenders[:3]))

    # Bare sibling imports are what the package replaced. One creeping back in
    # resolves only when something else has put that directory on sys.path,
    # which is precisely the accident-by-path-order this layout removed.
    local = {f.stem for f in PKG.rglob("*.py")} - {"__init__"}
    bare = [f"{f.relative_to(ROOT)}:{lineno} {mod}"
            for f in sorted(PKG.rglob("*.py"))
            for lineno, mod in _imported_modules(f)
            if mod.split(".")[0] in local]
    check("no module inside agv_core imports a sibling by bare name",
          not bare, "; ".join(bare[:3]))


# test_entry_point (main.py) and test_assets_are_offline (app/static) went
# with the legacy controller at U11 (2026-09-21).
TESTS = [
    test_path_anchors,
    test_the_library_is_a_package,
    test_the_library_depends_on_neither_runtime,
]
