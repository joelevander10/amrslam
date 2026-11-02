# Codebase restructure — the `agv_core` package (2026-09-20)

Record of what moved and why, for anyone porting code across branches or
reading a plan document written before the move.

This is a record, not a decision document. The live decision record is
`dual-product-plan.md`, `line-follow-u11-layout-plan.md` and
`unified_amr_service_plan.md`; where this file and those disagree about intent,
they win.

## What the problem was

The repository had what looked like two source roots. Library code lived at the
repo root as `config.py`, `core/` and `drivers/`; the ROS 2 workspace had its
own `amr_ws/src/`. The two were joined by `amr_base/agv_repo.py`, which walked
up the tree looking for a directory holding both `config.py` and `profiles/`,
then inserted four directories onto `sys.path` so every module could import its
neighbours by bare name (`import guard`, not `from drivers.canbus import guard`).

That flat namespace was deliberate and test-enforced, and it worked. What it
cost: module basenames had to be globally unique, no import stated where its
module lived, and ruff/mypy/pytest/the IDE could not resolve anything without
replicating the path surgery.

`amr_ws/src` was never a second source root competing with the first — it is the
layout colcon requires (`<ws>/src/<pkg>/<pkg>/`) and it did not change.

## The move

| Before | After |
|---|---|
| `config.py` | `agv_core/config.py` |
| `core/<name>.py` | `agv_core/<name>.py` |
| `drivers/<name>.py` | `agv_core/drivers/<name>.py` |
| `drivers/canbus/<name>.py` | `agv_core/drivers/canbus/<name>.py` |
| `import config` | `from agv_core import config` |
| `from guard import check` | `from agv_core.drivers.canbus.guard import check` |
| `from amr_base.agv_repo import config` | `from agv_core import config` |
| `import amr_base.agv_repo` (side effect) | deleted; there is no shim |
| `from server import main` | `from app.server import main` (`app/` is a package) |

137 import lines rewritten across 47 files, plus 24 `agv_repo` references.
`main.py`, `canworker.py`, `app/` and `profiles/` did not move.

Two anchors needed fixing by hand, both of the kind that fail silently:
`config.PROFILE_DIR` now walks one level up from the package to find
`profiles/`, and `runlog.LOG_DIR` was already anchored to the root and happens
to sit at the same depth as before.

## Finding the library at runtime

The legacy controller gets it free — `python3 main.py` from the repo root puts
the root on `sys.path[0]`. ROS nodes do not: they run out of the colcon install
space, which is outside the repo, and the self-locating walk went with
`agv_repo.py`. So:

- `amr_ws/deploy/amr-launch.sh` exports `PYTHONPATH` with the repo root
  (`AGV_CAN_ROOT` still overrides it), and `amr_ws/env/vehicle.sh` does the same
  for an interactive shell.
- `pip install -e .` at the repo root does the job permanently and makes those
  lines redundant but harmless.
- `amr_ws/deploy/validate.sh` refuses to install units if `agv_core/` is not
  beside `amr_ws/`, because the failure mode otherwise is a restart loop with a
  traceback that names no unit file.

`agv_core` is deliberately **not** an `<exec_depend>` in `package.xml` — there is
no rosdep key for it and naming one would only make `rosdep install` fail. It is
stated there as a comment instead.

## What did NOT happen, deliberately

**`app/`, `main.py` and `canworker.py` were not retired.** The plan that
preceded this work had them deleted as its final step. The product decision of
2026-09-20 reverses that: the vehicle ships as the magnetic-tape/RFID AGV with
the SLAM AMR as the paid upgrade, so the legacy controller stays until the
ported LINE mode is accepted on the vehicle. It was migrated to the package and
left fully working, and README.md says so where the tree is described.

Five API surfaces still exist only in `app/server.py` and have no ROS
equivalent — `/api/preflight`, `/api/disarm`, `/api/restart`, `/api/can`, and
the raw `/api/drive` — so retirement is gated on those regardless.

## Porting code from `gy-demo` after this

Increments 1–2 restore modules from the `gy-demo` branch. The source paths on
that branch are the old ones; only the destination changes:

```bash
git show gy-demo:core/autopilot.py    > agv_core/autopilot.py
git show gy-demo:drivers/canbus/x.py  > agv_core/drivers/canbus/x.py
```

A ported file still carries bare imports. Rewrite them to package imports, or
`tests/test_layout.py` will fail on "no module inside agv_core imports a sibling
by bare name" — which is the check existing precisely to catch this.

## Bench tools

They still run standalone, as modules rather than file paths:

```bash
python3 -m agv_core.drivers.canbus.verify_drivers      # from the repo root
python3 agv_core/drivers/canbus/verify_drivers.py      # no longer works
```

Run as a plain script, only the script's own directory reaches `sys.path` and
its `agv_core.` imports do not resolve. The docstring in
`agv_core/drivers/canbus/__init__.py` says so.

## What replaced the layout test

`tests/test_layout.py` used to pin the flat namespace: no two modules sharing a
basename, no stdlib shadowing, the layer directories not being packages. It now
pins the package boundary instead, in 5 checks rather than 4:

- `agv_core` and both subpackages have `__init__.py`; the retired layer
  directories are gone; no `agv_core` directory is itself on `sys.path` (which
  would let a module resolve twice, with two copies of its module state)
- `agv_core` imports neither ROS, nor Flask, nor the legacy controller — at any
  nesting level, lazy imports included. Two runtimes share this library and a
  stray `import flask` breaks the other one at import time, on the vehicle
- no module inside `agv_core` imports a sibling by bare name

`EXPECTED_CHECKS` went 893 → 898 for that reason.

## Verification actually performed

On the Windows dev box, which cannot run the vehicle code: the offline suite was
run before and after the move against a pristine `HEAD` worktree, with a fake
`fcntl` and `os.O_CLOEXEC` shim so it could get past the POSIX wall. Both sides
produced a **byte-identical set of 10 failures** — all of them the owner-lock
and bench-refusal checks that the shim makes meaningless. Additionally every one
of the 28 `agv_core` modules was imported, and every `from agv_core ... import
name` in the repository (ROS side included) was resolved against the real
module: zero unresolved references. `ruff check` over `amr_ws/src/` is clean
again, and the root tree went from 226 violations to 42 under the new shared
config.

None of that is vehicle acceptance. `VEHICLE-TEST-agv_core-restructure.txt` at
the repo root is the prompt for that, and it is explicit about which steps only
the vehicle can answer: the real owner lock, the colcon install space,
`PYTHONPATH` reaching a node, systemd, and any motion at all.
