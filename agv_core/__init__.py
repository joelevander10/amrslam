"""Vehicle library: profile loader, kinematics, bus ownership, device drivers.

This package holds everything that is neither a ROS node nor a web page — the
code the ROS 2 stack under `amr_ws/` imports (until 2026-09-21 the legacy
standalone controller imported it too; U11 retired that). It has no ROS
dependency and no Flask dependency, and nothing in it may acquire one.

Import it by package path, never by bare module name:

    from agv_core import config, kinematics
    from agv_core.drivers import dio
    from agv_core.drivers.canbus import guard

Before 2026-09-20 these modules lived at the repo root as `config.py`, `core/`
and `drivers/`, and were reached by inserting those directories onto sys.path
so every module could import its neighbours by bare name. That flat namespace
is gone: basenames no longer have to be globally unique, and an import states
where the module actually lives. `tests/test_layout.py` pins the new rules.

`profiles/` stays at the repo root — it is site data, not library code, and
`config.PROFILE_DIR` resolves to it by walking up from this package.
"""
