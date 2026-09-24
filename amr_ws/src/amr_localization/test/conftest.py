"""Simulation launch tests are opt-in (AMR_SIM_TESTS=1).

They are left out of collection here instead of relying only on their module-level
pytest.skip: with plugin autoload the launch_testing plugin imports them while
collecting, and that Skipped escapes its hook and ends the WHOLE session as
"1 skipped" - every unit test next to them silently not run (review R34).
"""

import os

SIM_TESTS = [
    "test_amcl_sim.py",
]

collect_ignore = [] if os.environ.get("AMR_SIM_TESTS") == "1" else list(SIM_TESTS)


def pytest_report_header(config):
    if collect_ignore:
        return f"amr_localization: sim tests skipped (AMR_SIM_TESTS=1 runs them): {', '.join(SIM_TESTS)}"
    return None
