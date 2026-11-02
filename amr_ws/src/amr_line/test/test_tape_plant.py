"""The sim plant closes the loop with the real engine, and the tape ends.

tape_synth_node re-hosts the offline rig's plant so a LINE layer can be
entered, run and left in sim. If its maths drifted from the rig's, the sim
would pass a controller the rig fails or vice versa; so the plant is driven
here by the real follower, at the rig's settings, and must converge the way
the rig says it does.
"""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
for _p in (ROOT, os.path.join(ROOT, "amr_ws", "src", "amr_line"),
           os.path.join(ROOT, "amr_ws", "src", "amr_sim")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from amr_line import autopilot, runtime  # noqa: E402
from amr_sim.tape_plant import TapePlant  # noqa: E402

runtime.load_from_profile()


def sensor(mm):
    if mm is None:
        return {"tracks": [], "has_track": False}
    return {"tracks": [{"index": 2, "pos_mm": mm, "width": 10}], "has_track": True}


def test_the_follower_converges_on_the_synthetic_tape():
    plant = TapePlant(0.030, 50.0, runtime.SENSOR_LOOKAHEAD_M)
    f = autopilot.LineFollower()
    f.reset()
    f._v_rpm = runtime.AUTO_RPM  # already cruising, the rig's demanding case
    left = right = runtime.AUTO_RPM
    t, dt, over = 0.0, 0.02, 0.0
    while t < 12.0:
        mm = plant.reading_mm()
        left, right, _ = f.update(sensor(mm), 0.0, dt, True)
        plant.step(left, right, dt)
        over = max(over, -(plant.y))
        t += dt
    assert abs(plant.y) < 0.002, f"did not settle: y={plant.y * 1000:.1f} mm"
    assert over * 1000 < 25.0, f"overshoot {over * 1000:.1f} mm"


def test_the_tape_ends_and_the_sensor_says_so():
    plant = TapePlant(0.0, 1.0, runtime.SENSOR_LOOKAHEAD_M)
    assert plant.reading_mm() == 0
    plant.step(1000.0, 1000.0, 5.0)  # ~1.57 m at 1000 r/min: past the end
    assert not plant.on_tape
    assert plant.reading_mm() is None


def test_the_reported_sign_is_the_measured_polarity():
    """Sensor 30 mm to the LEFT of the tape (positive y) reports the line at
    -30: line relative to sensor, the polarity measured on 2026-08-31."""
    plant = TapePlant(0.030, 10.0, runtime.SENSOR_LOOKAHEAD_M)
    assert plant.reading_mm() == -30
