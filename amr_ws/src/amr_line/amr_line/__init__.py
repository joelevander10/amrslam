"""LINE layer: magnetic-tape line following.

`autopilot.py` and `branch.py` are a byte-for-byte port of the engine that ran
this vehicle as a tape AGV on the `gy-demo` branch. They are not to be
reformatted, re-ordered or tidied: branch.py's rung order is semantics,
select_track continuity is the run-0023 merge fix, the lateral RATE clamp is
the run-0020 fix, and begin_measured_stop solving a = v^2/2d once rather than
per tick is deliberate. Port provenance is in the two commits that introduced
them - the first a pure copy, the second the three-line import rewrite.

The engine reads its constants from `runtime`, a namespace it is HANDED rather
than a profile loader it imports. Everything else in this package is the ROS
shell: `track` turns a LineTrack message into the sensor dict the engine eats,
and `line_follow_node` owns arming, the Start edge, holds and the command.
"""
