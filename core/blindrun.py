"""Encoder-only ("blind") motion: plan a move in wheel counts, then drive it.

Pure computation owned by the CAN thread: no clocks, no I/O. The
caller supplies the drives' 6064h counts each tick and writes the wheel speeds
returned. The purpose is to MEASURE the encoders before SLAM relies on them, so
the plan, the counts reached at rest and the integrated pose are reported as
they are; nothing here corrects towards a physical reference.

Vehicle terms throughout: x forward, heading positive counter-clockwise.

  straight  distance_m   signed; negative drives backwards
  arc       radius_m     signed centre-path radius; + turns left, - right
            angle_deg    swept angle; + forwards, - backwards
  pivot     angle_deg    + counter-clockwise, - clockwise
  pulses    left, right  raw 6064h counts in DRIVER terms, as the drive sees them
"""
import math

import config
import kinematics

RUNNING, SETTLING, DONE, ABORTED = "running", "settling", "done", "aborted"
KINDS = ("straight", "arc", "pivot", "pulses")
# Transit time allowed per segment: a multiple of the planned trapezoid plus
# slack. A stalled wheel or a frozen counter would otherwise wait for ever.
TIME_MARGIN = 2.0
TIME_SLACK_S = 2.0
# Longest wait at rest for both speed-zero bits before calling it a fault.
SETTLE_LIMIT_S = 5.0
# Completed segments kept in memory for the page; the CSV keeps all of them.
RESULTS_KEPT = 50


def counts_delta(new, old):
    """6064h is INT32 and wraps; the shorter way round is the real travel."""
    d = (int(new) - int(old)) & 0xFFFFFFFF
    return d - (1 << 32) if d >= 1 << 31 else d


def _num(spec, key, where):
    try:
        value = float(spec[key])
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"{where}: {key} must be a number") from None
    if not math.isfinite(value):
        raise ValueError(f"{where}: {key} must be finite")
    return value


def _vehicle(value, invert):
    """Driver terms <-> vehicle terms: the rule kinematics applies to r/min."""
    return -value if invert else value


def _speed_rpm(speed):
    if (not isinstance(speed, dict) or len(speed) != 1
            or next(iter(speed)) not in ("speed_mps", "motor_rpm")):
        raise ValueError("speed must be {speed_mps: n} or {motor_rpm: n}")
    (key,) = speed
    value = _num(speed, key, "speed")
    rpm = kinematics.mps_to_rpm(value) if key == "speed_mps" else value
    if rpm <= 0:
        raise ValueError("speed must be > 0")
    return rpm


def _duration(distance_m, rpm):
    """Trapezoid time for the faster wheel, at blind_run.accel_rpm_s."""
    v = kinematics.rpm_to_mps(rpm)
    a = kinematics.rpm_to_mps(config.BLIND_ACCEL_RPM_S)
    if distance_m >= v * v / a:
        return distance_m / v + v / a
    return 2.0 * math.sqrt(distance_m / a)


def pose_delta(left_m, right_m):
    """Constant-curvature pose change (dx, dy, dheading) for wheel travel."""
    s = (left_m + right_m) / 2.0
    dth = (right_m - left_m) / config.TRACK_M
    if abs(dth) < 1e-12:
        return s, 0.0, 0.0
    r = s / dth
    return r * math.sin(dth), r * (1.0 - math.cos(dth)), dth


def plan(segments, speed, counts_per_wheel_rev):
    """Validate a move and turn it into per-wheel count targets. Moves nothing."""
    if not counts_per_wheel_rev or counts_per_wheel_rev <= 0:
        raise ValueError("encoder scale unknown")
    if not isinstance(segments, list) or not segments:
        raise ValueError("at least one segment is required")
    if len(segments) > config.BLIND_MAX_SEGMENTS:
        raise ValueError(f"at most {config.BLIND_MAX_SEGMENTS} segments")
    ref_rpm = _speed_rpm(speed)
    m_per_count = math.pi * config.WHEEL_DIA_M / counts_per_wheel_rev
    half = config.TRACK_M / 2.0
    out = []
    for i, seg in enumerate(segments):
        where = f"segment {i + 1}"
        if not isinstance(seg, dict) or seg.get("kind") not in KINDS:
            raise ValueError(f"{where}: kind must be one of {', '.join(KINDS)}")
        kind = seg["kind"]
        centre = None                   # centre-path length, where there is one
        if kind == "straight":
            d = _num(seg, "distance_m", where)
            left_m = right_m = d
            centre = abs(d)
            spec = {"distance_m": d}
        elif kind == "arc":
            radius = _num(seg, "radius_m", where)
            angle = _num(seg, "angle_deg", where)
            if abs(radius) < half:
                raise ValueError(f"{where}: |radius_m| must be at least half the "
                                 f"track ({half:.3f} m); use a pivot to turn tighter")
            s = abs(radius) * math.radians(angle)
            left_m = s * (1.0 - half / radius)
            right_m = s * (1.0 + half / radius)
            centre = abs(s)
            spec = {"radius_m": radius, "angle_deg": angle}
        elif kind == "pivot":
            angle = _num(seg, "angle_deg", where)
            right_m = half * math.radians(angle)
            left_m = -right_m
            spec = {"angle_deg": angle}
        else:
            try:
                dl, dr = int(seg["left"]), int(seg["right"])
            except (KeyError, TypeError, ValueError):
                raise ValueError(f"{where}: left and right must be whole counts") from None
            left_m = _vehicle(dl, config.INVERT_LEFT) * m_per_count
            right_m = _vehicle(dr, config.INVERT_RIGHT) * m_per_count
            spec = {"left": dl, "right": dr}

        dom = max(abs(left_m), abs(right_m))
        if dom <= 0:
            raise ValueError(f"{where}: moves nothing")
        if dom > config.BLIND_MAX_DISTANCE_M:
            raise ValueError(f"{where}: a wheel would travel {dom:.2f} m, above "
                             f"blind_run.max_distance_m ({config.BLIND_MAX_DISTANCE_M:g} m)")
        # The speed names the centre path where there is one, else the faster wheel.
        dom_rpm = ref_rpm * dom / centre if centre else ref_rpm
        if dom_rpm > config.BLIND_MAX_RPM + 1e-9:
            raise ValueError(f"{where}: the faster wheel would need {dom_rpm:.0f} r/min, "
                             f"above blind_run.max_rpm ({config.BLIND_MAX_RPM:g})")
        if kind == "pulses":
            counts = (dl, dr)
        else:
            counts = (round(_vehicle(left_m / m_per_count, config.INVERT_LEFT)),
                      round(_vehicle(right_m / m_per_count, config.INVERT_RIGHT)))
        dx, dy, dth = pose_delta(left_m, right_m)
        out.append({
            "kind": kind, "spec": spec, "left_m": left_m, "right_m": right_m,
            "counts": list(counts), "dom_rpm": dom_rpm,
            "rpm": [dom_rpm * left_m / dom, dom_rpm * right_m / dom],
            "duration_s": _duration(dom, dom_rpm),
            "commanded": {"distance_m": (left_m + right_m) / 2.0, "dx_m": dx,
                          "dy_m": dy, "heading_deg": math.degrees(dth)},
        })
    return {"segments": out, "ref_rpm": ref_rpm,
            "speed_mps": kinematics.rpm_to_mps(ref_rpm),
            "counts_per_wheel_rev": counts_per_wheel_rev,
            "duration_s": sum(s["duration_s"] for s in out)
                          + config.BLIND_SETTLE_S * len(out)}


class BlindRun:
    """Drive a validated plan on encoder counts alone, one segment at a time.

    The faster wheel of each segment sets the pace on a trapezoid computed from
    its REMAINING counts; the other follows the planned ratio plus a trim on its
    progress mismatch. Each segment ends at rest, and only then are its counts
    recorded - a count taken while rolling would measure the stop, not the move.
    """

    def __init__(self, planned, start_counts):
        self.plan = planned
        self.segments = planned["segments"]
        self._m_per_count = (math.pi * config.WHEEL_DIA_M
                             / planned["counts_per_wheel_rev"])
        self._last = tuple(start_counts)
        self.index = 0
        self.phase = RUNNING
        self.reason = None
        self.pose = [0.0, 0.0, 0.0]
        self.results = []
        self._begin_segment(start_counts)

    def _begin_segment(self, counts):
        self._seg_counts = tuple(counts)
        self._seg_pose = list(self.pose)
        self.progress = [0.0, 0.0]
        self.v = 0.0
        self.seg_elapsed = 0.0
        self._settled_s = 0.0
        self._waited_s = 0.0

    @property
    def segment(self):
        return self.segments[self.index]

    def _integrate(self, counts):
        dl = counts_delta(counts[0], self._last[0])
        dr = counts_delta(counts[1], self._last[1])
        self._last = tuple(counts)
        lm = _vehicle(dl, config.INVERT_LEFT) * self._m_per_count
        rm = _vehicle(dr, config.INVERT_RIGHT) * self._m_per_count
        self.progress[0] += lm
        self.progress[1] += rm
        ds, dth = (lm + rm) / 2.0, (rm - lm) / config.TRACK_M
        mid = self.pose[2] + dth / 2.0
        self.pose[0] += ds * math.cos(mid)
        self.pose[1] += ds * math.sin(mid)
        self.pose[2] += dth

    def abort(self, reason):
        if self.phase not in (DONE, ABORTED):
            self.phase, self.reason = ABORTED, reason
        return 0.0, 0.0

    def update(self, counts, dt, stopped):
        """One tick. `stopped` is the drives' speed-zero verdict (None: unknown).

        Returns (left_rpm, right_rpm) in driver terms.
        """
        if self.phase in (DONE, ABORTED):
            return 0.0, 0.0
        self._integrate(counts)
        self.seg_elapsed += dt
        seg = self.segment
        target = (seg["left_m"], seg["right_m"])
        dom = 0 if abs(target[0]) >= abs(target[1]) else 1
        goal = abs(target[dom])
        done = self.progress[dom] * math.copysign(1.0, target[dom])
        tol = config.BLIND_STOP_TOLERANCE_MM / 1000.0
        # Checked while settling too: a wheel still turning after the stop
        # command is exactly the runaway this exists to catch.
        if done > goal * (1.0 + config.BLIND_OVERRUN_MARGIN) + tol:
            return self.abort(f"segment {self.index + 1} overran: {done:.3f} m "
                              f"of {goal:.3f} m by encoder")

        if self.phase == RUNNING:
            if self.seg_elapsed > TIME_MARGIN * seg["duration_s"] + TIME_SLACK_S:
                return self.abort(f"segment {self.index + 1} exceeded its time "
                                  f"budget at {done:.3f} m of {goal:.3f} m")
            remaining = goal - done
            if remaining <= tol:
                self.phase, self.v = SETTLING, 0.0
                return 0.0, 0.0
            a = kinematics.rpm_to_mps(config.BLIND_ACCEL_RPM_S)
            v_max = kinematics.rpm_to_mps(seg["dom_rpm"])
            self.v = min(v_max, self.v + a * dt, math.sqrt(2.0 * a * remaining))
            frac = max(0.0, done / goal)
            wheel = []
            for w in (0, 1):
                v_w = self.v * target[w] / goal
                if w != dom:
                    v_w += config.BLIND_SYNC_KP * (target[w] * frac - self.progress[w])
                wheel.append(v_w)
            left, right = kinematics.body_to_wheels(
                (wheel[0] + wheel[1]) / 2.0, (wheel[1] - wheel[0]) / config.TRACK_M)
            cap = config.MOTOR_MAX_RPM
            return max(-cap, min(cap, left)), max(-cap, min(cap, right))

        # SETTLING: counts are recorded only once both drives report standstill.
        if stopped:
            self._settled_s += dt
        else:
            self._settled_s = 0.0
            self._waited_s += dt
            if self._waited_s > SETTLE_LIMIT_S:
                return self.abort(f"segment {self.index + 1}: drives did not report "
                                  f"standstill")
        if self._settled_s >= config.BLIND_SETTLE_S:
            self._record(counts)
            if self.index + 1 >= len(self.segments):
                self.phase = DONE
            else:
                self.index += 1
                self.phase = RUNNING
                self._begin_segment(counts)
        return 0.0, 0.0

    def _record(self, counts):
        seg = self.segment
        final = [counts_delta(counts[0], self._seg_counts[0]),
                 counts_delta(counts[1], self._seg_counts[1])]
        # The pose change in the segment's own starting frame, so it compares
        # directly with the plan's commanded dx/dy.
        h0 = self._seg_pose[2]
        dx, dy = self.pose[0] - self._seg_pose[0], self.pose[1] - self._seg_pose[1]
        self.results.append({
            "segment": self.index + 1, "kind": seg["kind"], "spec": seg["spec"],
            "target_counts": list(seg["counts"]), "final_counts": final,
            "error_counts": [final[0] - seg["counts"][0], final[1] - seg["counts"][1]],
            "encoder_left_m": self.progress[0], "encoder_right_m": self.progress[1],
            "encoder_distance_m": (self.progress[0] + self.progress[1]) / 2.0,
            "encoder_heading_deg": math.degrees(self.pose[2] - h0),
            "encoder_dx_m": dx * math.cos(h0) + dy * math.sin(h0),
            "encoder_dy_m": -dx * math.sin(h0) + dy * math.cos(h0),
            "commanded_distance_m": seg["commanded"]["distance_m"],
            "commanded_heading_deg": seg["commanded"]["heading_deg"],
            "commanded_dx_m": seg["commanded"]["dx_m"],
            "commanded_dy_m": seg["commanded"]["dy_m"],
            "duration_s": self.seg_elapsed,
        })

    def snapshot(self):
        seg = self.segment
        return {"phase": self.phase, "reason": self.reason,
                "segment": self.index + 1, "segments": len(self.segments),
                "kind": seg["kind"], "target_m": [seg["left_m"], seg["right_m"]],
                "progress_m": list(self.progress), "speed_mps": self.v,
                "pose": {"x_m": self.pose[0], "y_m": self.pose[1],
                         "heading_deg": math.degrees(self.pose[2])},
                "completed": len(self.results)}
