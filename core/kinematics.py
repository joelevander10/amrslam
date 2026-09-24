"""Differential-drive geometry: the conversion between body motion and wheels.

*** This file is the seam the whole migration turns on. *** It knows nothing
about what decides where to go - the tape follower that used to sit above it is
gone, and a SLAM pose controller producing (linear velocity, angular velocity)
drops into the same place without touching this file or the motor interface
under it. Keep control logic and sensor knowledge out of here; a shortcut
through this module is the one change that would make that expensive.

The dimensions themselves live in the profile (vehicle section) and reach
here through config, which also does the deriving - MPS_PER_RPM and friends are
computed from wheel diameter, gearing and track so the profile cannot hold a
conversion factor that disagrees with the geometry it came from.
"""
import config


def max_yaw_accel(driver_accel_rpm_s):
    """rad/s^2 available for steering, given the 6083h/6084h setting.

    Yaw acceleration is capped by how fast the drivers will slew the wheel
    difference, which is two wheels each ramping at 6083h. This is not a spare
    fact: it is the binding constraint on steering authority, and it is a
    property of THESE DRIVES rather than of whatever is steering.

    *** Derive the limit from here; never hardcode one. *** A controller that
    commands a yaw rate the drives cannot slew to diverges, and a later change
    to drivers.ramp.auto.accel would silently make a hardcoded limit wrong.
    """
    return 2.0 * driver_accel_rpm_s * config.RAD_S_PER_RPM_DIFF


def body_to_wheels(v_mps, omega_rad_s):
    """(forward speed, yaw rate) -> (left, right) motor r/min in DRIVER terms.

    No clamping. Saturation is a control decision - the caller has to scale both
    wheels together to keep the turn ratio, and only it knows the base speed.
    """
    diff_rpm = omega_rad_s / config.RAD_S_PER_RPM_DIFF
    base_rpm = v_mps * config.RPM_PER_MPS
    left = base_rpm - diff_rpm / 2.0
    right = base_rpm + diff_rpm / 2.0
    return (-left if config.INVERT_LEFT else left,
            -right if config.INVERT_RIGHT else right)


def wheels_to_body(left_rpm, right_rpm):
    """Exact inverse of body_to_wheels(). For odometry and diagnostics."""
    left = -left_rpm if config.INVERT_LEFT else left_rpm
    right = -right_rpm if config.INVERT_RIGHT else right_rpm
    return ((left + right) / 2.0 * config.MPS_PER_RPM,
            (right - left) * config.RAD_S_PER_RPM_DIFF)


def rpm_to_mps(rpm):
    return rpm * config.MPS_PER_RPM


def mps_to_rpm(mps):
    return mps * config.RPM_PER_MPS
