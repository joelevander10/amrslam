"""Pure model behind fake_base_node: two wheels with a ramp, a watchdog and slip.

Ground truth integrates the wheels' ACTUAL motion. The reported wheel positions
carry multiplicative slip noise, so odometry drifts from truth the way it does
on a real floor. Fed a clock; owns no timers.
"""

import random
from dataclasses import dataclass, field

from amr_base.diff_drive import Geometry, OdomState, integrate, slew


@dataclass
class WheelModel:
    geom: Geometry
    wheel_accel_rad_s2: float
    cmd_timeout_s: float
    slip_noise_std: float = 0.0
    seed: int = 0

    actual_l: float = 0.0
    actual_r: float = 0.0
    target_l: float = 0.0
    target_r: float = 0.0
    pos_l: float = 0.0  # reported (slip-corrupted) wheel positions
    pos_r: float = 0.0
    truth: OdomState = field(default_factory=OdomState)
    last_cmd_t: float | None = None
    timed_out: bool = True
    _rng: random.Random = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def teleport(self, x: float, y: float, yaw: float) -> None:
        """Move REALITY only (kidnapped robot). Reported wheel positions keep integrating."""
        self.truth = OdomState(x=x, y=y, th=yaw, distance=self.truth.distance)

    def command(self, left_rad_s: float, right_rad_s: float, t: float) -> None:
        self.target_l, self.target_r = left_rad_s, right_rad_s
        self.last_cmd_t = t
        self.timed_out = False

    def step(self, t: float, dt: float) -> None:
        if self.last_cmd_t is None or t - self.last_cmd_t > self.cmd_timeout_s:
            self.target_l = self.target_r = 0.0
            self.timed_out = True
        self.actual_l = slew(self.actual_l, self.target_l, self.wheel_accel_rad_s2, dt)
        self.actual_r = slew(self.actual_r, self.target_r, self.wheel_accel_rad_s2, dt)
        d_l, d_r = self.actual_l * dt, self.actual_r * dt
        self.truth = integrate(self.truth, self.geom, d_l, d_r)
        if self.slip_noise_std > 0.0:
            d_l *= 1.0 + self._rng.gauss(0.0, self.slip_noise_std)
            d_r *= 1.0 + self._rng.gauss(0.0, self.slip_noise_std)
        self.pos_l += d_l
        self.pos_r += d_r
