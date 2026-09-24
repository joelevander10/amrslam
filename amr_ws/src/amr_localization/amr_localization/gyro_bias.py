"""Gyro yaw-rate bias estimation from stationary samples. Pure, clock-fed.

The MLS gyro has a +0.0574 deg/s bias (reconciliation D-3): 3.4 deg/min of
heading drift if fused uncorrected, and the EKF fuses yaw RATE, so it cannot
estimate the bias itself. Spec §4.3: average stationary samples, subtract,
refuse to publish until calibrated.

Stationary is decided by the caller (wheel velocities, not the gyro itself -
a gyro cannot tell a slow turn from its own bias). The estimate is refreshed
whenever the vehicle has been still for window_s, so temperature drift is
tracked between moves without any service call.
"""

from dataclasses import dataclass, field


@dataclass
class BiasEstimator:
    window_s: float = 2.0
    settle_s: float = 0.3  # ignore the first samples after stopping (ramp-down)
    bias: float | None = None
    _still_since: float | None = None
    _sum: float = 0.0
    _n: int = 0
    _n_windows: int = field(default=0)

    @property
    def calibrated(self) -> bool:
        return self.bias is not None

    @property
    def windows(self) -> int:
        return self._n_windows

    def reset(self) -> None:
        self.bias = None
        self._still_since = None
        self._sum, self._n = 0.0, 0

    def update(self, t: float, gyro_z: float, stationary: bool) -> float | None:
        """Feed one raw sample. Returns the corrected rate, or None if uncalibrated."""
        if not stationary:
            self._still_since = None
            self._sum, self._n = 0.0, 0
        else:
            if self._still_since is None:
                self._still_since = t
            elif t - self._still_since >= self.settle_s:
                self._sum += gyro_z
                self._n += 1
                if t - self._still_since >= self.settle_s + self.window_s and self._n > 0:
                    self.bias = self._sum / self._n
                    self._n_windows += 1
                    self._still_since = t  # start the next window
                    self._sum, self._n = 0.0, 0
        if self.bias is None:
            return None
        return gyro_z - self.bias
