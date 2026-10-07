"""Laser position and accelerometer, as the controller receives them through the drive's analog inputs.

Both sensors see the true motion `delay_s` ago (see calibration.py and vca_sim_validation.ipynb for
where the numbers come from), plus noise, and are read through a 12-bit-like
analog input with a step of 1/819.2 V. The laser value is converted to mm with the firmware formula,
so simulated positions land on the same 8 um lattice as the rig's.
"""

import math
from dataclasses import dataclass, asdict

AI_V_PER_COUNT = 1.0 / 819.2     # drive analog input step (main.h AI scaling)
AI1_MM_PER_V = 6.568             # laser calibration (main.h AI1_MM_SCALE)
AI1_OFFSET_MM = 22.5             # main.h AI1_MM_OFFSET
AI1_CENTRE_MM = 51.7             # laser distance at the centre (main.h AI1_CENTRE_MM)
G = 9.81


def position_mm_to_volts(y_mm):
    """Inverse of the firmware conversion position_mm = -(6.568 V + 22.5 - 51.7)."""
    return (AI1_CENTRE_MM - AI1_OFFSET_MM - y_mm) / AI1_MM_PER_V


def volts_to_position_mm(v):
    return -(AI1_MM_PER_V * v + AI1_OFFSET_MM - AI1_CENTRE_MM)


@dataclass(frozen=True)
class LaserConfig:
    delay_s: float = 1.15e-3         # calibration.compare_position_frf, greybox_C, 2 Oct chirps: 1.10-1.20 ms at 38-46 Hz
    quantise: bool = True            # 1.22 mV analog input step = 8 um
    noise_mm: float = 0.024          # white noise std left after removing mains lines (0 A idle windows, 2 Oct)
    mains_hz: float = 50.0           # 49.99-50.05 Hz measured
    mains_amp_mm: float = 0.092      # 50 Hz pickup amplitude (idle windows: 89-94 um)
    mains3_amp_mm: float = 0.049     # 150 Hz pickup amplitude (idle windows: 48-50 um)
    saturate: bool = True
    max_mm: float = 16.44            # laser out of range above this (4 mA end; vca_system_identification cell 6)
    min_mm: float = -33.3            # 20 mA end of the 4-20 mA loop through 476 ohm (beyond the mechanics)

    def describe(self):
        return asdict(self)


@dataclass(frozen=True)
class AccelConfig:
    delay_s: float = 1.25e-3         # logged 2.0 ms after position (TAU_ACC - TAU); the 78 Hz low-pass gives about 1.9 ms of that in band
    lowpass_hz: float = 78.0         # vca_inband_35_55Hz cell 26: first-order roll-off near 78 Hz
    scale_V_per_g: float = 0.0578 * 0.65   # vca_greybox_fit ACC_SCALE_V_PER_G; still 1.1-1.24 off: unresolved
    bias_V: float = 0.655            # idle-window offset (drifts run to run: 0.651-0.657 V on 2 Oct)
    noise_V: float = 0.0054          # idle-window std (2 Oct)
    quantise: bool = True

    def describe(self):
        return asdict(self)


NOISE_FREE_LASER = LaserConfig(quantise=False, noise_mm=0.0, mains_amp_mm=0.0, mains3_amp_mm=0.0, saturate=False)
NOISE_FREE_ACCEL = AccelConfig(noise_V=0.0, quantise=False)


class Laser:
    def __init__(self, cfg: LaserConfig, white, mains_phase, mains3_phase, dt, t0):
        self.cfg = cfg
        self.white = white               # pre-generated list, one value per tick
        self.w1 = 2 * math.pi * cfg.mains_hz
        self.ph1, self.ph3 = mains_phase, mains3_phase
        self.dt, self.t0 = dt, t0

    def read(self, x_mm, k):
        """Logged position_mm for cycle k, given the (delayed) true position in mm."""
        cfg = self.cfg
        t = self.t0 + k * self.dt
        y = (x_mm + self.white[k] + cfg.mains_amp_mm * math.sin(self.w1 * t + self.ph1)
             + cfg.mains3_amp_mm * math.sin(3 * self.w1 * t + self.ph3))
        if cfg.saturate:
            y = min(cfg.max_mm, max(cfg.min_mm, y))
        if cfg.quantise:
            v = round(position_mm_to_volts(y) / AI_V_PER_COUNT) * AI_V_PER_COUNT
            y = volts_to_position_mm(v)
        return y


class Accelerometer:
    """AI2 volts: bias + scale * low-passed delayed acceleration + noise."""

    def __init__(self, cfg: AccelConfig, white, dt):
        self.cfg = cfg
        self.white = white
        self.alpha = 1.0 - math.exp(-dt * 2 * math.pi * cfg.lowpass_hz) if cfg.lowpass_hz > 0 else 1.0
        self.y = 0.0                      # low-pass state, m/s^2
        self.v_per_ms2 = cfg.scale_V_per_g / G

    def read(self, a_ms2, k):
        cfg = self.cfg
        self.y += self.alpha * (a_ms2 - self.y)
        v = cfg.bias_V + self.v_per_ms2 * self.y + self.white[k]
        if cfg.quantise:
            v = round(v / AI_V_PER_COUNT) * AI_V_PER_COUNT
        return v
