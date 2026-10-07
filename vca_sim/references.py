"""Reference signals, evaluated once for the whole run before the loop starts.

A reference has a unit: "mm" for a position reference or "A" for a current (open-loop playback).
Each spec returns the value and its first two time derivatives, plus the instantaneous frequency
(NaN when the signal is not periodic), all as arrays over the run's time axis. Before t = 0 (the idle
window) every reference holds its t = 0 value with zero derivatives.

The shapes copy the firmware so simulated and rig logs overlay: exponential chirps with the same
phase law, the position chirp ending at a zero crossing, the scheduled chirp's amplitude ramps, and
the sine blocks.
"""

import math
from dataclasses import dataclass, field, asdict

import numpy as np


@dataclass(frozen=True)
class RefSignal:
    unit: str
    r: np.ndarray
    r_dot: np.ndarray
    r_ddot: np.ndarray
    freq_hz: np.ndarray


def _hold_idle(t, r, rd, rdd, f):
    """Before t = 0: hold the value at t = 0, zero derivatives."""
    idle = t < 0
    if idle.any():
        first = np.argmax(~idle) if (~idle).any() else 0
        r = np.where(idle, r[first], r)
        rd, rdd = np.where(idle, 0.0, rd), np.where(idle, 0.0, rdd)
        f = np.where(idle, np.nan, f)
    return r, rd, rdd, f


def _exp_chirp_phase(tau, f0, f1, T):
    """Phase of f(t) = f0 (f1/f0)^(t/T): 2 pi f0 L (exp(t/L) - 1), L = T / ln(f1/f0) (firmware exp_chirp_phase)."""
    L = T / math.log(f1 / f0)
    return 2 * math.pi * f0 * L * (np.exp(tau / L) - 1.0), L


@dataclass(frozen=True)
class Constant:
    value: float = 0.0
    unit: str = "mm"

    def evaluate(self, t):
        z = np.zeros_like(t)
        return RefSignal(self.unit, np.full_like(t, self.value), z, z.copy(), np.full_like(t, np.nan))


@dataclass(frozen=True)
class Steps:
    """Breakpoints (t_s, value): linear ramp of ramp_s to each new value, then hold (firmware POS_REF_STEPS)."""
    points: tuple = ((0.0, 0.0), (1.0, 1.0), (3.0, -1.0), (5.0, 0.0))
    ramp_s: float = 0.0
    unit: str = "mm"

    def evaluate(self, t):
        r, rd = np.full_like(t, self.points[0][1]), np.zeros_like(t)
        for (t_prev, v_prev), (t_i, v_i) in zip(self.points[:-1], self.points[1:]):
            if self.ramp_s > 0:
                into = t - t_i
                ramp = (into >= 0) & (into < self.ramp_s)
                r = np.where(ramp, v_prev + (v_i - v_prev) * into / self.ramp_s, r)
                rd = np.where(ramp, (v_i - v_prev) / self.ramp_s, rd)
                r = np.where(into >= self.ramp_s, v_i, r)
            else:
                r = np.where(t >= t_i, v_i, r)
        return RefSignal(self.unit, *_hold_idle(t, r, rd, np.zeros_like(t), np.full_like(t, np.nan)))


@dataclass(frozen=True)
class Sine:
    offset: float = 0.0
    amplitude: float = 1.0
    freq_hz: float = 45.0
    unit: str = "mm"

    def evaluate(self, t):
        w = 2 * math.pi * self.freq_hz
        s, c = np.sin(w * t), np.cos(w * t)
        r = self.offset + self.amplitude * s
        return RefSignal(self.unit, *_hold_idle(t, r, self.amplitude * w * c, -self.amplitude * w * w * s,
                                                np.full_like(t, self.freq_hz)))


@dataclass(frozen=True)
class Chirp:
    """Exponential sweep f0 -> f1 over duration_s, starting at start_s.

    stop = "zero_crossing": run on to the next zero crossing, then hold the offset (position chirp).
    stop = "hard": drop to the offset at duration_s (the firmware's current chirp).
    """
    offset: float = 0.0
    amplitude: float = 1.0
    f0_hz: float = 35.0
    f1_hz: float = 55.0
    duration_s: float = 10.0
    start_s: float = 0.0
    stop: str = "zero_crossing"
    unit: str = "mm"

    def end_s(self):
        """Sweep time at which the sweep stops (firmware pos_ref_chirp_end_s)."""
        if self.stop == "hard":
            return self.duration_s
        ph_T, L = _exp_chirp_phase(np.array(self.duration_s), self.f0_hz, self.f1_hz, self.duration_s)
        ph_end = math.pi * math.ceil(float(ph_T) / math.pi)
        return L * math.log(1.0 + ph_end / (2 * math.pi * self.f0_hz * L))

    def evaluate(self, t):
        tau = t - self.start_s
        ph, L = _exp_chirp_phase(tau, self.f0_hz, self.f1_hz, self.duration_s)
        f = self.f0_hz * np.exp(tau / L)
        w, wd = 2 * math.pi * f, 2 * math.pi * f / L        # phase rate and its derivative
        A = self.amplitude
        on = (tau >= 0) & (tau < self.end_s())
        r = np.where(on, self.offset + A * np.sin(ph), self.offset)
        rd = np.where(on, A * w * np.cos(ph), 0.0)
        rdd = np.where(on, A * (wd * np.cos(ph) - w * w * np.sin(ph)), 0.0)
        return RefSignal(self.unit, *_hold_idle(t, r, rd, rdd, np.where(on, f, np.nan)))


@dataclass(frozen=True)
class ScheduledChirp:
    """Chirp with an amplitude schedule (firmware EXPERIMENT_CHIRP_SCHEDULED): breakpoints (t_s, amplitude),
    each ramped in linearly over ramp_s; hard stop at duration_s."""
    schedule: tuple = ((0.0, 2.0), (30.0, 6.0), (60.0, 15.0))
    ramp_s: float = 30.0
    f0_hz: float = 10.0
    f1_hz: float = 55.0
    duration_s: float = 120.0
    unit: str = "A"

    def amplitude(self, t):
        a = np.full_like(t, self.schedule[0][1])
        for (t_prev, a_prev), (t_i, a_i) in zip(self.schedule[:-1], self.schedule[1:]):
            into = t - t_i
            if self.ramp_s > 0:
                a = np.where((into >= 0) & (into < self.ramp_s), a_prev + (a_i - a_prev) * into / self.ramp_s, a)
                a = np.where(into >= self.ramp_s, a_i, a)
            else:
                a = np.where(into >= 0, a_i, a)
        return a

    def evaluate(self, t):
        ph, L = _exp_chirp_phase(t, self.f0_hz, self.f1_hz, self.duration_s)
        on = (t >= 0) & (t < self.duration_s)
        r = np.where(on, self.amplitude(t) * np.sin(ph), 0.0)
        f = np.where(on, self.f0_hz * np.exp(t / L), np.nan)
        rd = np.gradient(r, t) if len(t) > 1 else np.zeros_like(t)   # envelope ramps are slow: numerical is fine
        rdd = np.gradient(rd, t) if len(t) > 1 else np.zeros_like(t)
        return RefSignal(self.unit, *_hold_idle(t, r, rd, rdd, f))


@dataclass(frozen=True)
class SineBlocks:
    """Blocks of (freq_hz, amplitude): ramp in, hold, ramp out, pause (firmware EXPERIMENT_SINE_BLOCKS)."""
    blocks: tuple = ((35.0, 1.0), (45.0, 1.0), (55.0, 1.0))
    ramp_s: float = 0.5
    hold_s: float = 2.0
    period_s: float = 4.0
    unit: str = "mm"

    def evaluate(self, t):
        tt = np.maximum(t, 0.0)
        k = (tt // self.period_s).astype(int)
        tb = tt - k * self.period_s
        valid = k < len(self.blocks)
        kk = np.minimum(k, len(self.blocks) - 1)
        f = np.array([b[0] for b in self.blocks])[kk]
        A = np.array([b[1] for b in self.blocks])[kk]
        R, H = self.ramp_s, self.hold_s
        env = np.select([tb < R, tb < R + H, tb < 2 * R + H], [tb / R if R > 0 else 1.0, 1.0, (2 * R + H - tb) / R if R > 0 else 0.0], 0.0)
        on = valid & (tb < 2 * R + H)
        r = np.where(valid, A * env * np.sin(2 * math.pi * f * tb), 0.0)
        rd = np.gradient(r, t) if len(t) > 1 else np.zeros_like(t)
        rdd = np.gradient(rd, t) if len(t) > 1 else np.zeros_like(t)
        return RefSignal(self.unit, *_hold_idle(t, r, rd, rdd, np.where(on, f, np.nan)))


@dataclass(frozen=True)
class FromArray:
    """A reference given as samples (e.g. a column of a rig log), linearly interpolated onto the run's time axis."""
    t_s: tuple = (0.0, 1.0)
    values: tuple = (0.0, 0.0)
    unit: str = "mm"
    label: str = "array"

    def evaluate(self, t):
        r = np.interp(t, np.asarray(self.t_s), np.asarray(self.values))
        rd = np.gradient(r, t) if len(t) > 1 else np.zeros_like(t)
        rdd = np.gradient(rd, t) if len(t) > 1 else np.zeros_like(t)
        return RefSignal(self.unit, r, rd, rdd, np.full_like(t, np.nan))


REFERENCE_TYPES = {cls.__name__: cls for cls in (Constant, Steps, Sine, Chirp, ScheduledChirp, SineBlocks, FromArray)}


def describe(spec):
    """Settings of a reference as a plain dict (arrays of FromArray summarised by length)."""
    d = asdict(spec)
    if isinstance(spec, FromArray):
        d = {"label": spec.label, "unit": spec.unit, "n_samples": len(spec.values)}
    return {"type": type(spec).__name__, **d}
