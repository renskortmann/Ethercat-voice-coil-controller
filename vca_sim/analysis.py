"""Measurements on simulated runs, all done by simulation so they work for any (black-box) controller.

- tracking_metrics(): error and current statistics of one run.
- sine_fit(): amplitude and phase of one frequency in a signal (least squares).
- tracking_response(): stepped sines; amplitude and phase of the position against the reference per
  frequency, on the true and on the measured position, with a convergence flag.
- loop_gain(): break the loop at the controller output by injecting a multisine there, in two runs with
  identical noise (with and without the injection); the differences give the loop gain L, sensitivity S
  and complementary sensitivity T at the injected frequencies, and from L the stability margins.
- msd_pi_loop_gain(): the analytic L of the PI on a linear plant, for checking loop_gain().
"""

import math
from dataclasses import dataclass, replace

import numpy as np

from . import references as R
from .loop import simulate


# ---------------------------------------------------------------- one run

def tracking_metrics(res, settle_s=0.0):
    """Error of the true and the measured position against the reference, and current use, for t >= settle_s.
    Position references only (for a current reference the error has no meaning)."""
    t = res.t
    m = t >= settle_s
    out = {"status": res.status, "duration_s": float(t[-1]) if len(t) else 0.0}
    if not m.any():
        return out
    if res.ref_unit == "mm":
        e_true = res["ref"][m] - res["x_true_mm"][m]
        e_meas = res["ref"][m] - res["position_mm"][m]
        out.update(rms_error_true_mm=float(np.sqrt(np.mean(e_true ** 2))),
                   max_error_true_mm=float(np.abs(e_true).max()),
                   rms_error_measured_mm=float(np.sqrt(np.mean(e_meas ** 2))))
    u = res["u_cmd_A"][m]
    out.update(rms_current_A=float(np.sqrt(np.mean(u ** 2))), peak_current_A=float(np.abs(u).max()),
               max_abs_position_mm=float(np.abs(res["x_true_mm"][m]).max()))
    return out


def sine_fit(t, y, f_hz):
    """Least-squares fit y = a sin(wt) + b cos(wt) + c. Returns (amplitude, phase in rad, offset)."""
    w = 2 * math.pi * f_hz
    X = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones(len(t))])
    a, b, c = np.linalg.lstsq(X, y, rcond=None)[0]
    return math.hypot(a, b), math.atan2(b, a), c


# ---------------------------------------------------------------- tracking per frequency

def tracking_response(scenario, freqs_hz, amplitude_mm=0.5, offset_mm=0.0, duration_s=3.0, window_periods=20,
                      tol_amp=0.01, tol_phase_deg=1.0):
    """Stepped sines: one run per frequency with a sine position reference.

    The run is cut into windows of window_periods periods from the end backwards; the result is the last
    window, and `converged` says whether it agrees with the one before it within the tolerances (so a
    slowly settling or learning controller is flagged rather than reported as finished).
    Also reports the current the plant's mass alone would need at that frequency (m w^2 A / Kf), the
    floor for any controller, against the drive's peak current.
    """
    rows = []
    ss = scenario.plant.small_signal()
    for f in freqs_hz:
        sc = replace(scenario, reference=R.Sine(offset_mm, amplitude_mm, float(f)), duration_s=duration_s)
        res = simulate(sc)
        row = dict(f_Hz=float(f), status=res.status,
                   mass_line_current_A=scenario.plant.params["m"] * (2 * math.pi * f) ** 2 * amplitude_mm * 1e-3
                   / ss["Kf_N_per_A"])
        if not res.ok:
            rows.append(row)
            continue
        t = res.t
        W = window_periods / f
        fits = {}
        for name in ("ref", "x_true_mm", "position_mm"):
            last = (t >= t[-1] - W)
            prev = (t >= t[-1] - 2 * W) & (t < t[-1] - W)
            fits[name] = [sine_fit(t[mk], res[name][mk], f) for mk in (prev, last)]
        (ra0, rp0, _), (ra1, rp1, _) = fits["ref"]
        for name, tag in (("x_true_mm", "true"), ("position_mm", "measured")):
            (a0, p0, _), (a1, p1, _) = fits[name]
            row[f"gain_{tag}"] = a1 / ra1
            row[f"phase_{tag}_deg"] = math.degrees(math.remainder(p1 - rp1, 2 * math.pi))
            if tag == "true":
                d_amp = abs(a1 / ra1 - a0 / ra0)
                d_ph = abs(math.degrees(math.remainder((p1 - rp1) - (p0 - rp0), 2 * math.pi)))
                row["converged"] = bool(d_amp <= tol_amp and d_ph <= tol_phase_deg)
        last = t >= t[-1] - W
        row["peak_current_A"] = float(np.abs(res["u_cmd_A"][last]).max())
        rows.append(row)
    return rows


# ---------------------------------------------------------------- loop gain by injection

@dataclass
class LoopGain:
    f_hz: np.ndarray
    L: np.ndarray
    S: np.ndarray
    T: np.ndarray
    floor: np.ndarray          # |dU_p| on nearby non-excited lines, relative to the excited ones (distortion + noise)
    periodic: bool             # the differences repeat from period to period (False: unstable or still settling)
    status: str
    margins: dict


def multisine(n_ticks, dt, period_s, lines, rms_A, seed=0):
    """Sum of sines at harmonic numbers `lines` of 1/period_s, random phases, total rms rms_A."""
    rng = np.random.default_rng(seed + 12345)
    t = np.arange(n_ticks) * dt
    amp = rms_A * math.sqrt(2.0 / len(lines))
    ph = rng.uniform(0, 2 * math.pi, len(lines))
    d = np.zeros(n_ticks)
    for k, p in zip(lines, ph):
        d += amp * np.sin(2 * math.pi * k / period_s * t + p)
    return d


def excited_lines(period_s, dt, n_lines=60, f_min_hz=None, f_max_hz=None):
    """Odd harmonics of 1/period_s, roughly log-spaced from f_min to f_max (default: 1 / period to 90 % of Nyquist)."""
    kmax = int((f_max_hz or 0.9 / (2 * dt)) * period_s)
    kmin = max(1, int(round((f_min_hz or 1.0 / period_s) * period_s)))
    cand = np.unique(np.round(np.geomspace(kmin, kmax, n_lines * 3)).astype(int))
    odd = np.unique(cand | 1)                                  # force odd
    odd = odd[(odd >= kmin) & (odd <= kmax)]
    if len(odd) > n_lines:
        odd = odd[np.round(np.linspace(0, len(odd) - 1, n_lines)).astype(int)]
    return np.unique(odd)


def loop_gain(scenario, rms_A=0.05, period_s=1.0, n_settle=2, n_measure=4, n_lines=60,
              f_min_hz=None, f_max_hz=None):
    """Loop gain at the controller output by two-run differencing.

    u_c is the controller output and u_p = u_c + d what is sent on. With D the injected spectrum and
    dU the difference between the run with and without injection (same seed, so the same noise):
        L = -dU_c / dU_p,   S = dU_p / D,   T = -dU_c / D.
    The one-cycle delay, the drive and the sensors are all inside L. For a nonlinear plant or controller
    this is the linearisation along the run's trajectory; make period_s a multiple of the reference
    period, and compare a few rms_A values: curves that differ mean L depends on amplitude.
    The injection starts at t = 0 (after the idle window) and the run lasts n_settle + n_measure periods.
    """
    dt = scenario.dt_s
    n_per = int(round(period_s / dt))
    total = (n_settle + n_measure) * period_s
    sc0 = replace(scenario, duration_s=total, injection_A=None)
    n_idle = int(round(scenario.idle_s / dt))
    lines = excited_lines(period_s, dt, n_lines, f_min_hz, f_max_hz)
    d_active = multisine(sc0.n_ticks - n_idle, dt, period_s, lines, rms_A, scenario.seed)
    d = np.r_[np.zeros(n_idle), d_active]
    r0 = simulate(sc0)
    r1 = simulate(replace(sc0, injection_A=d))
    f = lines / period_s
    if not (r0.ok and r1.ok):
        return LoopGain(f, *(np.full(len(f), np.nan + 0j) for _ in range(3)), np.full(len(f), np.nan), False,
                        f"run ended early: {r1.message or r0.message}", {})

    def spectra(x):
        """FFT of each measurement period, at every harmonic of 1/period."""
        seg = x[n_idle + n_settle * n_per: n_idle + (n_settle + n_measure) * n_per].reshape(n_measure, n_per)
        return np.fft.rfft(seg, axis=1)

    dUc = spectra(r1["u_ctrl_A"] - r0["u_ctrl_A"])
    dUp = spectra(r1["u_cmd_A"] - r0["u_cmd_A"])
    D = spectra(d)
    Uc, Up, Dm = dUc.mean(0), dUp.mean(0), D.mean(0)
    # periodicity: spread of the per-period spectra on the excited lines, relative to their mean
    spread = np.abs(dUp[:, lines] - Up[lines]).max() / (np.abs(Up[lines]).mean() + 1e-30)
    periodic = bool(spread < 0.05)
    L = -Uc[lines] / Up[lines]
    S = Up[lines] / Dm[lines]
    T = -Uc[lines] / Dm[lines]
    # floor: the even line next to each excited odd line carries no injection
    neigh = np.minimum(lines + 1, n_per // 2)
    floor = np.abs(Up[neigh]) / np.abs(Up[lines])
    status = "ok" if periodic else f"differences not periodic (spread {spread:.2f}): unstable or not settled"
    return LoopGain(f, L, S, T, floor, periodic, status, margins(f, L, S))


def margins(f, L, S=None, max_gain_margin_dB=40.0):
    """Crossovers of |L| = 1 with phase and delay margins, phase crossovers with gain margins, and the
    modulus margin 1 / max|S|. Interpolated between the measured lines (log frequency).

    Phase crossovers with more than max_gain_margin_dB of gain margin are left out: there |L| is below
    1 %, and with noise on the measurement its phase wanders across -180 degrees without meaning."""
    out = {"crossovers": [], "phase_crossovers": []}
    mag, ph = np.abs(L), np.angle(L)
    for k in range(len(f) - 1):
        if (mag[k] - 1) * (mag[k + 1] - 1) < 0:
            w = math.log(mag[k]) / (math.log(mag[k]) - math.log(mag[k + 1]))
            fc = math.exp(math.log(f[k]) + w * (math.log(f[k + 1]) - math.log(f[k])))
            dph = math.remainder(ph[k + 1] - ph[k], 2 * math.pi)
            phc = ph[k] + w * dph
            pm = math.degrees(math.remainder(phc + math.pi, 2 * math.pi))
            out["crossovers"].append(dict(f_Hz=fc, phase_margin_deg=pm,
                                          delay_margin_ms=(math.radians(pm) / (2 * math.pi * fc) * 1e3) if pm > 0 else 0.0))
        # phase crossover: L crosses the negative real axis
        if L[k].imag * L[k + 1].imag < 0 and (L[k].real < 0 or L[k + 1].real < 0):
            w = L[k].imag / (L[k].imag - L[k + 1].imag)
            re = L[k].real + w * (L[k + 1].real - L[k].real)
            if re < 0 and -20 * math.log10(-re) <= max_gain_margin_dB:
                fpc = math.exp(math.log(f[k]) + w * (math.log(f[k + 1]) - math.log(f[k])))
                out["phase_crossovers"].append(dict(f_Hz=fpc, gain_margin_dB=-20 * math.log10(-re)))
    if S is not None and np.all(np.isfinite(S)):
        k = int(np.argmax(np.abs(S)))
        out["modulus_margin"] = float(1 / np.abs(S[k]))
        out["max_S_f_Hz"] = float(f[k])
    return out


def msd_pi_loop_gain(f_hz, plant_params, Kp, Ki, dt, drive_delay_ticks=0, sensor_delay_s=0.0, n_alias=400):
    """Analytic loop gain of the firmware PI (A/mm) on a linear plant Kf / (m s^2 + c s + k), with the
    engine's timing: one-cycle delay, drive_delay_ticks more, zero-order hold, sensor delay, sampling.

    The sampled plant is the alias sum (1/T) sum_n ZOH(j w_n) G(j w_n) exp(-j w_n tau), w_n = w + n 2 pi / T.
    """
    m, c, k, Kf = plant_params["m"], plant_params["c"], plant_params["k1"], plant_params["kf0"]
    out = []
    ws = 2 * math.pi / dt
    n = np.arange(-n_alias, n_alias + 1)
    for f in np.atleast_1d(f_hz):
        w = 2 * math.pi * f
        wn = w + n * ws
        s = 1j * wn
        zoh = (1 - np.exp(-s * dt)) / s
        G = Kf / (m * s ** 2 + c * s + k) * 1e3                     # mm per A
        P = np.sum(zoh * G * np.exp(-s * sensor_delay_s)) / dt
        z = np.exp(1j * w * dt)
        C = Kp + Ki * dt / (1 - 1 / z)                               # I += Ki e dt; u = P + I (this cycle's e)
        out.append(C * z ** -(1 + drive_delay_ticks) * P)
    return np.array(out)
