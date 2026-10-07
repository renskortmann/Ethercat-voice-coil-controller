"""Where the drive and sensor defaults come from, recomputed from the rig logs.

The open-loop logs are the ground truth for timing: the command the PC sent in each cycle can be
rebuilt exactly from the file name (replay.reference_from_log_name), so the delays and gains from
command to reported current and to logged position can be measured without the drive's own echo.

- fit_drive(): gain_dc, gain_hf, shelf_tau_s and delay_ticks of DriveConfig, by least squares in the
  time domain, using the same recursion as drive.Drive.
- frf(): H1 frequency response and coherence between two signals.
- compare_position_frf(): frequency response from command to logged position, rig against simulation.
  Its phase difference sets LaserConfig.delay_s (the only free timing parameter left once the drive is
  fitted), up to the plant's own phase error.
- idle_noise(): sensor noise and mains pickup in a log's 0 A idle window.
"""

import math

import numpy as np
from scipy import optimize, signal

from .loop import DT_S

FS = 1.0 / DT_S


def drive_report(cmd, gain_dc, gain_hf, tau_s, delay_ticks, lag_s=0.0, dt=DT_S):
    """Noise-free reported current for a command sequence, vectorised. Same model as drive.Drive:
    reported(k) = coil current at the end of cycle k-1, driven by the command of cycle k-1-delay_ticks
    through the current-loop lag and the shelf, discretised exactly for a command held over each cycle."""
    cmd = np.asarray(cmd, float)
    lag = delay_ticks + 1
    c_del = np.concatenate([np.zeros(lag), cmd[:-lag]])
    tc, ts = max(lag_s, 1e-9), max(tau_s, 1e-9)
    if abs(tc - ts) < 1e-9:
        tc *= 1.000001
    # states [y, l]: tc y' = c - y, ts l' = y - l; output i = gain_hf y + (gain_dc - gain_hf) l
    A = np.array([[-1 / tc, 0.0], [1 / ts, -1 / ts]])
    B = np.array([[1 / tc], [0.0]])
    Cm = np.array([[gain_hf, gain_dc - gain_hf]])
    Ad, Bd, Cd, _, _ = signal.cont2discrete((A, B, Cm, np.zeros((1, 1))), dt, method="zoh")
    # x_k = Ad x_{k-1} + Bd c_del_k and report_k = Cd x_k, i.e. z * Cd (zI - Ad)^-1 Bd.
    # ss2tf gives Cd (zI - Ad)^-1 Bd = (b1 z + b2) / (z^2 + a1 z + a2), num = [0, b1, b2];
    # times z, in powers of z^-1: (b1 + b2 z^-1) / (1 + a1 z^-1 + a2 z^-2).
    num, den = signal.ss2tf(Ad, Bd, Cd, np.zeros((1, 1)))
    return signal.lfilter(num[0][1:], den, c_del)


def fit_drive(cmd, actual, delays=range(0, 5)):
    """Least-squares fit of the drive model (gain_dc, gain_hf, shelf_tau_s, lag_s) to a measured
    reported current, for each whole-cycle delay. Returns a list of dicts, best (lowest rms) first.

    Only a sweep that starts at low frequency can separate gain_dc from gain_hf: use the plain chirps."""
    out = []
    for d in delays:
        def res(p):
            return drive_report(cmd, p[0], p[1], math.exp(p[2]), d, math.exp(p[3])) - actual
        r = optimize.least_squares(res, [1.0, 0.87, math.log(0.02), math.log(2e-4)],
                                   bounds=([0.5, 0.5, math.log(1e-4), math.log(1e-6)],
                                           [1.5, 1.5, math.log(1.0), math.log(5e-3)]))
        g_dc, g_hf, lts, ltc = r.x
        out.append(dict(delay_ticks=d, gain_dc=float(g_dc), gain_hf=float(g_hf), shelf_tau_s=math.exp(lts),
                        lag_s=math.exp(ltc), rms_A=float(np.sqrt(np.mean(r.fun ** 2)))))
    return sorted(out, key=lambda q: q["rms_A"])


def frf(u, y, fs=FS, nperseg=8192):
    """H1 estimate y/u and coherence, Welch with Hann windows."""
    f, Puu = signal.welch(u, fs, nperseg=nperseg)
    _, Puy = signal.csd(u, y, fs, nperseg=nperseg)
    _, coh = signal.coherence(u, y, fs, nperseg=nperseg)
    return f, Puy / Puu, coh


def compare_position_frf(rig_t, rig_cmd, rig_pos, sim_t, sim_cmd, sim_pos, f_eval=(10, 15, 20, 25, 30, 38, 42, 46)):
    """Command -> position response of rig and simulation at a few frequencies (active part only).

    Returns rows with gain ratio sim/rig and phase difference sim - rig (degrees), and the delay that
    phase difference corresponds to (positive: the simulation lags less than the rig)."""
    rows = []
    fr, Hr, cr = frf(rig_cmd[rig_t >= 0], rig_pos[rig_t >= 0])
    fs_, Hs, cs = frf(sim_cmd[sim_t >= 0], sim_pos[sim_t >= 0])
    for fe in f_eval:
        k = int(np.argmin(np.abs(fr - fe)))
        dph = float(np.degrees(np.angle(Hs[k] / Hr[k])))
        rows.append(dict(f_Hz=fe, gain_rig=abs(Hr[k]), gain_sim=abs(Hs[k]), gain_ratio=abs(Hs[k] / Hr[k]),
                         phase_diff_deg=dph, delay_diff_ms=dph / 360.0 / fe * 1e3, coherence_rig=float(cr[k])))
    return rows


def idle_noise(idle_position_mm, idle_accel_V, idle_current_A, fs=FS):
    """Noise in a 0 A idle window: mains lines at the best-fitting mains frequency, and what is left."""
    y = np.asarray(idle_position_mm, float)
    y = y - y.mean()
    tt = np.arange(len(y)) / fs
    best = None
    for fm in np.arange(49.90, 50.10, 0.005):
        X = np.column_stack([np.sin(2 * np.pi * k * fm * tt) for k in (1, 3)] +
                            [np.cos(2 * np.pi * k * fm * tt) for k in (1, 3)])
        c = np.linalg.lstsq(X, y, rcond=None)[0]
        rr = float(np.std(y - X @ c))
        if best is None or rr < best["white_mm"]:
            best = dict(mains_hz=float(fm), mains_amp_mm=float(np.hypot(c[0], c[2])),
                        mains3_amp_mm=float(np.hypot(c[1], c[3])), white_mm=rr)
    return {**best, "position_std_mm": float(y.std()), "accel_std_V": float(np.std(idle_accel_V)),
            "current_std_A": float(np.std(idle_current_A))}
