"""Rig logs in and out: rebuild open-loop experiments from a log, replay measured current through a
plant model, reproduce the greybox notebook's evaluation, and write simulated runs as rig CSVs.
"""

import re
from pathlib import Path

import numpy as np

from . import references as R
from .loop import Scenario, HostConfig, DT_S
from .plant import M_KG

LOG_HEADER = ("time_s,actual_current_A,target_current_A,demand_current_A, ai1_value_V, ai2_value_V,"
              "dc_bus_voltage_V,power_W,energy_J, cycle_jitter_us, pdo_exchange_us,position_mm,ai2_corrected_V")
COIL_R_OHM = 4.75   # identification plan cell 2; only used for the power_W / energy_J columns of exports


# ---------------------------------------------------------------- open-loop experiments from log names

def reference_from_log_name(path):
    """The current the PC commanded in an open-loop log, rebuilt from the experiment tag in its file name.

    Knows the plain chirp (`chirp_6.5A_1.0to55.0Hz_120.0s`) and the scheduled chirp
    (`chirpsched_0.0s2.0A-30.0s6.0A-60.0s15.0A-r30.0s_10.0to55.0Hz_120.0s`). Returns None otherwise.
    """
    name = Path(path).name
    m = re.search(r"_chirp_([\d.]+)A_([\d.]+)to([\d.]+)Hz_([\d.]+)s_", name)
    if m:
        A, f0, f1, T = map(float, m.groups())
        return R.Chirp(0.0, A, f0, f1, T, 0.0, stop="hard", unit="A")
    m = re.search(r"_chirpsched_(.+)-r([\d.]+)s_([\d.]+)to([\d.]+)Hz_([\d.]+)s_", name)
    if m:
        table = tuple((float(a), float(b)) for a, b in re.findall(r"([\d.]+)s([\d.]+)A", m.group(1)))
        ramp, f0, f1, T = map(float, m.groups()[1:])
        return R.ScheduledChirp(table, ramp, f0, f1, T, unit="A")
    return None


def scenario_from_log(path, controller, plant, duration_s=None, **changes):
    """A scenario that repeats an open-loop rig experiment through the whole simulated chain.

    `controller` is the open-loop playback class (controllers/open_loop_current.py). Idle window and
    duration are taken from the log. The firmware has no position trip in current mode, so it is off.
    """
    import pandas as pd
    ref = reference_from_log_name(path)
    if ref is None:
        raise ValueError(f"cannot rebuild the command from the file name {Path(path).name}")
    t = pd.read_csv(path, usecols=[0]).iloc[:, 0].to_numpy()
    idle_s = max(0.0, -t.min())
    dur = float(t.max()) + DT_S if duration_s is None else duration_s
    base = dict(controller=controller, reference=ref, plant=plant, duration_s=dur, idle_s=round(idle_s, 4),
                host=HostConfig(trip_enable=False))
    return Scenario(**{**base, **changes})


# ---------------------------------------------------------------- plant driven by measured current

def replay_current(plant, t, i_A, x0_mm=None, substeps=1):
    """True position (mm) of the plant driven by a measured current, held over each sample (ZOH).

    No drive or sensor model: this is the plant alone, as the notebooks evaluate it.
    """
    acc = plant.make_acc()
    dt = float(np.median(np.diff(t)))
    h = dt / substeps
    x = plant.equilibrium_m() if x0_mm is None else x0_mm * 1e-3
    v = 0.0
    out = np.empty(len(i_A))
    for k, i in enumerate(np.asarray(i_A, float).tolist()):
        out[k] = x
        for _ in range(substeps):
            k1x, k1v = v, acc(x, v, i)
            k2x, k2v = v + 0.5 * h * k1v, acc(x + 0.5 * h * k1x, v + 0.5 * h * k1v, i)
            k3x, k3v = v + 0.5 * h * k2v, acc(x + 0.5 * h * k2x, v + 0.5 * h * k2v, i)
            k4x, k4v = v + h * k3v, acc(x + h * k3x, v + h * k3v, i)
            x += h / 6 * (k1x + 2 * k2x + 2 * k3x + k4x)
            v += h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
    return out * 1e3


# ---------------------------------------------------------------- the greybox notebook's evaluation

GB_TS, GB_TAU, GB_DECIM = 1e-3, 0.65e-3, 2   # vca_greybox_fit.ipynb cell 1


def load_greybox(path):
    """Same preprocessing as vca_greybox_fit.ipynb load(): 100 Hz zero-phase low-pass, position shifted
    back by TAU and notched at 50/150 Hz, idle window dropped, decimated to 1 kHz."""
    from scipy import signal
    fs = 2000.0
    b, a = signal.butter(4, 100.0 / (fs / 2))
    raw = np.loadtxt(path, delimiter=",", skiprows=2)
    t = raw[:, 0]
    lp = lambda s: signal.filtfilt(b, a, s)
    x = np.interp(t + GB_TAU, t, lp(raw[:, 11] * 1e-3))
    for f_n in (50.0, 150.0):
        x = signal.filtfilt(*signal.iirnotch(f_n, 30.0, fs=fs), x)
    keep = slice(np.searchsorted(t, 0.0), None, GB_DECIM)
    d = dict(t=t[keep], i=lp(raw[:, 1])[keep], x=x[keep])
    d["v"] = np.gradient(d["x"], GB_TS)
    m = re.search(r"_([\d.]+)to([\d.]+)Hz_([\d.]+)s_", Path(path).name)
    f0, f1, T = map(float, m.groups()) if m else (np.nan,) * 3
    d["f_inst"] = f0 * (f1 / f0) ** (d["t"] / T)
    d["sweep"] = d["f_inst"] <= 55.0
    return d


def rollout_nrmse(plant, d, band, H=1000, stride=10):
    """Position NRMSE against prediction horizon, H-step RK4 rollouts (1 ms, ZOH) from the measured
    state, every stride-th start in the band: the notebook's nrmse_curve()."""
    acc = plant.make_acc()   # plain arithmetic, so it also works on arrays of starts
    s = np.flatnonzero(band)[::stride]
    s = s[s + H < len(d["x"])]
    x, v = d["x"][s].copy(), d["v"][s].copy()
    err = [np.zeros(len(s))]
    h = GB_TS
    for k in range(H):
        u = d["i"][s + k]
        k1x, k1v = v, acc(x, v, u)
        k2x, k2v = v + h / 2 * k1v, acc(x + h / 2 * k1x, v + h / 2 * k1v, u)
        k3x, k3v = v + h / 2 * k2v, acc(x + h / 2 * k2x, v + h / 2 * k2v, u)
        k4x, k4v = v + h * k3v, acc(x + h * k3x, v + h * k3v, u)
        x = x + h / 6 * (k1x + 2 * k2x + 2 * k3x + k4x)
        v = v + h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        err.append(x - d["x"][s + k + 1])
    e = np.stack(err, axis=1)
    return np.sqrt(np.mean(e ** 2, axis=0)) / np.std(d["x"][band])


# ---------------------------------------------------------------- simulated runs as rig CSVs

def write_rig_csv(result, path, accel_bias_V=None):
    """Write a SimResult in the firmware's log format, so vca_log.load_log and the notebooks read it.

    Same header (including its leading spaces), column order and duplicated first timestamp as the
    firmware. ai2_corrected_V subtracts the idle-window mean, or accel_bias_V / the configured bias when
    there is no idle window. Extra columns are appended after the firmware's: position_ref_mm (or
    current reference), the controller output, its signals, and the simulator's true position and current.
    """
    c = result.columns
    t = c["t_s"]
    idle = t < 0
    if accel_bias_V is None:
        accel_bias_V = c["accel_V"][idle].mean() if idle.any() else result.settings["accel"]["bias_V"]
    i = c["actual_current_A"]
    p = COIL_R_OHM * c["i_true_A"] ** 2
    energy = np.cumsum(p) * result.settings["dt_s"]
    ai1 = (51.7 - 22.5 - c["position_mm"]) / 6.568
    ref_name = "position_ref_mm" if result.ref_unit == "mm" else "current_ref_A"
    extra = {ref_name: c["ref"], "controller_output_A": c["u_cmd_A"]}
    extra.update({f"ctrl_{s}": c[f"ctrl_{s}"] for s in result.signal_names})
    extra.update({"sim_true_position_mm": c["x_true_mm"], "sim_true_current_A": c["i_true_A"]})
    data = np.column_stack([t, i, c["target_current_A"], c["i_true_A"], ai1, c["accel_V"],
                            np.full(len(t), 535.0), p, energy, np.zeros(len(t)), np.zeros(len(t)),
                            c["position_mm"], c["accel_V"] - accel_bias_V] + list(extra.values()))
    fmt = ["%.6f"] * 9 + ["%.1f", "%.1f", "%.4f", "%.6f"] + ["%.6f"] * len(extra)
    data = np.vstack([data[:1], data])            # the firmware stamps its first two rows with the same time
    header = LOG_HEADER + "".join("," + k for k in extra)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, data, delimiter=",", fmt=fmt, header=header, comments="")
    return Path(path)
