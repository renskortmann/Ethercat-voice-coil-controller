#!/usr/bin/env python3
"""Position-control reference and tracking plots (EXPERIMENT_POSITION_PID).

Two uses:

* Preview, before a run (no hardware, no log):
      python scripts/plot_voice_coil_log_ref_track.py --preview
  Reads the reference settings straight from the firmware settings headers
  (run_settings.h, closed_loop_settings.h) (POS_REF_SHAPE, POS_REF_STEPS,
  POS_REF_RAMP_S, POS_REF_SINE_*, POS_REF_CHIRP_*, RUN_DURATION_S, BIAS_IDLE_S) and plots the
  reference the controller will track, with the allowed reference window and the
  position trip limits. Also prints the breakpoint table.

* Tracking, after a run:
      python scripts/plot_voice_coil_log_ref_track.py [path/to/log.csv] [--kf] [--chirp]
  With no path the newest log in gcsc_data/ is used. The controller (PI or SMC), reference,
  gains and filters are read from the log's file name (EXPERIMENT_TAG) and shown in the title;
  the traces are labelled for that controller. Four stacked axes:
    position (mm)  -- raw position and the filtered position, over the reference
    current (A)    -- controller output and its two logged terms (PI: P and I; SMC: equivalent
                      (model) control and switching term), and the measured coil current
    power (W)      -- bus-referred power, with the energy delivered since t = 0 on a twin axis
                      (as in plot_voice_coil_log.py; average / RMS power and energy are printed)
    error (mm)     -- reference - filtered position, the tracking error
  The filtered position is position_filt_mm: for the PI the position it used (EKF x(k|k) and/or
  notched); for the SMC the EKF x(k|k) (it acts on the one-step prediction, which is not logged).
  Older logs show a 60 Hz low-pass of the raw position instead.
  The reference comes from the log's position_ref_mm column, i.e. exactly what the
  controller used. If the log has no such column it falls back to the programmed
  reference and says so. If the settings have changed since the run, a note is printed.
  Extra figures, off by default:
  --kf: if the run used the EKF (POS_KF_ENABLE, kf_* columns finite) a second figure shows how well
  the filter worked, from t >= 0 (as in the Kalman cell of vca_greybox_fit.ipynb):
    innovation y(k) - x(k|k-1) against time, with +/- 2 predicted std (sqrt S)
    histogram of the innovation with the normal density the filter predicts
    PSD of the innovation with the white-noise level at the predicted std
  and prints the innovation RMS, the share inside +/- 2 std (95 % if consistent) and
  innovation std / predicted std (1 if SIG_A and SIG_Y fit the data; >> 1: model error).
  --chirp: for chirp runs (pos<controller>_chirp or _chirphold in the file name) it prints and plots the tracking
  per frequency: amplitude of position / reference and phase lag, fitted in windows
  of a few periods around 1, 1.5, 2, 3 ... 10 Hz. That analysis uses only the logged
  reference and position, not the settings headers.

The reference semantics mirror pos_ref() in closed_loop_position.c: at each breakpoint
the reference ramps linearly from the previous value over POS_REF_RAMP_S and then
holds; the last value holds to the end. The controller is off (reference nan)
during the 0 A idle window at t < 0.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys

import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_voice_coil_log import REPO_ROOT, latest_log, low_pass, rms_power, time_weighted_mean  # noqa: E402

# The #define names the firmware uses, read from its settings headers
SETTINGS_H = [os.path.join(REPO_ROOT, n) for n in ("run_settings.h", "closed_loop_settings.h")]
POSITION_LP_CUTOFF_HZ = 60.0  # same low-pass as the position trace in plot_voice_coil_log.py
PREVIEW_DT_S = 0.005


def read_settings(paths: list = SETTINGS_H) -> dict:
    """Reference settings from the firmware settings headers, using the same #define names."""
    text = "\n".join(open(p).read() for p in paths)
    path = " / ".join(os.path.basename(p) for p in paths)

    def define(name: str) -> str:
        m = re.search(rf"^#define\s+{name}\s+(.+?)\s*(?:/\*.*)?(?://.*)?$", text, re.MULTILINE)
        if not m:
            sys.exit(f"{name} not found in {path}")
        return m.group(1).strip()

    def number(name: str) -> float:
        return float(define(name))

    controllers = {"POS_CONTROLLER_PI": "PI", "POS_CONTROLLER_SMC": "SMC"}

    shape = define("POS_REF_SHAPE")
    shapes = {"POS_REF_SHAPE_STEPS": "steps", "POS_REF_SHAPE_SINE": "sine", "POS_REF_SHAPE_CHIRP": "chirp",
              "POS_REF_SHAPE_CHIRP_HOLD": "chirp_hold"}
    if shape not in shapes:
        sys.exit(f"unknown POS_REF_SHAPE {shape!r} in {path}")
    steps = [(float(t), float(p)) for t, p in
             re.findall(r"X\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)", define(r"POS_REF_STEPS\(X\)"))]
    return {
        "mode": define("EXPERIMENT_MODE"),
        "shape": shapes[shape],
        "steps": steps,
        "ramp_s": number("POS_REF_RAMP_S"),
        "sine_offset_mm": number("POS_REF_SINE_OFFSET_MM"),
        "sine_amplitude_mm": number("POS_REF_SINE_AMPLITUDE_MM"),
        "sine_freq_hz": number("POS_REF_SINE_FREQ_HZ"),
        "chirp_offset_mm": number("POS_REF_CHIRP_OFFSET_MM"),
        "chirp_amplitude_mm": number("POS_REF_CHIRP_AMPLITUDE_MM"),
        "chirp_f0_hz": number("POS_REF_CHIRP_F0_HZ"),
        "chirp_f1_hz": number("POS_REF_CHIRP_F1_HZ"),
        "chirp_duration_s": number("POS_REF_CHIRP_DURATION_S"),
        "chirp_start_s": number("POS_REF_CHIRP_START_S"),
        "chirp_taper_s": number("POS_REF_CHIRP_TAPER_S"),
        "chirp_f1_hold_s": number("POS_REF_CHIRP_F1_HOLD_S"),
        "run_s": number("RUN_DURATION_S"),
        "idle_s": number("BIAS_IDLE_S"),
        "ref_min_mm": number("POS_REF_MIN_MM"),
        "ref_max_mm": number("POS_REF_MAX_MM"),
        "trip_min_mm": number("POS_TRIP_MIN_MM"),
        "trip_max_mm": number("POS_TRIP_MAX_MM"),
        "controller": controllers.get(define("POS_CONTROLLER"), define("POS_CONTROLLER")),
        "output_limit_a": number("OUTPUT_LIMIT_A"),
        "kp": number("PID_KP_A_PER_MM"),
        "ki": number("PID_KI_A_PER_MM_S"),
        "smc_a_factor": number("SMC_A_FREQ_FACTOR"),
        "smc_steps_hz": number("SMC_STEPS_FREQ_HZ"),
        "smc_layer_mm": number("SMC_LAYER_MM"),
        "smc_eta_sigmas": number("SMC_ETA_SIGMAS"),
        "kf_enable": number("POS_KF_ENABLE"),
        "kf_sig_a": number("POS_KF_SIG_A_M_S2"),
    }


def chirp_phase(cfg: dict, tau: float) -> float:
    """Chirp phase, mirroring pos_ref_chirp_phase(): the exponential sweep of exp_chirp_phase() in
    waveforms.h, and for chirp_hold a constant f1 after T."""
    f0, f1, T = cfg["chirp_f0_hz"], cfg["chirp_f1_hz"], cfg["chirp_duration_s"]
    L = T / math.log(f1 / f0)
    if cfg["shape"] == "chirp_hold" and tau >= T:
        return 2 * math.pi * f0 * L * (math.exp(T / L) - 1.0) + 2 * math.pi * f1 * (tau - T)
    return 2 * math.pi * f0 * L * (math.exp(tau / L) - 1.0)


def chirp_after_s(cfg: dict) -> float:
    """Time after the sweep before the end: the hold at f1 and the fade-out (chirp_hold only)."""
    return cfg["chirp_f1_hold_s"] + cfg["chirp_taper_s"] if cfg["shape"] == "chirp_hold" else 0.0


def chirp_end_s(cfg: dict) -> float:
    """Sweep time of the first zero crossing at or after T (+ hold and fade-out for chirp_hold), mirroring
    pos_ref_chirp_end_s()."""
    f0, f1, T = cfg["chirp_f0_hz"], cfg["chirp_f1_hz"], cfg["chirp_duration_s"]
    phase_end = math.pi * math.ceil(chirp_phase(cfg, T + chirp_after_s(cfg)) / math.pi)
    if cfg["shape"] == "chirp_hold":
        return T + (phase_end - chirp_phase(cfg, T)) / (2 * math.pi * f1)
    L = T / math.log(f1 / f0)
    return L * math.log(1.0 + phase_end / (2 * math.pi * f0 * L))


def chirp_taper(cfg: dict, tau: float) -> float:
    """Raised-cosine amplitude envelope, mirroring pos_ref_chirp_taper(): 0 -> 1 over the first
    POS_REF_CHIRP_TAPER_S of the sweep, 1 -> 0 over the last."""
    F = cfg["chirp_taper_s"]
    u = min(tau, chirp_end_s(cfg) - tau)                 # time from the nearer end
    return 0.5 * (1.0 - math.cos(math.pi * u / F)) if 0.0 < F and u < F else 1.0


def reference_mm(cfg: dict, t: float) -> float:
    """Reference at time t, mirroring pos_ref() in closed_loop_position.c (nan in the idle window)."""
    if t < 0.0:
        return float("nan")
    if cfg["shape"] in ("chirp", "chirp_hold"):
        tau = t - cfg["chirp_start_s"]
        if tau <= 0.0 or tau >= chirp_end_s(cfg):
            return cfg["chirp_offset_mm"]
        return cfg["chirp_offset_mm"] + cfg["chirp_amplitude_mm"] * chirp_taper(cfg, tau) * math.sin(chirp_phase(cfg, tau))
    if cfg["shape"] == "sine":
        return cfg["sine_offset_mm"] + cfg["sine_amplitude_mm"] * math.sin(2 * math.pi * cfg["sine_freq_hz"] * t)
    steps = cfg["steps"]
    i = 0
    while i + 1 < len(steps) and steps[i + 1][0] <= t:
        i += 1
    into_ramp_s = t - steps[i][0]
    if i > 0 and cfg["ramp_s"] > 0.0 and into_ramp_s < cfg["ramp_s"]:
        prev = steps[i - 1][1]
        return prev + (steps[i][1] - prev) * into_ramp_s / cfg["ramp_s"]
    return steps[i][1]


def describe(cfg: dict) -> str:
    lines = []
    if cfg["mode"] != "EXPERIMENT_POSITION_PID":
        lines.append(f"NOTE: EXPERIMENT_MODE is {cfg['mode']}, so this reference is NOT used until it is "
                     "set to EXPERIMENT_POSITION_PID")
    if cfg["controller"] == "SMC":
        f_chirp = max(cfg["chirp_f0_hz"], cfg["chirp_f1_hz"])
        f_max = {"chirp": f_chirp, "chirp_hold": f_chirp, "sine": cfg["sine_freq_hz"]}.get(
            cfg["shape"], cfg["smc_steps_hz"])
        a = cfg["smc_a_factor"] * 2 * math.pi * f_max
        eta = cfg["smc_eta_sigmas"] * cfg["kf_sig_a"]
        lam_ts = eta / (a * cfg["smc_layer_mm"] * 1e-3) * 0.5e-3
        lines.append(f"controller: SMC, a = {a:.1f} 1/s ({cfg['smc_a_factor']} x 2 pi {f_max} Hz), "
                     f"layer {cfg['smc_layer_mm']} mm, eta = {eta:.2f} m/s^2, (eta / phi) Ts = {lam_ts:.2f}"
                     + ("" if lam_ts < 1 else " (>= 1: overshoot)" if lam_ts < 2 else " (>= 2: REFUSED at startup)"))
        if not cfg["kf_enable"]:
            lines.append("NOTE: the SMC needs POS_KF_ENABLE 1; the build will fail")
    else:
        lines.append(f"controller: {cfg['controller']}, Kp = {cfg['kp']} A/mm, Ki = {cfg['ki']} A/(mm s)")
    lines.append(f"output limit +/-{cfg['output_limit_a']} A")
    lines.append(f"t = {-cfg['idle_s']:.1f} .. 0 s: idle window, 0 A, controller off")
    if cfg["shape"] == "steps":
        ramped = cfg["ramp_s"] > 0.0
        lines.append("reference: absolute steps, " +
                     (f"linear ramp of {cfg['ramp_s']} s at each change" if ramped else "hard steps (no ramp)"))
        for i, (t, p) in enumerate(cfg["steps"]):
            if i == 0:
                lines.append(f"  t = {t:6.2f} s            : {p:+.3f} mm (from the moment the controller switches on)")
            elif not ramped:
                lines.append(f"  t = {t:6.2f} s            : step {cfg['steps'][i - 1][1]:+.3f} -> {p:+.3f} mm, then hold")
            else:
                end = t + cfg["ramp_s"]
                lines.append(f"  t = {t:6.2f} .. {end:6.2f} s : ramp {cfg['steps'][i - 1][1]:+.3f} -> {p:+.3f} mm, then hold")
        lines.append(f"  last value held until t = {cfg['run_s']:.2f} s, then voltage off")
    elif cfg["shape"] == "chirp_hold":
        start, end = cfg["chirp_start_s"], cfg["chirp_start_s"] + chirp_end_s(cfg)
        sweep_end = start + cfg["chirp_duration_s"]
        hold_end = sweep_end + cfg["chirp_f1_hold_s"]
        lines.append(f"reference: absolute chirp and hold {cfg['chirp_offset_mm']:+.3f} mm + {cfg['chirp_amplitude_mm']:.3f} mm "
                     f"* sin(phase), exponential sweep {cfg['chirp_f0_hz']} -> {cfg['chirp_f1_hz']} Hz, then {cfg['chirp_f1_hz']} Hz")
        lines.append(f"  t = 0 .. {start:.2f} s        : hold {cfg['chirp_offset_mm']:+.3f} mm")
        lines.append(f"  t = {start:.2f} .. {sweep_end:.2f} s : sweep, f(t) = {cfg['chirp_f0_hz']} * "
                     f"({cfg['chirp_f1_hz']}/{cfg['chirp_f0_hz']})^((t - {start:g}) / {cfg['chirp_duration_s']:g}) Hz"
                     + (f", fading in over the first {cfg['chirp_taper_s']} s" if cfg["chirp_taper_s"] > 0.0 else ""))
        lines.append(f"  t = {sweep_end:.2f} .. {hold_end:.2f} s : {cfg['chirp_f1_hz']} Hz at full amplitude")
        lines.append(f"  t = {hold_end:.2f} .. {end:.3f} s : {cfg['chirp_f1_hz']} Hz, fading out (raised cosine) "
                     "to the next zero crossing")
        lines.append(f"  t = {end:.3f} .. {cfg['run_s']:.2f} s : hold {cfg['chirp_offset_mm']:+.3f} mm, then voltage off")
    elif cfg["shape"] == "chirp":
        start, end = cfg["chirp_start_s"], cfg["chirp_start_s"] + chirp_end_s(cfg)
        lines.append(f"reference: absolute chirp {cfg['chirp_offset_mm']:+.3f} mm + {cfg['chirp_amplitude_mm']:.3f} mm "
                     f"* sin(phase), exponential sweep {cfg['chirp_f0_hz']} -> {cfg['chirp_f1_hz']} Hz")
        lines.append(f"  t = 0 .. {start:.2f} s        : hold {cfg['chirp_offset_mm']:+.3f} mm")
        lines.append(f"  t = {start:.2f} .. {end:.3f} s : sweep, f(t) = {cfg['chirp_f0_hz']} * "
                     f"({cfg['chirp_f1_hz']}/{cfg['chirp_f0_hz']})^((t - {start:g}) / {cfg['chirp_duration_s']:g}) Hz "
                     f"(ends at the zero crossing after {cfg['chirp_start_s'] + cfg['chirp_duration_s']:.2f} s)")
        if cfg["chirp_taper_s"] > 0.0:
            lines.append(f"  amplitude fades in over {start:.2f} .. {start + cfg['chirp_taper_s']:.2f} s and out over "
                         f"{end - cfg['chirp_taper_s']:.3f} .. {end:.3f} s (raised cosine)")
        lines.append(f"  t = {end:.3f} .. {cfg['run_s']:.2f} s : hold {cfg['chirp_offset_mm']:+.3f} mm, then voltage off")
    else:
        lines.append(f"reference: absolute sine {cfg['sine_offset_mm']:+.3f} mm + {cfg['sine_amplitude_mm']:.3f} mm "
                     f"* sin(2 pi {cfg['sine_freq_hz']} Hz t), t = 0 .. {cfg['run_s']:.2f} s")
    lines.append(f"allowed reference window {cfg['ref_min_mm']} .. {cfg['ref_max_mm']} mm, "
                 f"trip outside {cfg['trip_min_mm']} .. {cfg['trip_max_mm']} mm")
    return "\n".join(lines)


def draw_limits(ax, cfg: dict) -> None:
    for y in (cfg["ref_min_mm"], cfg["ref_max_mm"]):
        ax.axhline(y, color="tab:orange", ls=":", lw=1.0)
    for y in (cfg["trip_min_mm"], cfg["trip_max_mm"]):
        ax.axhline(y, color="tab:red", ls="--", lw=1.0)
    ax.plot([], [], color="tab:orange", ls=":", label="reference window")
    ax.plot([], [], color="tab:red", ls="--", label="position trip")


def shade_idle(ax, t0: float) -> None:
    if t0 < 0.0:
        ax.axvspan(t0, 0.0, color="0.85", alpha=0.6, lw=0)


def preview(cfg: dict) -> None:
    print(describe(cfg))
    n = int((cfg["idle_s"] + cfg["run_s"]) / PREVIEW_DT_S) + 1
    t = [-cfg["idle_s"] + k * PREVIEW_DT_S for k in range(n)]
    r = [reference_mm(cfg, x) for x in t]

    fig, ax = plt.subplots(figsize=(11, 5))
    shade_idle(ax, t[0])
    ax.plot(t, r, color="tab:red", lw=1.8, label="position reference")
    if cfg["shape"] == "steps":
        for i, (bt, bp) in enumerate(cfg["steps"]):
            reached = bt + (cfg["ramp_s"] if i > 0 else 0.0)  # where the ramp ends and the hold starts
            ax.plot(reached, bp, "o", color="tab:red", ms=4)
            ax.annotate(f"{bp:+.2f} mm @ {reached:g} s", (reached, bp), textcoords="offset points",
                        xytext=(4, 6), fontsize="small")
    draw_limits(ax, cfg)
    ax.text(t[0] / 2, 0.02, "idle (0 A)", transform=ax.get_xaxis_transform(), ha="center", fontsize="small", color="0.4")
    ax.set_xlabel("time_s (s)")
    ax.set_ylabel("position_mm (mm)")
    ax.set_title("Programmed position reference (from the settings headers, not from a run)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize="small")
    fig.tight_layout()
    plt.show()


# Trace labels per controller: what the logged pid_p_A / pid_i_A / pid_output_A mean
CONTROLLER_LABELS = {
    "PI": {"out": "PI output u (sent next cycle)", "p": "P term  Kp e", "i": "I term  integrator"},
    "SMC": {"out": "SMC output u (sent next cycle)", "p": "equivalent control  (r'' - a z2 - f) / g",
            "i": "switching term  -eta sat(s/phi) / g"},
}


def filtered_label(ctrl: str, kf: bool, notches: list[str], delay: int = 0) -> str:
    """What position_filt_mm is for this run (see closed_loop_step())."""
    if ctrl == "SMC":
        return f"EKF x(k|k)  (SMC acts on the prediction x(k+{1 + delay}|k))"
    used = (["EKF x(k|k)"] if kf else []) + ([f"notched {' + '.join(notches)} Hz"] if notches else [])
    return f"position used by {ctrl}: " + (", ".join(used) if used else "raw")


def describe_run(path: str) -> tuple[str, str, str]:
    """Controller name, a one-line description of the run and the label of position_filt_mm, from the
    EXPERIMENT_TAG in the file name,
    e.g. posSMC_chirp_0.0mm_3.0mm_1.0to5.0Hz_60.0s_hold2.0s_a3.0x_phi0.1mm_eta2.0sig_kfd4."""
    name = os.path.basename(path)
    m = re.match(r"voice_coil_log_pos([A-Za-z]+)_(.*?)(?:_\d{8}_\d{6})?\.csv$", name)
    if not m:
        return "PI", "(no position-control tag in the file name)", "filtered position"
    ctrl, tag = m.groups()
    parts = []
    ref = re.match(r"(steps|sine|chirphold|chirp)_", tag)
    if ref and ref.group(1) in ("chirp", "chirphold"):
        c = re.search(r"chirp(?:hold)?_([-\d.]+)mm_([\d.]+)mm_([\d.]+)to([\d.]+)Hz_([\d.]+)s", tag)
        if c:
            taper = re.search(r"_taper([\d.]+)s", tag)
            f1hold = re.search(r"_f1hold([\d.]+)s", tag)
            parts.append(f"chirp {c[3]} -> {c[4]} Hz, {c[2]} mm around {c[1]} mm, {c[5]} s"
                         + (f", then {c[4]} Hz for {f1hold[1]} s" if f1hold else "")
                         + (f", fade {taper[1]} s" if taper and float(taper[1]) > 0 else ""))
    elif ref and ref.group(1) == "sine":
        c = re.search(r"sine_([-\d.]+)mm_([\d.]+)mm_([\d.]+)Hz", tag)
        if c:
            parts.append(f"sine {c[3]} Hz, {c[2]} mm around {c[1]} mm")
    elif ref:
        parts.append("steps")
    if ctrl == "SMC":
        g = re.search(r"_a([\d.]+)x_phi([\d.]+)mm_eta([\d.]+)sig", tag)
        if g:
            parts.append(f"a = {g[1]} x 2 pi f_max, layer {g[2]} mm, eta = {g[3]} SIG_A")
    else:
        g = re.search(r"_kp([\d.]+)_ki([\d.]+)", tag)
        if g:
            parts.append(f"Kp {g[1]} A/mm, Ki {g[2]} A/(mm s)")
    kf_m, notches = re.search(r"_kf(?:d(\d+))?(m?)(?=_|$)", tag), re.findall(r"_notch([\d.]+)Hz", tag)
    out_notches = re.findall(r"_onotch([\d.]+)Hz_Q([\d.]+)", tag)
    kf, delay = kf_m is not None, int(kf_m[1]) if kf_m and kf_m[1] else 0
    ekf = ("EKF" + (f", delay {delay} cycles" if delay else "") + (", 50 Hz pickup state" if kf_m and kf_m[2] else ""))
    filters = ([ekf] if kf else []) + [f"notch {f} Hz" for f in notches] + \
        [f"output notch {f} Hz Q{q}" for f, q in out_notches]
    parts.append(" + ".join(filters) if filters else "no position filter")
    return ctrl, f"{ctrl}: " + " | ".join(parts), filtered_label(ctrl, kf, notches, delay)


def read_columns(path: str, names: list[str]) -> tuple[list[float], dict[str, list[float]]]:
    """time_s plus the named columns; a column missing from the log is all nan."""
    time_s: list[float] = []
    cols: dict[str, list[float]] = {n: [] for n in names}
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        reader.fieldnames = [f.strip() for f in (reader.fieldnames or [])]
        for row in reader:
            try:
                time_s.append(float(row["time_s"]))
            except (KeyError, TypeError, ValueError):
                continue
            for n in names:
                try:
                    cols[n].append(float(row[n]))
                except (KeyError, TypeError, ValueError):
                    cols[n].append(float("nan"))
    return time_s, cols


CHIRP_ANALYSIS_HZ = (1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)
CHIRP_WINDOW_PERIODS = 2.0  # fit window: +/- this many periods around the target frequency


def chirp_tracking(t: list[float], r: list[float], y: list[float]) -> list[tuple[float, float, float, float]]:
    """Amplitude ratio and phase lag of y relative to a swept-sine reference r, per frequency.

    Uses only the logged signals: the local frequency comes from the spacing of the reference's
    zero crossings around its centre value, and in a window of +/- CHIRP_WINDOW_PERIODS periods
    y is least-squares fitted as  a*r + b*(dr/dt)/w + c  (w = 2 pi f), so that
    gain = |a + j*b| (in the sense y = gain * A sin(phase - lag)) and lag = -angle(a + j*b).
    Returns (target Hz, local Hz, gain, lag in degrees) for each frequency the sweep reached.
    """
    import numpy as np

    t = np.asarray(t)
    r = np.asarray(r)
    y = np.asarray(y)
    ok = np.isfinite(r) & np.isfinite(y)
    t, r, y = t[ok], r[ok], y[ok]
    centre = np.median(r[t >= 0][:200])                 # the hold value before the sweep
    d = r - centre
    idx = np.nonzero((d[:-1] < 0) != (d[1:] < 0))[0]
    zc = t[idx] - d[idx] * (t[idx + 1] - t[idx]) / (d[idx + 1] - d[idx])
    if len(zc) < 6:
        return []
    f_local = 0.5 / np.diff(zc)
    t_mid = 0.5 * (zc[1:] + zc[:-1])
    drdt = np.gradient(r, t)
    out = []
    for f_target in CHIRP_ANALYSIS_HZ:
        k = int(np.argmin(np.abs(f_local - f_target)))
        if abs(f_local[k] - f_target) > 0.05 * f_target:
            continue                                     # the sweep never reached this frequency
        # Keep the window inside the sweep: at the ends, shift it inwards (and report that frequency).
        half = CHIRP_WINDOW_PERIODS / f_local[k]
        t_c = min(max(t_mid[k], zc[0] + half), zc[-1] - half)
        k = int(np.argmin(np.abs(t_mid - t_c)))
        f = f_local[k]
        w = 2 * np.pi * f
        m = np.abs(t - t_c) <= half
        X = np.column_stack([r[m] - centre, drdt[m] / w, np.ones(m.sum())])
        (a, b, _), *_ = np.linalg.lstsq(X, y[m], rcond=None)
        # r - centre = A sin(phi), dr/dt / w = A cos(phi); y ~ a A sin(phi) + b A cos(phi) = g A sin(phi - lag)
        out.append((f_target, f, math.hypot(a, b), -math.degrees(math.atan2(b, a))))
    return out


KF_PSD_NPERSEG = 4096  # samples per Welch segment: 2.05 s at 2 kHz, 0.49 Hz resolution


def welch_psd(x, fs: float, nperseg: int):
    """One-sided Welch PSD (Hann window, 50 % overlap, mean removed per segment), numpy only."""
    import numpy as np

    x = np.asarray(x, dtype=float)
    nperseg = min(nperseg, len(x))
    win = np.hanning(nperseg)
    scale = 1.0 / (fs * np.sum(win ** 2))
    starts = range(0, len(x) - nperseg + 1, nperseg // 2)
    P = np.mean([np.abs(np.fft.rfft((x[s:s + nperseg] - x[s:s + nperseg].mean()) * win)) ** 2 for s in starts], axis=0)
    P *= scale
    P[1:-1 if nperseg % 2 == 0 else None] *= 2.0
    return np.fft.rfftfreq(nperseg, 1.0 / fs), P


def kf_diagnostics(path: str, t: list[float], c: dict) -> None:
    """Second figure for EKF runs: innovation against time with +/- 2 predicted std, its histogram
    against the predicted normal density, and its PSD against the white level at the predicted std."""
    import numpy as np

    t = np.asarray(t)
    e = np.asarray(c["kf_innovation_mm"])
    sd = np.asarray(c["kf_innov_std_mm"])
    m = (t >= 0.0) & np.isfinite(e) & np.isfinite(sd)
    if m.sum() < 10:
        return
    te, e, sd = t[m], e[m], sd[m]
    s_mean = float(np.mean(sd))
    inside = float(np.mean(np.abs(e) <= 2.0 * sd))
    ratio = float(np.std(e)) / s_mean
    print(f"\nEKF innovation y(k) - x(k|k-1), t >= 0: RMS {np.sqrt(np.mean(e ** 2)):.4f} mm, "
          f"mean {np.mean(e):+.4f} mm, inside +/-2 predicted std {inside * 100:.1f} % (95 % if consistent)")
    print(f"  innovation std / predicted std = {ratio:.2f} (1 = consistent; >> 1: model error or SIG_A/SIG_Y too small; "
          f"<< 1: too large)")

    fs = 1.0 / float(np.median(np.diff(te)))
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle(f"EKF check: {os.path.basename(path)}", fontsize="medium")
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1])
    ax_t = fig.add_subplot(gs[0, :])
    ax_h = fig.add_subplot(gs[1, 0])
    ax_f = fig.add_subplot(gs[1, 1])

    ax_t.plot(te, e, color="tab:blue", lw=0.3, label="innovation y(k) - x(k|k-1)")
    pk_all = np.asarray(c.get("kf_pickup_mm", [np.nan] * len(t)))
    if np.isfinite(pk_all).any():           # runs with POS_KF_MAINS_ENABLE: the pickup estimate
        for name, mm in (("idle window", (t < 0.0) & np.isfinite(pk_all)), ("t >= 0", m & np.isfinite(pk_all))):
            if mm.sum() > 100:
                amp = 2 * abs(np.mean(pk_all[mm] * np.exp(-2j * np.pi * 50.0 * t[mm])))
                print(f"  mains pickup estimate kf_pickup_mm, {name}: 50 Hz amplitude {amp:.4f} mm")
        ax_t.plot(te, pk_all[m], color="tab:orange", lw=0.3, alpha=0.7, label="pickup estimate kf_pickup_mm")
    ax_t.plot(te, 2 * sd, color="black", lw=1.0, label="+/- 2 predicted std")   # on top of the dense trace
    ax_t.plot(te, -2 * sd, color="black", lw=1.0)
    ax_t.set(xlabel="time_s (s)", ylabel="innovation (mm)",
             title=f"innovation, {inside * 100:.1f} % inside +/- 2 std, std ratio {ratio:.2f}")
    ax_t.legend(loc="upper right", fontsize="small")

    lim = float(np.percentile(np.abs(e), 99.5))
    ax_h.hist(e, bins=np.linspace(-lim, lim, 101), density=True, color="tab:blue", alpha=0.7)
    g = np.linspace(-lim, lim, 400)
    ax_h.plot(g, np.exp(-0.5 * (g / s_mean) ** 2) / (s_mean * np.sqrt(2 * np.pi)), "k--",
              label=f"predicted N(0, {s_mean:.4f} mm)")
    ax_h.set(xlabel="innovation (mm)", ylabel="density",
             title=f"histogram: mean {np.mean(e):+.4f} mm, std {np.std(e):.4f} mm")
    ax_h.legend(fontsize="small")

    f, P = welch_psd(e, fs, KF_PSD_NPERSEG)
    ax_f.plot(f, 10 * np.log10(P + 1e-20), color="tab:blue", lw=0.8, label="innovation PSD")
    ax_f.axhline(10 * np.log10(2 * s_mean ** 2 / fs), color="k", ls="--", label="white at predicted std")
    ax_f.set(xlim=(0, fs / 2), xlabel="frequency (Hz)", ylabel="PSD (dB re 1 mm²/Hz)",
             title="PSD: flat if optimal; peaks = missed dynamics or mains pickup")
    ax_f.legend(fontsize="small")
    for ax in (ax_t, ax_h, ax_f):
        ax.grid(True, alpha=0.3)
    fig.tight_layout()


def tracking(cfg: dict, path: str, show_kf: bool = False, show_chirp: bool = False) -> None:
    names = ["position_mm", "position_ref_mm", "pid_p_A", "pid_i_A", "pid_output_A", "actual_current_A",
             "position_filt_mm", "kf_innovation_mm", "kf_innov_std_mm", "power_W", "energy_J", "kf_pickup_mm"]
    t, c = read_columns(path, names)
    if not t:
        sys.exit(f"no usable rows in {path}")
    ctrl, run_desc, filt_label = describe_run(path)
    lab = CONTROLLER_LABELS.get(ctrl, {"out": f"{ctrl} output u (sent next cycle)", "p": "pid_p_A", "i": "pid_i_A"})
    print(run_desc)
    y = c["position_mm"]
    r = c["position_ref_mm"]
    from_main_h = [reference_mm(cfg, x) for x in t]
    if not any(math.isfinite(v) for v in r):
        print("note: this log has no position_ref_mm column (not a position-control run); "
              "showing the CURRENT settings-header reference instead")
        r = from_main_h
    else:
        diff = max((abs(a - b) for a, b in zip(r, from_main_h) if math.isfinite(a) and math.isfinite(b)), default=0.0)
        if diff > 1e-3:
            print(f"note: the settings headers' reference differs from this run's by up to {diff:.3f} mm "
                  "(settings changed since the run); plotting the run's own reference")

    # Logs with position_filt_mm: the filtered position (what the PI acted on; the SMC's EKF x(k|k)).
    # Older logs: fall back to a 60 Hz low-pass of the raw position, for display only.
    if any(math.isfinite(v) for v in c["position_filt_mm"]):
        y_ctrl = c["position_filt_mm"]
        ctrl_label = filt_label
        err_label = "r - filtered position"
    else:
        y_ctrl = low_pass(t, y, POSITION_LP_CUTOFF_HZ)
        ctrl_label = f"position ({POSITION_LP_CUTOFF_HZ:.0f} Hz low-pass)"
        err_label = f"{POSITION_LP_CUTOFF_HZ:.0f} Hz low-passed"
    err = [a - b if math.isfinite(a) else float("nan") for a, b in zip(r, y_ctrl)]
    run = [i for i, x in enumerate(t) if x >= 0.0 and math.isfinite(err[i])]
    if run:
        rms = math.sqrt(sum(err[i] ** 2 for i in run) / len(run))
        print(f"tracking error ({err_label}), t >= 0: rms {rms:.3f} mm, "
              f"max |e| {max(abs(err[i]) for i in run):.3f} mm")
    u = [v for v in c["pid_output_A"] if math.isfinite(v)]
    if u:
        n_lim = sum(abs(v) >= cfg["output_limit_a"] - 1e-9 for v in u)
        print(f"{ctrl} output: max |u| {max(abs(v) for v in u):.3f} A, RMS {math.sqrt(sum(v * v for v in u) / len(u)):.3f} A, "
              f"RMS change per cycle {math.sqrt(sum((a - b) ** 2 for a, b in zip(u[1:], u)) / max(len(u) - 1, 1)):.3f} A, "
              f"at the +/-{cfg['output_limit_a']} A limit {100 * n_lim / len(u):.1f} % of cycles")

    # Power and energy over the experiment only (t >= 0), as in plot_voice_coil_log.py
    has_power = any(math.isfinite(v) for v in c["power_W"])
    energy = c["energy_J"]
    power_desc = ""
    if has_power:
        k0 = next((i for i, x in enumerate(t) if x >= 0.0), len(t))
        idle_energy_J = energy[k0 - 1] if k0 > 0 and math.isfinite(energy[k0 - 1]) else 0.0
        energy = [v - idle_energy_J for v in energy]
        exp = [i for i in range(k0, len(t)) if math.isfinite(c["power_W"][i])]
        if len(exp) > 1:
            t_exp, p_exp = [t[i] for i in exp], [c["power_W"][i] for i in exp]
            e_end = next((energy[i] for i in reversed(exp) if math.isfinite(energy[i])), float("nan"))
            power_desc = (f"average power {time_weighted_mean(t_exp, p_exp):.1f} W | "
                          f"RMS power {rms_power(t_exp, p_exp):.1f} W | energy delivered {e_end:.1f} J")
            print(f"power, t >= 0: {power_desc}")
    else:
        print("note: no power_W in this log; power panel left empty")

    fig, (ax_pos, ax_cur, ax_pwr, ax_err) = plt.subplots(4, 1, sharex=True, figsize=(11, 11),
                                                        gridspec_kw={"height_ratios": [3, 2, 1.5, 1.5]})
    fig.suptitle(f"{run_desc}\n{os.path.basename(path)}" + (f"\n{power_desc}" if power_desc else ""),
                 fontsize="medium")
    for ax in (ax_pos, ax_err, ax_cur, ax_pwr):
        shade_idle(ax, t[0])
        ax.grid(True, alpha=0.3)

    ax_pos.plot(t, y, color="tab:brown", lw=0.6, alpha=0.5, label="position (raw)")
    ax_pos.plot(t, y_ctrl, color="black", lw=1.0, label=ctrl_label)
    ax_pos.plot(t, r, color="tab:red", lw=1.6, label="reference")
    draw_limits(ax_pos, cfg)
    ax_pos.set_ylabel("position_mm (mm)")
    ax_pos.legend(loc="upper right", fontsize="small", ncol=2)

    ax_err.plot(t, err, color="tab:purple", lw=1.0)
    ax_err.axhline(0.0, color="0.5", lw=0.8)
    ax_err.set_ylabel("error (mm)")
    ax_err.set_title(f"tracking error: reference - {ctrl_label}", fontsize="small", loc="left")
    ax_err.set_xlabel("time_s (s)")

    # Drawn back to front, so a busy term (e.g. the SMC switching term) does not hide the output
    ax_cur.plot(t, c["pid_i_A"], color="tab:orange", lw=0.6, alpha=0.5, label=lab["i"])
    ax_cur.plot(t, c["actual_current_A"], color="tab:blue", lw=0.8, alpha=0.6, label="actual current")
    ax_cur.plot(t, c["pid_output_A"], color="tab:purple", lw=1.0, label=lab["out"])
    if ctrl != "SMC":  # the SMC's equivalent control swamps the axis; its output and switching term are enough
        ax_cur.plot(t, c["pid_p_A"], color="tab:green", lw=1.0, label=lab["p"])
    ax_cur.set_ylabel("current (A)")
    ax_cur.legend(loc="upper right", fontsize="small", ncol=2)

    ax_pwr.set_ylabel("power (W)")
    if has_power:
        (l_p,) = ax_pwr.plot(t, c["power_W"], color="tab:red", lw=0.8, label="power")
        ax_energy = ax_pwr.twinx()
        (l_e,) = ax_energy.plot(t, energy, color="tab:purple", lw=1.0, linestyle="--", label="energy since t = 0")
        ax_energy.set_ylabel("energy (J)")
        ax_pwr.legend(handles=[l_p, l_e], loc="upper left", fontsize="small")
    else:
        ax_pwr.text(0.5, 0.5, "no power_W in this log", transform=ax_pwr.transAxes,
                    ha="center", va="center", color="0.4")

    fig.tight_layout()

    if show_kf:
        if any(math.isfinite(v) for v in c["kf_innovation_mm"]):
            kf_diagnostics(path, t, c)
        else:
            print("--kf: this run did not use the EKF (no kf_* values)")

    if show_chirp and re.search(r"_chirp(hold)?_", os.path.basename(path)):
        rows = chirp_tracking(t, r, y)
        if rows:
            print("\nchirp tracking (raw position vs reference):")
            print("  target Hz | local Hz | amplitude | lag (deg)")
            for f_t, f, g, lag in rows:
                print(f"  {f_t:9.1f} | {f:8.2f} | {g * 100:7.1f} % | {lag:8.1f}")
            fig2, (ax_g, ax_p) = plt.subplots(2, 1, sharex=True, figsize=(7, 6))
            fig2.suptitle(f"Chirp tracking, raw position / reference\n{run_desc}", fontsize="medium")
            fs = [row[1] for row in rows]
            ax_g.semilogx(fs, [row[2] * 100 for row in rows], "o-", color="tab:blue")
            ax_g.axhline(100, color="0.5", lw=0.8)
            ax_g.set_ylabel("amplitude (%)")
            ax_p.semilogx(fs, [row[3] for row in rows], "o-", color="tab:red")
            ax_p.axhline(0, color="0.5", lw=0.8)
            ax_p.set_ylabel("lag (deg)")
            ax_p.set_xlabel("frequency (Hz)")
            for ax in (ax_g, ax_p):
                ax.grid(True, which="both", alpha=0.3)
            fig2.tight_layout()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", nargs="?", help="log CSV (default: newest in gcsc_data/)")
    parser.add_argument("--preview", action="store_true",
                        help="plot the reference programmed in closed_loop_settings.h, without a log")
    parser.add_argument("--kf", action="store_true", help="also plot the EKF innovation check (EKF runs)")
    parser.add_argument("--chirp", action="store_true", help="also print and plot tracking per frequency (chirp runs)")
    args = parser.parse_args()
    cfg = read_settings()
    if args.preview:
        preview(cfg)
    else:
        tracking(cfg, args.log or latest_log(), show_kf=args.kf, show_chirp=args.chirp)


if __name__ == "__main__":
    main()
