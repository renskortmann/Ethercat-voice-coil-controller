#!/usr/bin/env python3
"""Design a periodic multisine current setpoint for voice-coil system identification.

Produces one period of an odd random-phase multisine (Pintelon & Schoukens style)
with amplitude shaping and crest-factor optimisation, then shows it so it can be
checked BEFORE it is ever sent to the plant. Nothing here talks to the drive.

Design summary:

* Period ``PERIOD_S`` at the firmware cycle time, so every line sits on an exact
  FFT bin (k * fs / N) and periods can be averaged without leakage.
* Only odd harmonics of the base frequency are used. Even bins then contain only
  even-order nonlinear distortion (and noise), which makes them a free diagnostic.
* Roughly ``N_LINES_TARGET`` lines are log-spaced over ``F_MIN_HZ``..``F_MAX_HZ``
  and snapped to the odd-harmonic grid. One line in every ``DETECTION_GROUP``
  consecutive lines is deliberately left unexcited ("detection line"): energy
  showing up there in the measurement is odd-order distortion.
* Line amplitude is ``LOW_LINE_A`` below ``SHAPE_CORNER_HZ`` and rises with f^2
  above it until it reaches ``HIGH_LINE_A``. Low lines are sized for the laser
  (position), high lines for the accelerometer, whose signal grows with f^2.
* Phases start random (``--seed``) and are then crest-factor optimised by
  iterative clipping, so the peak current budget ``PEAK_CURRENT_A`` is spent on
  signal rather than on rare spikes. The whole signal is then scaled up or down
  until either the peak current or the predicted travel hits its limit; the
  binding limit and scale factor are reported. LOW_LINE_A / HIGH_LINE_A set only
  the spectral shape.
* A second-order plant model (constants below, fitted by eye to the 3 A chirp of
  2026-09-23) predicts position and acceleration so the travel limit
  ``TRAVEL_LIMIT_MM`` can be checked. It is a sanity check, not the identification.

Outputs (in ``signals/``; ``data/`` is root-owned because the controller runs under sudo):

* ``<tag>.csv``   one period, columns ``index,target_current_A``; the firmware
                  plays this table verbatim, so what is plotted is what is sent
* ``<tag>.json``  every design parameter, the line table and the statistics
                  (committed: with the seed it reproduces the CSV exactly)
* ``<tag>.png``   the figure shown on screen

Usage:
    .venv/bin/python scripts/design_multisine.py [--seed N] [--no-show]

Run it twice with different seeds to get two phase realisations; differences
between their measured responses that exceed the noise variance are distortion.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO_ROOT, "signals")

# --- timing (must match main.h) -------------------------------------------------
CYCLE_TIME_MS = 0.5        # EtherCAT cycle period, main.h CYCLE_TIME_MS
PERIOD_S = 8.0             # one multisine period; frequency grid = 1 / PERIOD_S

# --- frequency grid ----------------------------------------------------------------
F_MIN_HZ = 0.5             # lowest excited frequency
F_MAX_HZ = 100.0           # highest excited frequency
N_LINES_TARGET = 60        # log-spaced targets before snapping to the odd-harmonic grid
DETECTION_GROUP = 4        # leave one line in every group of this many unexcited
DETECTION_SEED = 1         # fixed, so both phase realisations share the detection lines

# --- amplitude shaping -----------------------------------------------------------
LOW_LINE_A = 0.25          # current amplitude per line below SHAPE_CORNER_HZ, A
HIGH_LINE_A = 0.60         # current amplitude per line once the f^2 rise is capped, A
SHAPE_CORNER_HZ = 3.0      # start of the f^2 rise
PEAK_CURRENT_A = 6.0       # hard budget on |target current|; signal is scaled down to meet it
TRAVEL_LIMIT_MM = 10.0     # predicted |position| above this is flagged

# --- crest-factor optimisation ---------------------------------------------------
CF_ITERATIONS = 1000
CF_CLIP_START = 0.50       # clip level as a fraction of the current peak, first iteration
CF_CLIP_END = 0.95         # ... ramped linearly to this by the last iteration (fixed 0.85 stalled at ~2.3)

# --- second-order plant sanity model (target current -> position) ----------------
# |G| = K / sqrt((1 - r^2)^2 + (2 zeta r)^2), r = f / fn. Fitted by eye to the
# 3 A chirp gains: 4.7 mm/A at 1-2 Hz, 1.35 mm/A at 10 Hz, 0.38 mm/A at 20 Hz.
PLANT_K_MM_PER_A = 4.7
PLANT_FN_HZ = 5.5
PLANT_ZETA = 0.7
G_MPS2 = 9.80665


def plant_response(f_hz: np.ndarray) -> np.ndarray:
    """Complex position response in mm/A of the sanity model at the given frequencies."""
    r = f_hz / PLANT_FN_HZ
    return PLANT_K_MM_PER_A / (1.0 - r * r + 2j * PLANT_ZETA * r)


def choose_lines(n: int, df: float):
    """Pick excited and detection bins on the odd-harmonic grid.

    Returns (excited_bins, detection_bins) as sorted integer arrays.
    """
    k_odd = np.arange(1, n // 2, 2)
    f_odd = k_odd * df
    cand = k_odd[(f_odd >= F_MIN_HZ) & (f_odd <= F_MAX_HZ)]
    if len(cand) == 0:
        sys.exit("no odd harmonics inside the requested band")
    targets = np.geomspace(cand[0] * df, cand[-1] * df, N_LINES_TARGET)
    chosen = sorted({int(cand[np.abs(cand * df - t).argmin()]) for t in targets})
    chosen = np.array(chosen)

    rng = np.random.default_rng(DETECTION_SEED)
    detection = []
    for g in range(0, len(chosen), DETECTION_GROUP):
        group = chosen[g:g + DETECTION_GROUP]
        if len(group) == DETECTION_GROUP:
            detection.append(int(rng.choice(group)))
    detection = np.array(sorted(detection), dtype=int)
    excited = np.array([k for k in chosen if k not in set(detection.tolist())], dtype=int)
    return excited, detection


def line_amplitudes(f_hz: np.ndarray) -> np.ndarray:
    """Amplitude shaping: flat, then f^2 rise from SHAPE_CORNER_HZ, capped at HIGH_LINE_A."""
    rise = (np.maximum(f_hz, SHAPE_CORNER_HZ) / SHAPE_CORNER_HZ) ** 2
    return LOW_LINE_A * np.minimum(rise, HIGH_LINE_A / LOW_LINE_A)


def synthesize(n: int, bins: np.ndarray, amps: np.ndarray, phases: np.ndarray,
               gain: np.ndarray | None = None) -> np.ndarray:
    """One period of sum_k amps[k] * cos(2 pi bins[k] t / N + phases[k]) via inverse FFT.

    With ``gain`` (complex, per line) the lines are multiplied by it first, which
    is how the predicted plant response is built.
    """
    spectrum = np.zeros(n // 2 + 1, dtype=complex)
    lines = 0.5 * n * amps * np.exp(1j * phases)
    if gain is not None:
        lines = lines * gain
    spectrum[bins] = lines
    return np.fft.irfft(spectrum, n)


def crest_factor(x: np.ndarray) -> float:
    return float(np.abs(x).max() / np.sqrt(np.mean(x * x)))


def optimise_phases(n: int, bins: np.ndarray, amps: np.ndarray, seed: int):
    """Random start, then iterative clipping (Van der Ouderaa) keeping the best phases."""
    rng = np.random.default_rng(seed)
    phases = rng.uniform(0.0, 2.0 * math.pi, size=len(bins))
    best_phases = phases.copy()
    best_cf = crest_factor(synthesize(n, bins, amps, phases))
    cf_start = best_cf
    for it in range(CF_ITERATIONS):
        x = synthesize(n, bins, amps, phases)
        ratio = CF_CLIP_START + (CF_CLIP_END - CF_CLIP_START) * it / CF_ITERATIONS
        lim = ratio * np.abs(x).max()
        spectrum = np.fft.rfft(np.clip(x, -lim, lim))
        phases = np.angle(spectrum[bins])
        cf = crest_factor(synthesize(n, bins, amps, phases))
        if cf < best_cf:
            best_cf, best_phases = cf, phases.copy()
    return best_phases, cf_start, best_cf


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seed", type=int, default=1, help="phase realisation seed (default 1)")
    ap.add_argument("--no-show", action="store_true", help="save the figure but do not open a window")
    ap.add_argument("--out-dir", default=OUT_DIR, help=f"output directory (default {OUT_DIR})")
    args = ap.parse_args()

    fs = 1000.0 / CYCLE_TIME_MS
    n = int(round(PERIOD_S * fs))
    if abs(n - PERIOD_S * fs) > 1e-9:
        sys.exit("PERIOD_S must be an integer number of cycles")
    df = fs / n
    t = np.arange(n) / fs

    excited, detection = choose_lines(n, df)
    f_ex = excited * df
    amps = line_amplitudes(f_ex)

    phases, cf_start, cf = optimise_phases(n, excited, amps, args.seed)
    current = synthesize(n, excited, amps, phases)

    # Scale the whole signal (up or down) until the tighter of the two limits is met exactly:
    # peak current PEAK_CURRENT_A, or predicted travel TRAVEL_LIMIT_MM from the sanity model.
    gain = plant_response(f_ex)                      # mm/A, complex
    peak_unscaled = float(np.abs(current).max())
    travel_unscaled = float(np.abs(synthesize(n, excited, amps, phases, gain)).max())
    scale_current = PEAK_CURRENT_A / peak_unscaled
    scale_travel = TRAVEL_LIMIT_MM / travel_unscaled
    scale = min(scale_current, scale_travel)
    binding = "peak current" if scale_current <= scale_travel else "predicted travel"
    amps = amps * scale
    current = current * scale
    peak = float(np.abs(current).max())
    rms = float(np.sqrt(np.mean(current * current)))

    # Predicted plant response (sanity model).
    pos_mm = synthesize(n, excited, amps, phases, gain)
    acc_gain = -((2.0 * math.pi * f_ex) ** 2) * gain * 1e-3 / G_MPS2   # g/A
    acc_g = synthesize(n, excited, amps, phases, acc_gain)
    pos_line_mm = amps * np.abs(gain)
    acc_line_g = amps * np.abs(acc_gain)
    pos_peak = float(np.abs(pos_mm).max())
    acc_peak = float(np.abs(acc_g).max())

    tag = (f"multisine_{PERIOD_S:g}s_{F_MIN_HZ:g}to{F_MAX_HZ:g}Hz_"
           f"{PEAK_CURRENT_A:g}A_seed{args.seed}")
    os.makedirs(args.out_dir, exist_ok=True)
    csv_path = os.path.join(args.out_dir, tag + ".csv")
    json_path = os.path.join(args.out_dir, tag + ".json")
    png_path = os.path.join(args.out_dir, tag + ".png")

    # --- terminal report ---------------------------------------------------------
    print(f"period {PERIOD_S:g} s = {n} samples at {fs:g} Hz, grid {df:g} Hz")
    print(f"excited lines: {len(excited)}, detection lines: {len(detection)}, "
          f"band {f_ex[0]:.3f}..{f_ex[-1]:.3f} Hz")
    print(f"crest factor: {cf_start:.2f} (random start) -> {cf:.2f} (optimised)")
    print(f"line amplitudes scaled by {scale:.3f}; binding limit: {binding} "
          f"({PEAK_CURRENT_A:g} A / {TRAVEL_LIMIT_MM:g} mm)")
    print(f"peak {peak:.3f} A, rms {rms:.3f} A, first sample {current[0]:+.3f} A")
    print(f"predicted (sanity model): |position| <= {pos_peak:.2f} mm, "
          f"|acceleration| <= {acc_peak:.3f} g")
    if pos_peak > TRAVEL_LIMIT_MM:
        print(f"WARNING: predicted travel exceeds TRAVEL_LIMIT_MM = {TRAVEL_LIMIT_MM:g} mm")
    print("\n   bin      f_Hz   amp_A  pos_mm   acc_g")
    for k, f, a, p, g in zip(excited, f_ex, amps, pos_line_mm, acc_line_g):
        print(f"  {k:4d}  {f:8.3f}  {a:6.3f}  {p:6.3f}  {g:6.4f}")
    print("detection bins:", " ".join(f"{k}({k * df:g}Hz)" for k in detection))

    # --- files -----------------------------------------------------------------------
    with open(csv_path, "w") as fh:
        fh.write("index,target_current_A\n")
        for i, v in enumerate(current):
            fh.write(f"{i},{v:.6f}\n")
    design = {
        "cycle_time_ms": CYCLE_TIME_MS, "period_s": PERIOD_S, "samples": n, "grid_hz": df,
        "f_min_hz": F_MIN_HZ, "f_max_hz": F_MAX_HZ, "n_lines_target": N_LINES_TARGET,
        "detection_group": DETECTION_GROUP, "detection_seed": DETECTION_SEED,
        "phase_seed": args.seed, "low_line_a": LOW_LINE_A, "high_line_a": HIGH_LINE_A,
        "shape_corner_hz": SHAPE_CORNER_HZ, "peak_current_a": PEAK_CURRENT_A,
        "cf_iterations": CF_ITERATIONS, "cf_clip_start": CF_CLIP_START, "cf_clip_end": CF_CLIP_END,
        "plant_model": {"k_mm_per_a": PLANT_K_MM_PER_A, "fn_hz": PLANT_FN_HZ, "zeta": PLANT_ZETA},
        "amplitude_scale_applied": scale, "binding_limit": binding,
        "stats": {"peak_a": peak, "rms_a": rms, "crest_factor": cf,
                  "predicted_peak_position_mm": pos_peak, "predicted_peak_acceleration_g": acc_peak},
        "excited_lines": [{"bin": int(k), "f_hz": float(f), "amp_a": float(a), "phase_rad": float(p)}
                          for k, f, a, p in zip(excited, f_ex, amps, phases)],
        "detection_bins": [int(k) for k in detection],
        "csv": os.path.basename(csv_path),
    }
    with open(json_path, "w") as fh:
        json.dump(design, fh, indent=1)

    # --- figure ----------------------------------------------------------------------
    fig, ax = plt.subplots(3, 2, figsize=(15, 10))
    fig.suptitle(f"{tag}   peak {peak:.2f} A, rms {rms:.2f} A, crest {cf:.2f}, "
                 f"predicted |x| <= {pos_peak:.1f} mm, |a| <= {acc_peak:.2f} g")

    ax[0, 0].plot(t, current, lw=0.6, color="tab:blue")
    ax[0, 0].axhline(PEAK_CURRENT_A, color="tab:red", ls="--", lw=0.8)
    ax[0, 0].axhline(-PEAK_CURRENT_A, color="tab:red", ls="--", lw=0.8)
    ax[0, 0].set_ylabel("target current (A)")
    ax[0, 0].set_title("one period, as it will be commanded")

    ax[1, 0].plot(t, pos_mm, lw=0.6, color="tab:brown")
    ax[1, 0].axhline(TRAVEL_LIMIT_MM, color="tab:red", ls="--", lw=0.8)
    ax[1, 0].axhline(-TRAVEL_LIMIT_MM, color="tab:red", ls="--", lw=0.8)
    ax[1, 0].set_ylabel("predicted position (mm)")
    ax[1, 0].set_title("sanity model: 2nd order, K=%.1f mm/A, fn=%.1f Hz, zeta=%.2f"
                       % (PLANT_K_MM_PER_A, PLANT_FN_HZ, PLANT_ZETA))

    ax[2, 0].plot(t, acc_g, lw=0.6, color="tab:cyan")
    ax[2, 0].set_ylabel("predicted acceleration (g)")
    ax[2, 0].set_xlabel("time within period (s)")

    ax[0, 1].stem(f_ex, amps, linefmt="tab:blue", markerfmt="o", basefmt=" ")
    ax[0, 1].vlines(detection * df, 0, amps.max(), color="tab:red", lw=0.8, ls=":",
                    label="detection lines (unexcited)")
    ax[0, 1].set_ylabel("line amplitude (A)")
    ax[0, 1].set_title("excited lines (odd harmonics of %g Hz)" % df)
    ax[0, 1].legend(loc="upper left")

    ax[1, 1].loglog(f_ex, pos_line_mm, "o-", ms=3, color="tab:brown")
    ax[1, 1].set_ylabel("predicted position per line (mm)")

    ax[2, 1].loglog(f_ex, acc_line_g, "o-", ms=3, color="tab:cyan")
    ax[2, 1].set_ylabel("predicted acceleration per line (g)")
    ax[2, 1].set_xlabel("frequency (Hz)")

    for a in ax[:, 1]:
        a.set_xscale("log")
        a.grid(True, which="both", lw=0.3)
    for a in ax[:, 0]:
        a.grid(True, lw=0.3)
        a.set_xlim(0, PERIOD_S)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(png_path, dpi=120)
    print(f"\nwrote {csv_path}\n      {json_path}\n      {png_path}")
    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
