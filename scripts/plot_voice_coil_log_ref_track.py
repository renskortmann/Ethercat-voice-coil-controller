#!/usr/bin/env python3
"""Position-control reference and tracking plots (EXPERIMENT_POSITION_PID).

Two uses:

* Preview, before a run (no hardware, no log):
      python scripts/plot_voice_coil_log_ref_track.py --preview
  Reads the reference settings straight from main.h (POS_REF_SHAPE, POS_REF_STEPS,
  POS_REF_RAMP_S, POS_REF_SINE_*, RUN_DURATION_S, BIAS_IDLE_S) and plots the
  reference the controller will track, with the allowed reference window and the
  position trip limits. Also prints the breakpoint table.

* Tracking, after a run:
      python scripts/plot_voice_coil_log_ref_track.py [path/to/log.csv]
  With no path the newest log in gcsc_data/ is used. Three stacked axes:
    position (mm)  -- measured position (raw and 60 Hz low-pass) over the reference
    error (mm)     -- reference - position (low-pass), the tracking error
    current (A)    -- PI output, its P and I terms, and the measured coil current
  The reference comes from the log's position_ref_mm column, i.e. exactly what the
  controller used. If the log has no such column it falls back to the main.h
  reference and says so. If main.h has changed since the run, a note is printed.

The reference semantics mirror pos_ref_mm() in control_loop.c: at each breakpoint
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
from plot_voice_coil_log import REPO_ROOT, latest_log, low_pass  # noqa: E402

MAIN_H = os.path.join(REPO_ROOT, "main.h")
POSITION_LP_CUTOFF_HZ = 60.0  # same low-pass as the position trace in plot_voice_coil_log.py
PREVIEW_DT_S = 0.005


def read_main_h(path: str = MAIN_H) -> dict:
    """Reference settings from main.h, using the same #define names as the firmware."""
    text = open(path).read()

    def define(name: str) -> str:
        m = re.search(rf"^#define\s+{name}\s+(.+?)\s*(?:/\*.*)?(?://.*)?$", text, re.MULTILINE)
        if not m:
            sys.exit(f"{name} not found in {path}")
        return m.group(1).strip()

    def number(name: str) -> float:
        return float(define(name))

    shape = define("POS_REF_SHAPE")
    if shape not in ("POS_REF_SHAPE_STEPS", "POS_REF_SHAPE_SINE"):
        sys.exit(f"unknown POS_REF_SHAPE {shape!r} in {path}")
    steps = [(float(t), float(p)) for t, p in
             re.findall(r"X\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)", define(r"POS_REF_STEPS\(X\)"))]
    return {
        "mode": define("EXPERIMENT_MODE"),
        "shape": "steps" if shape == "POS_REF_SHAPE_STEPS" else "sine",
        "steps": steps,
        "ramp_s": number("POS_REF_RAMP_S"),
        "sine_offset_mm": number("POS_REF_SINE_OFFSET_MM"),
        "sine_amplitude_mm": number("POS_REF_SINE_AMPLITUDE_MM"),
        "sine_freq_hz": number("POS_REF_SINE_FREQ_HZ"),
        "run_s": number("RUN_DURATION_S"),
        "idle_s": number("BIAS_IDLE_S"),
        "ref_min_mm": number("POS_REF_MIN_MM"),
        "ref_max_mm": number("POS_REF_MAX_MM"),
        "trip_min_mm": number("POS_TRIP_MIN_MM"),
        "trip_max_mm": number("POS_TRIP_MAX_MM"),
        "kp": number("PID_KP_A_PER_MM"),
        "ki": number("PID_KI_A_PER_MM_S"),
    }


def reference_mm(cfg: dict, t: float) -> float:
    """Reference at time t, mirroring pos_ref_mm() in control_loop.c (nan in the idle window)."""
    if t < 0.0:
        return float("nan")
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
    lines.append(f"Kp = {cfg['kp']} A/mm, Ki = {cfg['ki']} A/(mm s)")
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
    ax.set_title("Programmed position reference (from main.h, not from a run)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize="small")
    fig.tight_layout()
    plt.show()


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


def tracking(cfg: dict, path: str) -> None:
    names = ["position_mm", "position_ref_mm", "pid_p_A", "pid_i_A", "pid_output_A", "actual_current_A"]
    t, c = read_columns(path, names)
    if not t:
        sys.exit(f"no usable rows in {path}")
    y = c["position_mm"]
    r = c["position_ref_mm"]
    from_main_h = [reference_mm(cfg, x) for x in t]
    if not any(math.isfinite(v) for v in r):
        print("note: this log has no position_ref_mm column (not a position-control run); "
              "showing the CURRENT main.h reference instead")
        r = from_main_h
    else:
        diff = max((abs(a - b) for a, b in zip(r, from_main_h) if math.isfinite(a) and math.isfinite(b)), default=0.0)
        if diff > 1e-3:
            print(f"note: main.h's reference differs from this run's by up to {diff:.3f} mm "
                  "(main.h changed since the run); plotting the run's own reference")

    y_lp = low_pass(t, y, POSITION_LP_CUTOFF_HZ)
    err_lp = [a - b if math.isfinite(a) else float("nan") for a, b in zip(r, y_lp)]
    run = [i for i, x in enumerate(t) if x >= 0.0 and math.isfinite(err_lp[i])]
    if run:
        rms = math.sqrt(sum(err_lp[i] ** 2 for i in run) / len(run))
        print(f"tracking error (60 Hz low-passed), t >= 0: rms {rms:.3f} mm, "
              f"max |e| {max(abs(err_lp[i]) for i in run):.3f} mm")
    u = [v for v in c["pid_output_A"] if math.isfinite(v)]
    if u:
        print(f"PI output: max |u| {max(abs(v) for v in u):.3f} A")

    fig, (ax_pos, ax_err, ax_cur) = plt.subplots(3, 1, sharex=True, figsize=(11, 9),
                                                gridspec_kw={"height_ratios": [3, 1.5, 2]})
    fig.suptitle(os.path.basename(path), fontsize="medium")
    for ax in (ax_pos, ax_err, ax_cur):
        shade_idle(ax, t[0])
        ax.grid(True, alpha=0.3)

    ax_pos.plot(t, y, color="tab:brown", lw=0.6, alpha=0.5, label="position (raw)")
    ax_pos.plot(t, y_lp, color="black", lw=1.0, label=f"position ({POSITION_LP_CUTOFF_HZ:.0f} Hz low-pass)")
    ax_pos.plot(t, r, color="tab:red", lw=1.6, label="reference")
    draw_limits(ax_pos, cfg)
    ax_pos.set_ylabel("position_mm (mm)")
    ax_pos.legend(loc="upper right", fontsize="small", ncol=2)

    ax_err.plot(t, err_lp, color="tab:purple", lw=1.0)
    ax_err.axhline(0.0, color="0.5", lw=0.8)
    ax_err.set_ylabel("error r - y (mm)")

    ax_cur.plot(t, c["actual_current_A"], color="tab:blue", lw=0.8, alpha=0.6, label="actual current")
    ax_cur.plot(t, c["pid_output_A"], color="tab:purple", lw=1.2, label="PI output u (sent next cycle)")
    ax_cur.plot(t, c["pid_p_A"], color="tab:green", lw=0.8, label="P term")
    ax_cur.plot(t, c["pid_i_A"], color="tab:orange", lw=0.8, label="I term")
    ax_cur.set_ylabel("current (A)")
    ax_cur.set_xlabel("time_s (s)")
    ax_cur.legend(loc="upper right", fontsize="small", ncol=2)

    fig.tight_layout()
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", nargs="?", help="log CSV (default: newest in gcsc_data/)")
    parser.add_argument("--preview", action="store_true",
                        help="plot the reference programmed in main.h, without a log")
    args = parser.parse_args()
    cfg = read_main_h()
    if args.preview:
        preview(cfg)
    else:
        tracking(cfg, args.log or latest_log())


if __name__ == "__main__":
    main()
