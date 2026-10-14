#!/usr/bin/env python3
"""Replay the position EKF on a logged run with the current delayed by D extra cycles, to choose
POS_KF_DELAY_CYCLES offline.

    python scripts/ekf_delay_scan.py [path/to/log.csv] [--max-delay 8]

With no path the newest log in gcsc_data/ is used. The filter is a line-by-line port of vca_ekf_step()
in vca_ekf.c (Method C model from vca_model.h, POS_KF_SIG_* from closed_loop_settings.h), run on the raw
position_mm at 2 kHz. Its input for the step k-1 -> k is the current sent in cycle k-1-D; the firmware
today uses D = 0. The current sent in cycle k is pid_output_A of row k-1 (control_loop.c sends the
previous cycle's output).

Check: with D = 0 the replayed innovation must equal the logged kf_innovation_mm (printed as the
largest difference); if not, the port or the settings headers differ from the run.

For each D it prints, over t >= 0, the innovation RMS, its mean and innovation std / predicted std
(1 if the model and noise levels fit), overall and per window of the chirp (frequency from the file
name). The best D keeps the ratio nearest 1 and the mean nearest 0 at high frequency.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_voice_coil_log import REPO_ROOT, latest_log  # noqa: E402

TS = 0.5e-3                     # CYCLE_TIME_MS in main.h
WINDOW_S = 1.0                  # width of the per-frequency windows
WINDOW_HZ = (2.0, 5.0, 10.0, 15.0, 20.0, 22.5, 25.0, 27.5, 30.0, 35.0, 40.0, 45.0, 50.0)


def defines(*names: str) -> dict[str, str]:
    """#define values from the firmware headers."""
    text = "\n".join(open(os.path.join(REPO_ROOT, n)).read() for n in names)
    return dict(re.findall(r"^#define\s+(\w+)\s+([-+\d.eE]+)\b", text, re.MULTILINE))


def model() -> dict[str, float]:
    d = defines("vca_model.h", "closed_loop_settings.h")
    keys = ["VCA_MASS_KG", "VCA_GAMMA_N_PER_A", "VCA_GAMMA1_N_PER_AM", "VCA_DAMPING_NS_PER_M", "VCA_C_R_NM3",
            "VCA_C_L_NM3", "VCA_GAP_M", "POS_KF_SIG_A_M_S2", "POS_KF_SIG_Y_MM", "POS_KF_MAINS_ENABLE",
            "POS_KF_MAINS_PICKUP_MM"]
    missing = [k for k in keys if k not in d]
    if missing:
        sys.exit(f"not found in the headers: {', '.join(missing)}")
    return {k: float(d[k]) for k in keys}


def ekf_replay(p: dict, y_mm: np.ndarray, u_A: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Port of vca_ekf_step(): innovation and its predicted std (mm) per row; u_A[k] is the input for
    the step k-1 -> k."""
    m, G, G1, c = p["VCA_MASS_KG"], p["VCA_GAMMA_N_PER_A"], p["VCA_GAMMA1_N_PER_AM"], p["VCA_DAMPING_NS_PER_M"]
    CR, CL, g0 = p["VCA_C_R_NM3"], p["VCA_C_L_NM3"], p["VCA_GAP_M"]
    sa2 = p["POS_KF_SIG_A_M_S2"] ** 2
    q11, q12, q22 = sa2 * TS ** 4 / 4, sa2 * TS ** 3 / 2, sa2 * TS ** 2
    R = (p["POS_KF_SIG_Y_MM"] * 1e-3) ** 2
    if p.get("POS_KF_MAINS_ENABLE"):   # this 2-state filter has no pickup state: count it as noise (vca_ekf_init)
        R += (p["POS_KF_MAINS_PICKUP_MM"] * 1e-3) ** 2 / 2
    ts = TS

    def acc(x, v, u):
        gr, gl = g0 - x, g0 + x
        return ((G + G1 * x) * u - c * v - (CR / (gr * gr * gr) - CL / (gl * gl * gl))) / m

    ys, us = (y_mm * 1e-3).tolist(), u_A.tolist()       # python floats: the scalar loop runs faster
    n = len(ys)
    innov, istd = [math.nan] * n, [math.nan] * n
    x = v = p11 = p12 = p22 = 0.0
    seeded = False
    for k in range(n):
        y, u = ys[k], us[k]
        if not math.isfinite(y):
            continue
        if not seeded:
            x, v, p11, p12, p22, seeded = y, 0.0, R, 0.0, 1e-4, True
        else:
            k1x, k1v = v, acc(x, v, u)
            k2x, k2v = v + ts / 2 * k1v, acc(x + ts / 2 * k1x, v + ts / 2 * k1v, u)
            k3x, k3v = v + ts / 2 * k2v, acc(x + ts / 2 * k2x, v + ts / 2 * k2v, u)
            k4x, k4v = v + ts * k3v, acc(x + ts * k3x, v + ts * k3v, u)
            x, v = x + ts / 6 * (k1x + 2 * k2x + 2 * k3x + k4x), v + ts / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
            gr, gl = g0 - x, g0 + x
            a21 = -(3 * CR / gr ** 4 + 3 * CL / gl ** 4 - G1 * u) / m
            a22 = -c / m
            f11, f12 = 1 + a21 * ts * ts / 2, ts + a22 * ts * ts / 2
            f21, f22 = a21 * ts + a21 * a22 * ts * ts / 2, 1 + a22 * ts + (a21 + a22 * a22) * ts * ts / 2
            g11, g12 = f11 * p11 + f12 * p12, f11 * p12 + f12 * p22
            g21, g22 = f21 * p11 + f22 * p12, f21 * p12 + f22 * p22
            p11, p12, p22 = g11 * f11 + g12 * f12 + q11, g11 * f21 + g12 * f22 + q12, g21 * f21 + g22 * f22 + q22
        S = p11 + R
        k1, k2, e = p11 / S, p12 / S, y - x
        x, v = x + k1 * e, v + k2 * e
        p11, p12, p22 = p11 - k1 * p11, p12 - k1 * p12, p22 - k2 * p12
        innov[k], istd[k] = e * 1e3, math.sqrt(S) * 1e3
        if not all(map(math.isfinite, (x, v, p11, p12, p22))):
            x, v, p11, p12, p22 = y, 0.0, R, 0.0, 1e-4
    return np.array(innov), np.array(istd)


def chirp_time(path: str, f_hz: float) -> float | None:
    """Time at which the exponential chirp in the file name passes f_hz (None outside the sweep); for a
    chirp_hold run at f1, the middle of the hold."""
    name = os.path.basename(path)
    m = re.search(r"_chirp(?:hold)?_[-\d.]+mm_[\d.]+mm_([\d.]+)to([\d.]+)Hz_([\d.]+)s", name)
    if not m:
        return None
    f0, f1, T = map(float, m.groups())
    if not min(f0, f1) <= f_hz <= max(f0, f1) or f0 == f1:
        return None
    hold = re.search(r"_f1hold([\d.]+)s", name)
    if hold and f_hz == f1:
        return T + float(hold[1]) / 2
    return T * math.log(f_hz / f0) / math.log(f1 / f0)


def stats(e: np.ndarray, s: np.ndarray, mask: np.ndarray) -> str:
    ok = mask & np.isfinite(e)
    if ok.sum() < 100:
        return "        -"
    return f"{np.sqrt(np.mean(e[ok] ** 2)):.4f} {np.mean(e[ok]):+.4f} {np.std(e[ok]) / np.mean(s[ok]):4.2f}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", nargs="?", help="log CSV (default: newest in gcsc_data/)")
    parser.add_argument("--max-delay", type=int, default=8, help="largest extra delay D to try, in cycles")
    args = parser.parse_args()
    path = args.log or latest_log()
    p = model()
    a = np.genfromtxt(path, delimiter=",", names=True, autostrip=True)
    t, y = a["time_s"], a["position_mm"]
    out = np.nan_to_num(a["pid_output_A"], nan=0.0)        # 0 A in the idle window
    sent = np.concatenate(([0.0], out[:-1]))                # sent in cycle k: output of row k-1
    print(os.path.basename(path))
    print(f"model: m {p['VCA_MASS_KG']} kg, Gamma {p['VCA_GAMMA_N_PER_A']} N/A; "
          f"SIG_A {p['POS_KF_SIG_A_M_S2']} m/s^2, SIG_Y {p['POS_KF_SIG_Y_MM']} mm (current headers)")

    f1 = re.search(r"to([\d.]+)Hz_.*_f1hold", os.path.basename(path))   # chirp_hold: also a window in the hold
    windows = [(f, chirp_time(path, f)) for f in sorted(set(WINDOW_HZ) | ({float(f1[1])} if f1 else set()))]
    windows = [(f, tc) for f, tc in windows if tc is not None and tc + WINDOW_S / 2 <= t[-1]]
    run = t >= 0.0
    head = f"{'D':>2} {'ms':>4} | {'all t >= 0':^20} | " + " | ".join(f"{f:^20g}" for f, _ in windows)
    print("\ncolumns per block: innovation RMS (mm), mean (mm), std / predicted std; blocks by chirp frequency (Hz)")
    print(head)
    print("-" * len(head))
    for D in range(args.max_delay + 1):
        u = np.concatenate((np.zeros(D + 1), sent))[: len(sent)]   # input for step k-1 -> k: sent in cycle k-1-D
        e, s = ekf_replay(p, y, u)
        if D == 0 and np.isfinite(a["kf_innovation_mm"]).any():
            diff = np.nanmax(np.abs(e - a["kf_innovation_mm"]))
            print(f"   (check, D = 0 vs logged kf_innovation_mm: largest difference {diff:.2e} mm)")
        cells = [stats(e, s, run)] + [stats(e, s, (t > tc - WINDOW_S / 2) & (t < tc + WINDOW_S / 2))
                                      for _, tc in windows]
        print(f"{D:2d} {D * TS * 1e3:4.1f} | " + " | ".join(f"{c:^20}" for c in cells))


if __name__ == "__main__":
    main()
