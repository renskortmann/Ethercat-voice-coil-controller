#!/usr/bin/env python3
"""Replay the position EKF on a logged run with and without a 50 Hz mains-pickup state, to see offline
whether modelling the pickup keeps it out of the estimate the controller uses.

    python scripts/ekf_mains_replay.py [path/to/log.csv] [--delay 4] [--hold 121 139] [--bw 0.1 0.3 1.0]

With no path the newest log in gcsc_data/ is used. Filters compared, all with the current delayed by
--delay cycles as in the firmware (POS_KF_DELAY_CYCLES):
  * baseline: the firmware EKF, state [x, v] (port of vca_ekf_step(); checked against the logged
    innovation when the delay matches the run)
  * mains, B Hz: state [x, v, c, s]; c, s are the in-phase / quadrature parts of a 50 Hz pickup on the
    laser, y = x + c + e. Each step [c, s] is rotated by 2 pi 50 Ts plus a random walk whose size sets the
    tracking bandwidth B of the pickup estimate (about a notch of width B on the innovation only). The
    current does not drive c, s. The measurement noise is the idle-window laser noise with its 50 Hz
    line removed (the 150 Hz line stays in it).
For each filter the SMC output is recomputed open-loop from the filter's prediction at the time the
output acts (same law and settings as controller_smc.c, current settings headers), so it shows the 50 Hz
current the controller would have commanded. The real plant would respond differently to a different
current, so this is the controller's reaction to the same measured data, not a closed-loop prediction.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_voice_coil_log import latest_log  # noqa: E402
from ekf_delay_scan import TS, defines, model  # noqa: E402
import plot_voice_coil_log_ref_track as ref_track  # noqa: E402

F_MAINS = 50.0


def acc_fn(p: dict):
    m, G, G1, c = p["VCA_MASS_KG"], p["VCA_GAMMA_N_PER_A"], p["VCA_GAMMA1_N_PER_AM"], p["VCA_DAMPING_NS_PER_M"]
    CR, CL, g0 = p["VCA_C_R_NM3"], p["VCA_C_L_NM3"], p["VCA_GAP_M"]

    def acc(x, v, u):
        gr, gl = g0 - x, g0 + x
        return ((G + G1 * x) * u - c * v - (CR / (gr * gr * gr) - CL / (gl * gl * gl))) / m
    return acc


def rk4(acc, x, v, u, ts=TS):
    k1x, k1v = v, acc(x, v, u)
    k2x, k2v = v + ts / 2 * k1v, acc(x + ts / 2 * k1x, v + ts / 2 * k1v, u)
    k3x, k3v = v + ts / 2 * k2v, acc(x + ts / 2 * k2x, v + ts / 2 * k2v, u)
    k4x, k4v = v + ts * k3v, acc(x + ts * k3x, v + ts * k3v, u)
    return x + ts / 6 * (k1x + 2 * k2x + 2 * k3x + k4x), v + ts / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)


class Ekf:
    """Port of vca_ekf_step() / vca_ekf_predict_n(). q_pickup_mm None: the firmware 2-state filter [x, v].
    Otherwise state [x, v, c1, s1, c2, s2, ...] with a pickup on the measurement at each frequency in
    harmonics (default 50 Hz), y = x + c1 + c2 + ... + e, whose c, s random walk has this std per step (mm);
    pickup0_mm (one value or one per harmonic) is the initial std of c, s. SI units inside."""

    def __init__(self, p: dict, sig_y_mm: float, q_pickup_mm: float | None, pickup0_mm=0.094,
                 harmonics=(F_MAINS,)):
        self.acc = acc_fn(p)
        self.m, self.G1, self.c_d = p["VCA_MASS_KG"], p["VCA_GAMMA1_N_PER_AM"], p["VCA_DAMPING_NS_PER_M"]
        self.CR, self.CL, self.g0 = p["VCA_C_R_NM3"], p["VCA_C_L_NM3"], p["VCA_GAP_M"]
        sa2 = p["POS_KF_SIG_A_M_S2"] ** 2
        ts = TS
        self.mains = q_pickup_mm is not None
        nh = len(harmonics) if self.mains else 0
        n = self.n = 2 + 2 * nh
        self.Q = np.zeros((n, n))
        self.Q[:2, :2] = [[sa2 * ts ** 4 / 4, sa2 * ts ** 3 / 2], [sa2 * ts ** 3 / 2, sa2 * ts ** 2]]
        self.rot = np.eye(n)
        self.H = np.zeros(n)
        self.H[0] = 1.0
        p0 = np.broadcast_to(np.asarray(pickup0_mm, dtype=float), (max(nh, 1),))
        self.pickup_var = np.zeros(n)
        for h in range(nh):
            i = 2 + 2 * h
            self.Q[i, i] = self.Q[i + 1, i + 1] = (q_pickup_mm * 1e-3) ** 2
            th = 2 * math.pi * harmonics[h] * ts
            self.rot[i:i + 2, i:i + 2] = [[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]]
            self.H[i] = 1.0
            self.pickup_var[i] = self.pickup_var[i + 1] = (p0[h] * 1e-3) ** 2
        self.R = (sig_y_mm * 1e-3) ** 2
        self.z = None
        self.P = None

    def step(self, y_mm: float, u: float) -> tuple[float, float]:
        """Filter one sample; u is the current acting over the step k-1 -> k. Returns innovation and its
        predicted std in mm."""
        ts, n, y = TS, self.n, y_mm * 1e-3
        if self.z is None:
            self.z = np.zeros(n)
            self.z[0] = y
            self.P = np.zeros((n, n))
            self.P[0, 0], self.P[1, 1] = self.R, 1e-4
            self.P[2:, 2:] = np.diag(self.pickup_var[2:])   # pickup unknown at the start
        else:
            z = self.z
            x, v = rk4(self.acc, z[0], z[1], u)
            gr, gl = self.g0 - x, self.g0 + x
            a21 = -(3 * self.CR / gr ** 4 + 3 * self.CL / gl ** 4 - self.G1 * u) / self.m
            a22 = -self.c_d / self.m
            F = np.eye(n)
            F[0, 0], F[0, 1] = 1 + a21 * ts * ts / 2, ts + a22 * ts * ts / 2
            F[1, 0], F[1, 1] = a21 * ts + a21 * a22 * ts * ts / 2, 1 + a22 * ts + (a21 + a22 * a22) * ts * ts / 2
            z[0], z[1] = x, v
            if self.mains:
                F[2:, 2:] = self.rot[2:, 2:]
                z[2:] = self.rot[2:, 2:] @ z[2:]
            self.P = F @ self.P @ F.T + self.Q
        PH = self.P @ self.H
        S = self.H @ PH + self.R
        K = PH / S
        e = y - self.H @ self.z
        self.z = self.z + K * e
        self.P = self.P - np.outer(K, PH)
        self.P = 0.5 * (self.P + self.P.T)
        return e * 1e3, math.sqrt(S) * 1e3

    def predict(self, us) -> tuple[float, float]:
        """x, v in mm, mm/s after len(us) model steps with these currents, filter state unchanged."""
        x, v = self.z[0], self.z[1]
        for u in us:
            x, v = rk4(self.acc, x, v, u)
        return x * 1e3, v * 1e3


def replay(p: dict, y_mm: np.ndarray, sent_A: np.ndarray, delay: int, sig_y_mm: float, q_pickup_mm: float | None,
           pickup0_mm: float = 0.094):
    """The EKF over a log. Returns per row: x(k|k), v(k|k), pickup c(k|k), innovation, its predicted std
    (mm, mm/s), and the prediction x, v at the time the output acts."""
    f = Ekf(p, sig_y_mm, q_pickup_mm, pickup0_mm)
    N = len(y_mm)
    out = {k: np.full(N, np.nan) for k in ("x", "v", "pick", "innov", "istd", "xp", "vp")}
    us = sent_A.tolist()
    sent = lambda j: us[j] if j >= 0 else 0.0
    for k in range(N):
        out["innov"][k], out["istd"][k] = f.step(y_mm[k], sent(k - 1 - delay))     # sent in cycle k-1-D
        out["x"][k], out["v"][k] = f.z[0] * 1e3, f.z[1] * 1e3
        if f.mains:
            out["pick"][k] = f.z[2] * 1e3
        out["xp"][k], out["vp"][k] = f.predict([sent(j) for j in range(k - delay, k + 1)])
    return out


def smc_output(p: dict, s: dict, t: np.ndarray, xp_mm, vp_mm, delay: int, cfg: dict):
    """Open-loop SMC law of controller_smc.c on the given predictions; returns (u, eq, sw) in A."""
    acc = acc_fn(p)
    a = s["SMC_A_FREQ_FACTOR"] * 2 * math.pi * max(cfg["chirp_f0_hz"], cfg["chirp_f1_hz"])
    phi = a * s["SMC_LAYER_MM"] * 1e-3
    eta = s["SMC_ETA_SIGMAS"] * p["POS_KF_SIG_A_M_S2"]
    lim = s["OUTPUT_LIMIT_A"]
    G, G1, m = p["VCA_GAMMA_N_PER_A"], p["VCA_GAMMA1_N_PER_AM"], p["VCA_MASS_KG"]
    h = 2e-5
    t_act = t + (1 + delay) * TS
    ref = np.vectorize(lambda tt: ref_track.reference_mm(cfg, tt))
    r0, rp, rm = ref(t_act), ref(t_act + h), ref(t_act - h)
    r, rd, rdd = r0 * 1e-3, (rp - rm) / (2 * h) * 1e-3, (rp - 2 * r0 + rm) / (h * h) * 1e-3
    x, v = xp_mm * 1e-3, vp_mm * 1e-3
    g = (G + G1 * x) / m
    z1, z2 = x - r, v - rd
    sv = a * z1 + z2
    eq = (rdd - a * z2 - acc(x, v, 0.0)) / g
    sw = -eta * np.clip(sv / phi, -1, 1) / g
    u = np.clip(eq + sw, -lim, lim)
    return u, eq, sw


def simulate(p: dict, st: dict, f_ref: float, amp_mm: float, delay: int, filt: Ekf, noise: dict,
             plant_gain: float = 1.0, mains_hz: float = F_MAINS, dur_s: float = 12.0, seed: int = 1):
    """Closed loop: Method C plant (motor constant scaled by plant_gain) driven by the current sent D+1
    cycles earlier, laser y = x + 50 Hz and 150 Hz pickup + white noise, the EKF and the SMC of
    controller_smc.c. Reference: amp_mm sine at f_ref, faded in over the first second.
    Returns t and per-cycle true x, laser y, reference r (mm) and output u (A)."""
    rng = np.random.default_rng(seed)
    pp = dict(p, VCA_GAMMA_N_PER_A=p["VCA_GAMMA_N_PER_A"] * plant_gain,
              VCA_GAMMA1_N_PER_AM=p["VCA_GAMMA1_N_PER_AM"] * plant_gain)
    acc_true, acc = acc_fn(pp), acc_fn(p)
    a = st["SMC_A_FREQ_FACTOR"] * 2 * math.pi * f_ref
    phi = a * st["SMC_LAYER_MM"] * 1e-3
    eta = st["SMC_ETA_SIGMAS"] * p["POS_KF_SIG_A_M_S2"]
    lim = st["OUTPUT_LIMIT_A"]
    G, G1, m = p["VCA_GAMMA_N_PER_A"], p["VCA_GAMMA1_N_PER_AM"], p["VCA_MASS_KG"]
    w = 2 * math.pi * f_ref

    def ref(t):                                   # mm, mm/s, mm/s^2 (raised-cosine fade-in over 1 s)
        if t <= 0:
            return 0.0, 0.0, 0.0
        if t < 1.0:
            e, ed, edd = 0.5 * (1 - math.cos(math.pi * t)), 0.5 * math.pi * math.sin(math.pi * t), \
                0.5 * math.pi ** 2 * math.cos(math.pi * t)
        else:
            e, ed, edd = 1.0, 0.0, 0.0
        sn, cs = math.sin(w * t), math.cos(w * t)
        return (amp_mm * e * sn, amp_mm * (ed * sn + e * w * cs),
                amp_mm * (edd * sn + 2 * ed * w * cs - e * w * w * sn))

    N = int(dur_s / TS)
    t = np.arange(N) * TS
    xs, ys, rs, uo = (np.zeros(N) for _ in range(4))
    sent = [0.0] * N                               # current sent in each cycle
    x, v = 0.0, 0.0                                # true plant, SI
    sa = p["POS_KF_SIG_A_M_S2"]
    ph50, ph150 = rng.uniform(0, 2 * math.pi, 2)
    out_prev = 0.0
    for k in range(N):
        sent[k] = out_prev
        if k > 0:                                  # plant step k-1 -> k with the current sent in k-1-D
            u_act = sent[k - 1 - delay] if k - 1 - delay >= 0 else 0.0
            x, v = rk4(acc_true, x, v, u_act)
            v += sa * TS * rng.standard_normal()   # white acceleration disturbance
        tk = t[k]
        y = (x * 1e3 + noise["a50"] * math.sin(2 * math.pi * mains_hz * tk + ph50)
             + noise["a150"] * math.sin(2 * math.pi * 3 * mains_hz * tk + ph150)
             + noise["white"] * rng.standard_normal())
        filt.step(y, sent[k - 1 - delay] if k - 1 - delay >= 0 else 0.0)
        xp, vp = filt.predict([sent[j] if j >= 0 else 0.0 for j in range(k - delay, k + 1)])
        r, rd, rdd = ref(tk + (1 + delay) * TS)
        xp, vp, r, rd, rdd = xp * 1e-3, vp * 1e-3, r * 1e-3, rd * 1e-3, rdd * 1e-3
        g = (G + G1 * xp) / m
        z2 = vp - rd
        sv = a * (xp - r) + z2
        u = (rdd - a * z2 - acc(xp, vp, 0.0)) / g - eta * max(-1.0, min(1.0, sv / phi)) / g
        out_prev = max(-lim, min(lim, u))
        xs[k], ys[k], rs[k], uo[k] = x * 1e3, y, ref(tk)[0], out_prev
    return t, xs, ys, rs, uo


def phasor(t, x, f, mask):
    tt, xx = t[mask], x[mask]
    ok = np.isfinite(xx)
    tt, xx = tt[ok], xx[ok] - xx[ok].mean()
    return np.mean(xx * np.exp(-2j * math.pi * f * tt)) * 2


def envelope(t, x, f, mask, win=0.2):
    """Min and max of the amplitude at f over windows of win seconds (the beat shows as max - min)."""
    tt, xx = t[mask], x[mask] - np.nanmean(x[mask])
    zz = xx * np.exp(-2j * math.pi * f * tt)
    nw = int(win / TS)
    k = len(zz) // nw
    amp = np.abs(zz[: k * nw].reshape(k, nw).mean(1) * 2)
    return amp.min(), amp.max()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log", nargs="?", help="log CSV (default: newest in gcsc_data/)")
    parser.add_argument("--delay", type=int, default=4, help="POS_KF_DELAY_CYCLES the replay uses")
    parser.add_argument("--hold", type=float, nargs=2, default=(121.0, 139.0), help="analysis window, s")
    parser.add_argument("--bw", type=float, nargs="+", default=None,
                        help="pickup tracking bandwidths B to try, Hz (default: POS_KF_MAINS_BW_HZ)")
    parser.add_argument("--sim", action="store_true",
                        help="closed-loop simulation with the measured pickup and noise instead of the replay")
    parser.add_argument("--plant-gain", type=float, nargs="+", default=(1.0, 1.4),
                        help="--sim: plant motor constant over the model's (1.4 mimics the 51 Hz overshoot)")
    parser.add_argument("--mains-hz", type=float, default=F_MAINS, help="--sim: actual mains frequency")
    args = parser.parse_args()
    path = args.log or latest_log()
    p = model()
    s = {k: float(v) for k, v in defines("closed_loop_settings.h").items()}
    cfg = ref_track.read_settings()
    a = np.genfromtxt(path, delimiter=",", names=True, autostrip=True)
    t, y = a["time_s"], a["position_mm"]
    out_logged = np.nan_to_num(a["pid_output_A"], nan=0.0)
    sent = np.concatenate(([0.0], out_logged[:-1]))            # sent in cycle k: output of row k-1
    idle = (t < 0) & (t > t[0] + 0.5)
    hold = (t >= args.hold[0]) & (t < args.hold[1])
    f_ref = max(cfg["chirp_f0_hz"], cfg["chirp_f1_hz"])
    print(os.path.basename(path))
    ref_diff = np.nanmax(np.abs(np.vectorize(lambda tt: ref_track.reference_mm(cfg, tt))(t[hold]) - a["position_ref_mm"][hold]))
    print(f"reference from the settings headers vs logged, in the window: largest difference {ref_diff:.2e} mm")

    # Measurement noise without the 50 Hz pickup: idle-window laser minus its fitted 50 Hz sine
    P50 = phasor(t, y, F_MAINS, idle)
    yi = y[idle] - np.mean(y[idle]) - np.real(P50 * np.exp(2j * math.pi * F_MAINS * t[idle]))
    sig_y_white = float(np.std(yi))
    print(f"idle window: 50 Hz pickup {abs(P50):.4f} mm; laser std {np.std(y[idle]):.4f} mm, "
          f"{sig_y_white:.4f} mm without the 50 Hz line (firmware POS_KF_SIG_Y_MM {p['POS_KF_SIG_Y_MM']})")

    # as vca_ekf_init(): without the pickup state its power is counted as noise
    pick0 = s["POS_KF_MAINS_PICKUP_MM"]
    runs = [("baseline (no pickup state)", None, math.sqrt(p["POS_KF_SIG_Y_MM"] ** 2 + pick0 ** 2 / 2))]
    for B in args.bw or [s["POS_KF_MAINS_BW_HZ"]]:
        # steady-state gain of a random-walk phasor seen through noise r: k ~ sqrt(q / r); bandwidth k fs / (2 pi)
        q = 2 * math.pi * B * TS * p["POS_KF_SIG_Y_MM"]
        runs.append((f"mains 50 Hz, B {B:g} Hz", q, p["POS_KF_SIG_Y_MM"]))

    if args.sim:
        P150 = phasor(t, y, 3 * F_MAINS, idle)
        noise = {"a50": abs(P50), "a150": abs(P150),
                 "white": math.sqrt(max(sig_y_white ** 2 - abs(P150) ** 2 / 2, 1e-6))}
        amp = cfg["chirp_amplitude_mm"]
        print(f"\nclosed-loop simulation: {amp} mm at {f_ref:g} Hz, delay {args.delay}, mains {args.mains_hz} Hz; "
              f"laser = x + {noise['a50']:.3f} mm @50 Hz + {noise['a150']:.3f} mm @150 Hz + {noise['white']:.3f} mm white; "
              "analysis t = 4 .. 12 s, all on the TRUE position")
        head = (f"{'plant':>5} {'filter':24} | {'true x 50Hz':>11} {'u 50Hz':>6} | {'true x/r @f1':>12} {'deg':>6} | "
                f"{'true x env min-max':>18} | {'err rms true':>12} {'err rms laser':>13} | {'max |u|':>7}")
        print(head)
        print("-" * len(head))
        for gain in args.plant_gain:
            for name, q, sy in runs:
                ts_, xs, ys, rs, uo = simulate(p, s, f_ref, amp, args.delay, Ekf(p, sy, q, pick0), noise, gain, args.mains_hz)
                w = ts_ >= 4.0
                X = phasor(ts_, xs, f_ref, w) / phasor(ts_, rs, f_ref, w)
                e0, e1 = envelope(ts_, xs, f_ref, w)
                print(f"{gain:5.2f} {name:24} | {abs(phasor(ts_, xs, F_MAINS, w)):11.3f} {abs(phasor(ts_, uo, F_MAINS, w)):6.2f} | "
                      f"{abs(X):12.3f} {math.degrees(np.angle(X)):6.1f} | {e0:8.3f} - {e1:7.3f} | "
                      f"{np.std(rs[w] - xs[w]):12.3f} {np.std(rs[w] - ys[w]):13.3f} | {np.max(np.abs(uo[w])):7.2f}")
        return

    rows = []
    for name, q, sy in runs:
        o = replay(p, y, sent, args.delay, sy, q, pick0)
        u, eq, sw = smc_output(p, s, t, o["xp"], o["vp"], args.delay, cfg)
        on = t >= 0
        u = np.where(on, u, 0.0)
        if q is None:
            ok = hold & np.isfinite(a["kf_innovation_mm"])
            print(f"check, baseline vs log in the window: innovation max diff "
                  f"{np.nanmax(np.abs(o['innov'][ok] - a['kf_innovation_mm'][ok])):.1e} mm, "
                  f"SMC output max diff {np.max(np.abs(u[hold] - out_logged[hold])):.2e} A")
        X51 = phasor(t, o["x"], f_ref, hold) / phasor(t, a["position_ref_mm"], f_ref, hold)
        env = envelope(t, o["x"], f_ref, hold)
        envu = envelope(t, u, f_ref, hold)
        rows.append((name,
                     abs(phasor(t, o["x"], F_MAINS, hold)), abs(phasor(t, o["v"], F_MAINS, hold)),
                     abs(phasor(t, u, F_MAINS, hold)), abs(phasor(t, eq, F_MAINS, hold)), abs(phasor(t, sw, F_MAINS, hold)),
                     abs(X51), math.degrees(np.angle(X51)), env[0], env[1], envu[0], envu[1],
                     np.nanstd(o["innov"][hold]) / np.nanmean(o["istd"][hold]),
                     abs(phasor(t, o["innov"], F_MAINS, hold)),
                     abs(phasor(t, o["pick"], F_MAINS, hold)) if q is not None else float("nan"),
                     abs(phasor(t, o["pick"], F_MAINS, idle)) if q is not None else float("nan")))

    print(f"\nwindow t = {args.hold[0]:g} .. {args.hold[1]:g} s (reference {f_ref:g} Hz); 50 Hz content of each "
          "signal, and the 51 Hz amplitude of x(k|k) over the reference")
    head = (f"{'filter':24} | {'x 50Hz':>7} {'v 50Hz':>7} | {'u 50Hz':>6} {'eq':>5} {'sw':>5} | "
            f"{'x/r @f1':>7} {'deg':>6} | {'x env min-max mm':>16} | {'u env min-max A':>15} | "
            f"{'innov ratio':>11} {'innov 50Hz':>10} | {'pickup est hold/idle':>20}")
    print(head)
    print("-" * len(head))
    for r_ in rows:
        (name, x50, v50, u50, eq50, sw50, g51, ph51, e0, e1, u0, u1, ratio, i50, pk_h, pk_i) = r_
        print(f"{name:24} | {x50:7.3f} {v50:7.1f} | {u50:6.2f} {eq50:5.2f} {sw50:5.2f} | {g51:7.3f} {ph51:6.1f} | "
              f"{e0:7.3f} - {e1:6.3f} | {u0:6.2f} - {u1:6.2f} | {ratio:11.2f} {i50:10.4f} | "
              + (f"{pk_h:9.4f} / {pk_i:.4f}" if math.isfinite(pk_h) else f"{'-':>20}"))
    print("(units: x mm, v mm/s, u eq sw A, innovation mm; laser in the window at 50 Hz: "
          f"{abs(phasor(t, y, F_MAINS, hold)):.3f} mm, logged SMC output at 50 Hz: "
          f"{abs(phasor(t, out_logged, F_MAINS, hold)):.2f} A)")


if __name__ == "__main__":
    main()
