"""Presentation figures for the 16 Oct 2026 update. Run from the repository root with the project venv.

Figures that need the 8 Oct bench logs (#1, #5, #6, #7) get a placeholder. The others are rebuilt from
the committed notebooks and scripts: Joshua's vca_greybox_fit.ipynb (4deab16) for #3, #4, #10, Tom's
uncommitted vca_greybox_fit_friction.ipynb for #9, scripts/ekf_mains_replay.py (origin/sys_id) for #8
and vca_sim (origin/simulation_tool) for #2.
"""
import json
import math
import sys
import traceback
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRATCH = Path(__file__).parent
REPO = Path.cwd()
OUT = REPO / "presentation_figures"
OUT.mkdir(exist_ok=True)
WT_SYSID, WT_SIM = SCRATCH / "wt_sysid", SCRATCH / "wt_sim"

# dataviz reference palette (light mode): blue, orange, aqua are the categorical slots; ink for text and reference lines
BLUE, ORANGE, AQUA, INK, INK2, GRID = "#2a78d6", "#eb6834", "#1baf7a", "#0b0b0b", "#52514e", "#d9d8d3"
BAND = "#b0afa9"


def style():
    "Large labels, transparent background, recessive grid. Called before every figure (the notebooks reset rcParams)."
    plt.rcdefaults()
    plt.rcParams.update({
        "font.size": 18, "axes.labelsize": 22, "axes.titlesize": 22, "xtick.labelsize": 18, "ytick.labelsize": 18,
        "legend.fontsize": 17, "axes.edgecolor": INK2, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
        "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
        "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False, "lines.linewidth": 2.0,
        "figure.facecolor": "none", "axes.facecolor": "none", "savefig.facecolor": "none", "font.family": "DejaVu Sans",
    })


def save(fig, name):
    path = OUT / f"{name}.png"
    fig.savefig(path, dpi=300, bbox_inches="tight", transparent=True)
    plt.close(fig)
    print(f"saved {path.name}")


def code_cells(nb_path):
    nb = json.loads(Path(nb_path).read_text(encoding="utf-8"))
    return ["".join(c["source"]) for c in nb["cells"] if c["cell_type"] == "code"]


def find_cell(cells, needle):
    hits = [c for c in cells if needle in c]
    assert len(hits) == 1, f"{needle!r}: {len(hits)} cells"
    return hits[0]


def cut(src, marker):
    "Source up to (not including) the first line that contains marker."
    lines = src.splitlines()
    idx = next(i for i, l in enumerate(lines) if marker in l)
    return "\n".join(lines[:idx])


def keep_through(src, marker):
    "Source up to and including the first line that contains marker."
    lines = src.splitlines()
    idx = next(i for i, l in enumerate(lines) if marker in l)
    return "\n".join(lines[:idx + 1])


def run(src, ns):
    exec(compile(src, "<cell>", "exec"), ns)


def placeholder(name, title, needs, figsize=(10, 8)):
    style()
    fig, ax = plt.subplots(figsize=figsize)
    ax.axis("off")
    ax.text(0.5, 0.62, title, ha="center", va="center", fontsize=26, color=INK, wrap=True)
    ax.text(0.5, 0.42, "placeholder: needs " + needs, ha="center", va="center", fontsize=18, color=INK2, wrap=True)
    save(fig, name)


def thin(t, y, n=12000):
    "Keep the plot files small: min/max per bin so peaks survive."
    if len(t) <= n:
        return t, y
    b = -(-len(t) // n)                                     # points per bin; at most b - 1 points dropped at the end
    m = len(t) // b * b
    tt, yy = t[:m].reshape(-1, b), y[:m].reshape(-1, b)
    t2 = np.repeat(tt.mean(1), 2)
    y2 = np.column_stack([yy.min(1), yy.max(1)]).ravel()
    return t2, y2


# ---------------------------------------------------------------- Joshua's notebook: #3, #4, #10

def josh_namespace():
    cells = code_cells(SCRATCH / "josh.ipynb")
    ns = {}
    plt.show = lambda *a, **k: None
    run(find_cell(cells, "FS_LOG, DECIM"), ns)
    data_cell = find_cell(cells, "DATA = {exp:")
    run(keep_through(data_cell, 'for exp in ("chirp", "chirpsched")}'), ns)   # DATA = {...} only, no overview plots
    run(find_cell(cells, '"C_R": -1 / (G00 - x)**3'), ns)   # Method C fit -> p_C, magnets
    kf = find_cell(cells, "def load_raw(path)")
    run(cut(kf, "# single-step prediction error"), ns)      # RAW, SIG_A, SIG_Y, ekf()
    smc = find_cell(cells, "def smc(eta)")
    run(cut(smc, "ZOOMS = ["), ns)                          # t, r, rd, xs, ss, us, hold, A_SMC, PHI, ETA
    gap = find_cell(cells, "F_FF, POLY_DEG")
    run(cut(gap, "BLUE, ORANGE"), ns)                       # G0S, E, g, E_fit, g_min, g_res, f_n
    return ns


def fig3_ekf(ns):
    style()
    ekf, magnets, p_C, SIG_A, SIG_Y, TS_RAW, RAW = (ns[k] for k in ("ekf", "magnets", "p_C", "SIG_A", "SIG_Y", "TS_RAW", "RAW"))
    runs = [("scheduled chirp, 10 to 55 Hz (test run)", RAW["chirpsched", "test"]),
            ("plain chirp, 1 to 55 Hz, 6 A (test run)", RAW["chirp", "test"])]
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for ax, (label, d) in zip(axes, runs):
        xf, vf, xp, sp = ekf(magnets, p_C, d, SIG_A, SIG_Y, TS_RAW)
        m = d["t"] >= 1.0
        e = (d["x"] - xp)[m] * 1e3
        rms = math.sqrt(np.mean(e ** 2))
        tt, ee = thin(d["t"][m], e)
        ax.plot(tt, ee, color=BLUE, lw=0.6, label="one-step prediction error")
        ax.fill_between(d["t"][m][::20], -2e3 * sp[m][::20], 2e3 * sp[m][::20], color=INK2, alpha=0.45, lw=0, label="±2 std the filter expects", zorder=3)
        ax.set_title(label, loc="left", fontsize=20, color=INK2)
        ax.set_ylabel("error (mm)")
        ax.text(0.99, 0.95, f"RMS {rms:.2f} mm", transform=ax.transAxes, ha="right", va="top", fontsize=20, color=INK,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.7, pad=4))
        print(f"  #3 {label}: RMS {rms:.4f} mm, ratio {np.std(e) / (np.mean(sp[m]) * 1e3):.2f}")
    axes[1].legend(loc="lower right")
    axes[1].set_xlabel("time (s)")
    fig.suptitle("Kalman filter on model C: how far each new laser sample is from the prediction", fontsize=20, x=0.02, ha="left")
    fig.tight_layout()
    save(fig, "03_ekf_prediction_error")


def fig4_smc(ns):
    style()
    t, r, xs, us, hold = (ns[k] for k in ("t", "r", "xs", "us", "hold"))
    T_RAMP, T_HOLD, F1 = ns["T_RAMP"], ns["T_HOLD"], ns["F1"]
    err = (xs - r) * 1e3
    rms_u = math.sqrt(np.mean(us[hold] ** 2))
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), gridspec_kw=dict(height_ratios=[1.2, 1, 1]))
    # top: 0.1 s zoom in the middle of the 50 Hz hold, reference and position on top of each other
    t0 = T_RAMP + T_HOLD / 2
    z = (t >= t0) & (t < t0 + 0.1)
    ax = axes[0]
    ax.plot(t[z], r[z] * 1e3, color=INK, lw=3.5, label="reference")
    ax.plot(t[z], xs[z] * 1e3, color=ORANGE, lw=1.8, label="position (model)")
    ax.set_ylabel("position (mm)")
    ax.set_title(f"zoom in the {F1:.0f} Hz hold: 0.1 s", loc="left", fontsize=20, color=INK2, pad=36)
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2, borderaxespad=0.0)
    # middle and bottom: whole run
    for ax in axes[1:]:
        ax.axvspan(T_RAMP, T_RAMP + T_HOLD, color=BAND, alpha=0.25, lw=0)
    tt, ee = thin(t, err)
    axes[1].plot(tt, ee, color=BLUE, lw=0.8)
    axes[1].set_ylabel("error (mm)")
    axes[1].set_title("whole run: chirp 1 to 50 Hz over 30 s, then 10 s at 50 Hz (shaded)", loc="left", fontsize=20, color=INK2)
    axes[1].text(0.01, 0.9, f"largest error {np.abs(err).max():.3f} mm", transform=axes[1].transAxes, va="top", fontsize=18)
    tt, uu = thin(t, us)
    axes[2].plot(tt, uu, color=AQUA, lw=0.8)
    axes[2].set_ylabel("current (A)")
    axes[2].set_xlabel("time (s)")
    axes[2].text(0.01, 0.9, f"{rms_u:.1f} A rms in the hold", transform=axes[2].transAxes, va="top", fontsize=18)
    fig.suptitle("Sliding-mode controller on the model C plant (simulation, no noise)", fontsize=20, x=0.02, ha="left")
    fig.tight_layout()
    save(fig, "04_smc_simulation")
    print(f"  #4 max err {np.abs(err).max():.4f} mm, rms u hold {rms_u:.2f} A")


def fig10_gap(ns):
    style()
    G0S, E, g, E_fit, g_min, g_res = (ns[k] for k in ("G0S", "E", "g", "E_fit", "g_min", "g_res"))
    print(f"  #10 effort at 5, 7.65, 12, 22.3 mm: " + ", ".join(f"{np.interp(x, G0S * 1e3, E):.1f}" for x in (5, 7.65, 12, 22.3))
          + f"; g_min {g_min * 1e3:.2f}, g_res {g_res * 1e3:.2f}")
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.scatter(G0S * 1e3, E, s=55, color=BLUE, edgecolor="white", linewidth=0.8, zorder=3, label="simulated effort")
    ax.plot(g * 1e3, E_fit, color=ORANGE, lw=2.5, label="polynomial fit, 5 to 12 mm", zorder=2)
    ymax = ax.get_ylim()[1]
    for xg, col, txt, dy in ((g_min, ORANGE, f"least effort\n{g_min * 1e3:.1f} mm", 0.97),
                             (g_res, AQUA, f"resonance at 51 Hz\n{g_res * 1e3:.1f} mm", 0.80),
                             (22.3e-3, INK2, "current gap\n22.3 mm", 0.97)):
        ax.axvline(xg * 1e3, color=col, ls=":", lw=2.5, zorder=1)
        ax.text(xg * 1e3 + (0.3 if xg < 0.02 else -0.3), ymax * dy, txt, color=col, fontsize=18, va="top",
                ha="left" if xg < 0.02 else "right")
    ax.set_xlabel("magnet gap $g_0$ (mm)")
    ax.set_ylabel("total current effort  ∫|u| dt  (A·s)")
    ax.set_title("Current effort to hold 2 mm at 51 Hz, against magnet gap (model C)", fontsize=20, loc="left")
    ax.legend(loc="center right")
    fig.tight_layout()
    save(fig, "10_gap_sweep")


# ---------------------------------------------------------------- Tom's friction notebook: #9

def fig9_model_error():
    style()
    cells = code_cells(SCRATCH / "friction.ipynb")
    ns = {}
    plt.show = lambda *a, **k: None
    run(find_cell(cells, "FS_LOG, DECIM"), ns)
    run(cut(find_cell(cells, 'USE = [("Stage 1"'), "USE = ["), ns)
    run(find_cell(cells, "SPRING = False"), ns)              # Stage 1 (p_hf)
    run(find_cell(cells, 'cols = {"Gamma1": x * i, "C_R"'), ns)   # Method C (this notebook's version)
    res = find_cell(cells, "F_EDGES = np.arange")
    run(cut(res, "# figure:"), ns)                           # BANDS, fits
    BANDS, F_EDGES = ns["BANDS"], ns["F_EDGES"]
    style()
    fig, ax = plt.subplots(figsize=(10, 8))
    centres = BANDS["train"][:, 0]
    w = 1.8
    ax.bar(centres - w / 2, BANDS["train"][:, 1], width=w, color=BLUE, label="fit data (train run)")
    ax.bar(centres + w / 2, BANDS["test"][:, 1], width=w, color=ORANGE, label="unseen data (test run)")
    for c0, s in zip(centres, BANDS["test"][:, 1]):
        ax.text(c0 + w / 2, s, f"{s:.0f}", ha="center", va="bottom", fontsize=14, color=INK2)
    ax.set_xticks(centres)
    ax.set_xticklabels([f"{lo:.0f}–{hi:.0f}" for lo, hi in zip(F_EDGES[:-1], F_EDGES[1:])], fontsize=16)
    ax.set_xlabel("chirp frequency band (Hz)")
    ax.set_ylabel("unexplained force, std (N)")
    ax.set_title("Force that model C leaves unexplained, per frequency band", fontsize=20, loc="left")
    ax.legend(loc="upper left")
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    save(fig, "09_model_error_per_band")
    print("  #9 bands:", np.round(BANDS["train"][:, 1]).astype(int), np.round(BANDS["test"][:, 1]).astype(int))
    print(f"  #9 c_eff train: {np.round(BANDS['train'][:, 2]).astype(int)}")


# ---------------------------------------------------------------- mains replay script: #8

def fig8_pickup():
    style()
    sys.path.insert(0, str(WT_SYSID / "scripts"))
    import ekf_mains_replay as mr
    p = mr.model()
    s = {k: float(v) for k, v in mr.defines("closed_loop_settings.h").items()}
    # pickup and noise from the idle window of a 2 Oct log (the 8 Oct logs are not on this machine)
    log = next((REPO / "gcsc_data").glob("voice_coil_log_chirpsched_*102726.csv"))
    a = np.genfromtxt(log, delimiter=",", names=True, autostrip=True)
    t, y = a["time_s"], a["position_mm"]
    idle = (t < 0) & (t > t[0] + 0.5)
    P50, P150 = mr.phasor(t, y, 50.0, idle), mr.phasor(t, y, 150.0, idle)
    yi = y[idle] - np.mean(y[idle]) - np.real(P50 * np.exp(2j * math.pi * 50.0 * t[idle]))
    sig_white = float(np.std(yi))
    noise = {"a50": abs(P50), "a150": abs(P150), "white": math.sqrt(max(sig_white ** 2 - abs(P150) ** 2 / 2, 1e-6))}
    print(f"  #8 idle window of {log.name}: 50 Hz {noise['a50']:.4f} mm, 150 Hz {noise['a150']:.4f} mm, white {noise['white']:.4f} mm")
    pick0 = s["POS_KF_MAINS_PICKUP_MM"]
    B = s["POS_KF_MAINS_BW_HZ"]
    q = 2 * math.pi * B * mr.TS * p["POS_KF_SIG_Y_MM"]
    runs = [("without pickup state", None, math.sqrt(p["POS_KF_SIG_Y_MM"] ** 2 + pick0 ** 2 / 2), BLUE),
            ("with pickup state in the filter", q, p["POS_KF_SIG_Y_MM"], ORANGE)]
    f_ref, amp, delay = 51.0, 0.6, int(s["POS_KF_DELAY_CYCLES"])
    fig, ax = plt.subplots(figsize=(12, 6))
    fgrid = np.arange(47.0, 55.01, 0.02)
    for name, qq, sy, col in runs:
        ts_, xs, ys, rs, uo = mr.simulate(p, s, f_ref, amp, delay, mr.Ekf(p, sy, qq, pick0), noise, 1.0, 50.0)
        w = ts_ >= 4.0
        tw, uw = ts_[w], uo[w] - uo[w].mean()
        win = np.hanning(len(uw))
        amp_f = np.array([2 * abs(np.sum(win * uw * np.exp(-2j * math.pi * f * tw))) / win.sum() for f in fgrid])
        u50 = abs(mr.phasor(ts_, uo, 50.0, w))
        x50 = abs(mr.phasor(ts_, xs, 50.0, w))
        print(f"  #8 {name}: u 50 Hz {u50:.2f} A, true x 50 Hz {x50:.3f} mm, max|u| {np.max(np.abs(uo[w])):.1f} A")
        ax.plot(fgrid, amp_f, color=col, lw=2.2, label=name)
        ax.annotate(f"{u50:.2f} A at 50 Hz", (50.0, u50), xytext=(47.3, 3.4 if qq is None else 1.4),
                    fontsize=18, color=col, arrowprops=dict(arrowstyle="-", color=col, lw=1.2))
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("commanded current (A)")
    ax.set_title(f"Spectrum of the controller current, {amp} mm reference at {f_ref:.0f} Hz (closed-loop simulation)", fontsize=20, loc="left")
    ax.legend(loc="upper right")
    ax.set_xlim(47, 55)
    fig.tight_layout()
    save(fig, "08_pickup_state_replay")


# ---------------------------------------------------------------- simulator: #2

def fig2_simulator():
    style()
    sys.path.insert(0, str(WT_SIM))
    import pandas as pd
    from vca_sim import PRESETS, simulate, load_controller
    from vca_sim.replay import scenario_from_log
    log = next((REPO / "gcsc_data").glob("voice_coil_log_chirpsched_*103233.csv"))   # the test run, not used by the fit
    ol = load_controller(WT_SIM / "controllers" / "open_loop_current.py")
    sim = simulate(scenario_from_log(log, ol, PRESETS["greybox_C"]))
    print(f"  #2 sim status {sim.status} {sim.message!r}")
    df = pd.read_csv(log, skipinitialspace=True)
    t, x, i = df["time_s"].to_numpy(), df["position_mm"].to_numpy(), df["actual_current_A"].to_numpy()
    ts, xs, is_ = sim.t, sim["position_mm"], sim["actual_current_A"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), gridspec_kw=dict(height_ratios=[1.3, 1, 1]))
    tt, xx = thin(t, x); tts, xxs = thin(ts, xs)
    axes[0].plot(tt, xx, color=BLUE, lw=0.6, label="rig (laser)")
    axes[0].plot(tts, xxs, color=ORANGE, lw=0.6, alpha=0.8, label="simulator")
    axes[0].set_ylabel("position (mm)")
    axes[0].set_title("whole run: scheduled chirp 10 to 55 Hz, 2 Oct", loc="left", fontsize=20, color=INK2)
    axes[0].legend(loc="lower center", ncol=2, bbox_to_anchor=(0.5, 1.02), borderaxespad=0.0, handlelength=1.5)
    axes[0].set_title("whole run: scheduled chirp 10 to 55 Hz, 2 Oct", loc="left", fontsize=20, color=INK2, pad=40)
    t0 = 45.0
    z, zs = (t >= t0) & (t < t0 + 0.25), (ts >= t0) & (ts < t0 + 0.25)
    axes[1].plot(t[z], x[z], color=BLUE, lw=2.0, label="rig (laser)")
    axes[1].plot(ts[zs], xs[zs], color=ORANGE, lw=2.0, label="simulator")
    axes[1].set_ylabel("position (mm)")
    axes[1].set_title(f"zoom: 0.25 s at t = {t0:.0f} s", loc="left", fontsize=20, color=INK2)
    axes[2].plot(t[z], i[z], color=BLUE, lw=2.0)
    axes[2].plot(ts[zs], is_[zs], color=ORANGE, lw=2.0)
    axes[2].set_ylabel("current (A)")
    axes[2].set_xlabel("time (s)")
    fig.suptitle("Simulator replay of a rig experiment (model C, calibrated drive and laser)", fontsize=20, x=0.02, ha="left")
    fig.tight_layout(h_pad=2.0)
    save(fig, "02_simulator_vs_rig")
    # how good is the match in the zoomed window
    xi = np.interp(t[z], ts, xs)
    print(f"  #2 zoom window rms difference {np.sqrt(np.mean((xi - x[z]) ** 2)):.3f} mm, rig amplitude {np.std(x[z]) * math.sqrt(2):.2f} mm")


if __name__ == "__main__":
    only = set(sys.argv[1:])
    jobs = {
        "1": lambda: placeholder("01_pi_loop_tracking", "PI loop, 2 Oct: tracking gain against frequency",
                                 "the 2 Oct position-chirp log (1 to 10 Hz, 3 mm, PI). Not on this machine."),
        "5": lambda: placeholder("05_bench_51hz_chirphold", "Bench, 8 Oct: 51 Hz chirp-and-hold with the SMC",
                                 "the 8 Oct bench log. Not on this machine."),
        "6": lambda: placeholder("06_delay_scan", "Delay scan: EKF prediction error against assumed current delay",
                                 "the 8 Oct 1 to 40 Hz SMC chirp log for scripts/ekf_delay_scan.py. Not on this machine."),
        "7": lambda: placeholder("07_bench_beat", "Bench, 8 Oct: 51 Hz hold, amplitude beating 0.65 to 1.05 mm",
                                 "the 8 Oct 51 Hz chirp-and-hold log. Not on this machine.", figsize=(12, 6)),
        "9": fig9_model_error,
        "8": fig8_pickup,
        "2": fig2_simulator,
    }
    if not only or only & {"3", "4", "10"}:
        try:
            ns = josh_namespace()
            for key, fn in (("3", fig3_ekf), ("4", fig4_smc), ("10", fig10_gap)):
                if not only or key in only:
                    fn(ns)
        except Exception:
            traceback.print_exc()
    for key, fn in jobs.items():
        if only and key not in only:
            continue
        try:
            fn()
        except Exception:
            print(f"figure {key} failed:")
            traceback.print_exc()
