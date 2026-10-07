"""Figures for the app. Pure functions of runs and settings, so they can be tested without a browser.

Encoding, the same in every figure, by colour only (no dotted or dashed lines):
- current run: blue = the physical truth (true position, coil current), orange = what the controller
  sees or sends (laser position, command), grey = the reference;
- pinned runs: only their physical truth, each in its own colour (runs.SERIES), never blue or orange.
Every quantity gets its own stacked panel with one y-axis.

Long traces are thinned for display by min/max per pixel bucket over the visible window, so peaks
survive; the time-trace callbacks re-thin when the user zooms.
"""

import math

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e6e5e1"
REF_COLOR = "#8f8e88"      # neutral grey: references and guide lines
TRUE_COLOR = "#2a78d6"     # palette slot 1, blue: physical truth of the current run
SEEN_COLOR = "#eb6834"     # palette slot 2, orange: what the controller sees or sends
SIGNAL_COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948")
LIMIT_COLOR = "#d03b3b"     # status "critical": only for hard limits, always with a text label
MAX_POINTS = 1500           # buckets per trace over the visible window (SVG traces: works without WebGL)


def decimate(t, y, t0=None, t1=None, n=MAX_POINTS):
    """Points of (t, y) inside [t0, t1], thinned to about 2 n points by keeping each bucket's min and max."""
    t, y = np.asarray(t), np.asarray(y)
    if t0 is not None or t1 is not None:
        m = (t >= (t0 if t0 is not None else -np.inf)) & (t <= (t1 if t1 is not None else np.inf))
        idx = np.flatnonzero(m)
        if len(idx):                                   # one sample either side so lines reach the edges
            idx = np.arange(max(idx[0] - 1, 0), min(idx[-1] + 2, len(t)))
        t, y = t[idx], y[idx]
    if len(t) <= 2 * n:
        return t, y
    edges = np.linspace(0, len(t), n + 1).astype(int)
    keep = []
    for a, b in zip(edges[:-1], edges[1:]):
        seg = y[a:b]
        if not len(seg):
            continue
        i_min, i_max = a + int(np.nanargmin(seg)), a + int(np.nanargmax(seg))
        keep.extend(sorted({i_min, i_max}))
    keep = np.asarray(keep)
    return t[keep], y[keep]


def _layout(fig, height, title=None):
    fig.update_layout(
        height=height, margin=dict(l=70, r=20, t=30, b=40), title=title,
        paper_bgcolor=SURFACE, plot_bgcolor=SURFACE, font=dict(color=INK, size=12),
        hovermode="x unified")
    _legends_in_panels(fig)
    fig.update_xaxes(gridcolor=GRID, zerolinecolor=GRID, linecolor=GRID, tickfont=dict(color=INK_2))
    fig.update_yaxes(gridcolor=GRID, zerolinecolor=GRID, linecolor=GRID, tickfont=dict(color=INK_2))
    return fig


def _legends_in_panels(fig):
    """Give every stacked panel its own legend, inside the panel's top-right corner, listing only the
    traces drawn in that panel."""
    rows = set()
    for tr in fig.data:
        ya = getattr(tr, "yaxis", None) or "y"
        row = int(ya[1:]) if len(ya) > 1 else 1
        rows.add(row)
        tr.legend = "legend" if row == 1 else f"legend{row}"
        if tr.name:
            tr.showlegend = True
    for row in rows:
        axis = fig.layout["yaxis" if row == 1 else f"yaxis{row}"]
        top = axis.domain[1] if axis.domain else 1.0
        fig.layout["legend" if row == 1 else f"legend{row}"] = dict(
            x=0.995, xanchor="right", y=top, yanchor="top", orientation="v", traceorder="normal",
            bgcolor="rgba(252,252,251,0.85)", bordercolor=GRID, borderwidth=1, font=dict(color=INK, size=11))


def empty(message, height=300):
    fig = go.Figure()
    fig.add_annotation(text=message, showarrow=False, font=dict(color=INK_2, size=14), xref="paper", yref="paper",
                       x=0.5, y=0.5)
    fig.update_xaxes(visible=False)
    fig.update_yaxes(visible=False)
    return _layout(fig, height)


# ---------------------------------------------------------------- time traces

PANELS = ("position", "error", "current", "accel", "signals")


def time_figure(runs, x_range=None, panels=("position", "error", "current")):
    """Stacked time traces, shared time axis. runs: list of app.runs.Run (current first)."""
    if not runs:
        return empty("No run yet")
    unit = runs[0].result.ref_unit
    panels = [p for p in PANELS if p in panels and not (p == "error" and unit != "mm")]
    titles = {"position": "position (mm)" if unit == "mm" else "position (mm), reference in A shown under current",
              "error": "tracking error, reference - position (mm)", "current": "current (A)",
              "accel": "accelerometer AI2 (V)", "signals": "controller signals"}
    fig = make_subplots(rows=len(panels), cols=1, shared_xaxes=True, vertical_spacing=0.05,
                        subplot_titles=[titles[p] for p in panels])
    row = {p: i + 1 for i, p in enumerate(panels)}
    t0, t1 = (x_range or (None, None))

    def add(run, panel, y, name, color=None, **line):
        if panel not in row:
            return
        tt, yy = decimate(run.result.t, y, t0, t1)
        fig.add_trace(go.Scatter(x=tt, y=yy, mode="lines", name=name, legendgroup=run.id,
                                   line=dict(color=color or run.color, **line)),
                      row=row[panel], col=1)

    for i, run in enumerate(runs):
        r = run.result
        if i == 0:
            # current run: reference underneath, then what the controller sees, then the truth on top
            add(run, "position" if r.ref_unit == "mm" else "current", r["ref"], "reference", REF_COLOR, width=2)
            add(run, "position", r["position_mm"], "laser position_mm", SEEN_COLOR, width=1.2)
            add(run, "position", r["x_true_mm"], "true position", TRUE_COLOR, width=2)
            if r.ref_unit == "mm":
                add(run, "error", r["ref"] - r["position_mm"], "error, measured", SEEN_COLOR, width=1.2)
                add(run, "error", r["ref"] - r["x_true_mm"], "error, true", TRUE_COLOR, width=2)
            add(run, "current", r["u_cmd_A"], "command", SEEN_COLOR, width=1.2)
            add(run, "current", r["i_true_A"], "coil current", TRUE_COLOR, width=2)
            add(run, "accel", r["accel_V"], "accel_V", SEEN_COLOR, width=1.2)
            for k, name in enumerate(r.signal_names):
                add(run, "signals", r[f"ctrl_{name}"], name, SIGNAL_COLORS[k % len(SIGNAL_COLORS)], width=1.5)
            continue
        # pinned run: physical truth only, in the run's own colour
        tag = f" [{run.label}]"
        if r.ref_unit != runs[0].result.ref_unit or not np.array_equal(r["ref"], runs[0].result["ref"]):
            add(run, "position" if r.ref_unit == "mm" else "current", r["ref"], f"reference{tag}", width=1)
        add(run, "position", r["x_true_mm"], f"true position{tag}", width=2)
        if r.ref_unit == "mm":
            add(run, "error", r["ref"] - r["x_true_mm"], f"error, true{tag}", width=2)
        add(run, "current", r["i_true_A"], f"coil current{tag}", width=2)
        add(run, "accel", r["accel_V"], f"accel_V{tag}", width=1.2)
        for name in r.signal_names:
            add(run, "signals", r[f"ctrl_{name}"], f"{name}{tag}", width=1.5)
    fig.update_xaxes(title_text="time (s)", row=len(panels), col=1)
    if x_range:
        fig.update_xaxes(range=list(x_range))
    for a in fig.layout.annotations:
        a.font = dict(color=INK_2, size=12)
        a.x, a.xanchor = 0, "left"
    t_first = min(float(run.result.t[0]) for run in runs if len(run.result.t))
    if t_first < 0:
        # idle window before t = 0: drive enabled, 0 A, controller off (firmware BIAS_IDLE_S)
        for k in range(1, len(panels) + 1):
            fig.add_vrect(x0=t_first, x1=0, fillcolor=REF_COLOR, opacity=0.12, line_width=0, layer="below",
                          row=k, col=1)
        fig.add_annotation(x=t_first, xanchor="left", y=0, yanchor="bottom", xref="x", yref="y domain",
                           text="idle, 0 A", showarrow=False, font=dict(color=INK_2, size=11), row=1, col=1)
    _layout(fig, 220 * len(panels) + 40)
    fig.update_layout(uirevision="time")            # keep zoom and legend state across re-runs
    return fig


def x_range_from_relayout(relayout, current=None):
    """Visible time window from a Plotly relayoutData event: (t0, t1), None to reset, or `current` if the
    event says nothing about the x axis (e.g. a y-only zoom or a resize)."""
    if not relayout:
        return current
    for k, v in relayout.items():
        if k.startswith("xaxis") and k.endswith(".autorange") and v:
            return None
    for k in relayout:
        if k.startswith("xaxis") and k.endswith(".range[0]"):
            ax = k[:-len(".range[0]")]
            return float(relayout[k]), float(relayout[f"{ax}.range[1]"])
        if k.startswith("xaxis") and k.endswith(".range"):
            return float(relayout[k][0]), float(relayout[k][1])
    return current


# ---------------------------------------------------------------- tracking per frequency

def tracking_figure(series, drive_kp_A=None):
    """series: list of (run, rows) with rows from analysis.tracking_response."""
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.07,
                        subplot_titles=["amplitude, position / reference", "phase, position - reference (deg)",
                                        "peak command current (A)"])
    mass_line_done = False
    for i, (run, rows) in enumerate(series):
        ok = [q for q in rows if q.get("status") == "ok"]
        f = [q["f_Hz"] for q in ok]
        # current run: true (blue) and laser (orange); pinned runs: true only, in their own colour
        color = TRUE_COLOR if i == 0 else run.color
        curves = [("true", color, f"true position, {run.label}")]
        if i == 0:
            curves.append(("measured", SEEN_COLOR, f"laser, {run.label}"))
        for key, c, name in curves:
            line = dict(color=c, width=2)
            fig.add_trace(go.Scatter(x=f, y=[q[f"gain_{key}"] for q in ok], mode="lines+markers", name=name,
                                     legendgroup=name, line=line, marker=dict(size=8)), row=1, col=1)
            fig.add_trace(go.Scatter(x=f, y=[q[f"phase_{key}_deg"] for q in ok], mode="lines+markers",
                                     showlegend=False, legendgroup=name, line=line, marker=dict(size=8)), row=2, col=1)
        fig.add_trace(go.Scatter(x=f, y=[q["peak_current_A"] for q in ok], mode="lines+markers",
                                 name=f"peak current, {run.label}", showlegend=False,
                                 legendgroup=f"true position, {run.label}",
                                 line=dict(color=color, width=2), marker=dict(size=8)), row=3, col=1)
        not_conv = [q for q in ok if not q.get("converged", True)]
        if not_conv:
            fig.add_trace(go.Scatter(x=[q["f_Hz"] for q in not_conv], y=[q["gain_true"] for q in not_conv],
                                     mode="markers", name=f"not settled, {run.label}",
                                     marker=dict(size=16, symbol="circle-open", color=color, line=dict(width=2))),
                          row=1, col=1)
        for q in rows:
            if q.get("status") != "ok":
                fig.add_annotation(x=q["f_Hz"], y=0, text=q["status"], showarrow=False, row=1, col=1,
                                   font=dict(color=INK_2))
        if not mass_line_done and rows:
            mass_line_done = True
            fig.add_trace(go.Scatter(x=[q["f_Hz"] for q in rows], y=[q["mass_line_current_A"] for q in rows],
                                     mode="lines", name="current the mass alone needs",
                                     line=dict(color=REF_COLOR, width=2)), row=3, col=1)
    fig.add_hline(y=1.0, line=dict(color=REF_COLOR, width=1), row=1, col=1)
    fig.add_hline(y=0.0, line=dict(color=REF_COLOR, width=1), row=2, col=1)
    if drive_kp_A:
        fig.add_hline(y=drive_kp_A, line=dict(color=LIMIT_COLOR, width=1), row=3, col=1,
                      annotation_text=f"drive peak {drive_kp_A:g} A", annotation_font_color=INK_2)
    fig.update_xaxes(title_text="frequency (Hz)", row=3, col=1)
    for a in fig.layout.annotations:
        if a.text and a.text.startswith(("amplitude", "phase", "peak command")):
            a.font, a.x, a.xanchor = dict(color=INK_2, size=12), 0, "left"
    _layout(fig, 760)
    fig.update_layout(hovermode="closest")
    return fig


# ---------------------------------------------------------------- loop gain

def stability_figure(series):
    """series: list of (run, LoopGain)."""
    fig = make_subplots(rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.07,
                        subplot_titles=["|L|, loop gain at the controller output", "phase of L (deg)",
                                        "|S|, sensitivity (peak = 1 / modulus margin)"])
    for i, (run, lg) in enumerate(series):
        ok = np.isfinite(lg.L)
        if not ok.any():
            continue
        f = lg.f_hz[ok]
        color = TRUE_COLOR if i == 0 else run.color
        common = dict(legendgroup=run.id, line=dict(color=color, width=2), marker=dict(size=6))
        fig.add_trace(go.Scatter(x=f, y=np.abs(lg.L[ok]), mode="lines+markers", name=run.label, **common), row=1, col=1)
        fig.add_trace(go.Scatter(x=f, y=np.degrees(np.unwrap(np.angle(lg.L[ok]))), mode="lines+markers",
                                 showlegend=False, **common), row=2, col=1)
        fig.add_trace(go.Scatter(x=f, y=np.abs(lg.S[ok]), mode="lines+markers", showlegend=False, **common),
                      row=3, col=1)
        for c in lg.margins.get("crossovers", []):
            fig.add_vline(x=c["f_Hz"], line=dict(color=color, width=1), opacity=0.5)
    fig.add_hline(y=1.0, line=dict(color=REF_COLOR, width=1), row=1, col=1)
    fig.add_hline(y=-180.0, line=dict(color=REF_COLOR, width=1), row=2, col=1)
    fig.add_hline(y=1.0, line=dict(color=REF_COLOR, width=1), row=3, col=1)
    fig.update_xaxes(type="log", title_text="frequency (Hz)", row=3, col=1)
    fig.update_xaxes(type="log")
    fig.update_yaxes(type="log", row=1, col=1)
    fig.update_yaxes(type="log", row=3, col=1)
    for a in fig.layout.annotations:
        a.font, a.x, a.xanchor = dict(color=INK_2, size=12), 0, "left"
    _layout(fig, 760)
    fig.update_layout(hovermode="closest")
    return fig


def margins_rows(series):
    """One table row per crossover per run (or one row saying why there is none)."""
    rows = []
    for run, lg in series:
        m = lg.margins
        base = {"run": run.label, "status": lg.status,
                "modulus margin": _r(m.get("modulus_margin")), "max |S| at (Hz)": _r(m.get("max_S_f_Hz"))}
        gms = "; ".join(f"{g['gain_margin_dB']:.1f} dB @ {g['f_Hz']:.1f} Hz" for g in m.get("phase_crossovers", []))
        if not m.get("crossovers"):
            rows.append({**base, "crossover (Hz)": "-", "phase margin (deg)": "-", "delay margin (ms)": "-",
                         "gain margins": gms or "-"})
        for c in m.get("crossovers", []):
            rows.append({**base, "crossover (Hz)": _r(c["f_Hz"]), "phase margin (deg)": _r(c["phase_margin_deg"]),
                         "delay margin (ms)": _r(c["delay_margin_ms"]), "gain margins": gms or "-"})
    return rows


def _r(v, n=3):
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "-"
    return float(f"{v:.{n}g}")


# ---------------------------------------------------------------- rig logs

def log_figure(log_run, sim=None, x_range=None, sim_color=TRUE_COLOR, rig_color=SEEN_COLOR):
    """A rig log (vca_log.load_log) with an optional simulated replay on top."""
    df = log_run.run if hasattr(log_run, "run") else log_run
    has_ref = "position_ref_mm" in df.columns
    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.06,
                        subplot_titles=["position_mm", "current (A)"])
    t = df["time_s"].to_numpy()
    t0, t1 = (x_range or (None, None))

    def add(tt, y, name, color, row, **line):
        x_, y_ = decimate(tt, y, t0, t1)
        fig.add_trace(go.Scatter(x=x_, y=y_, mode="lines", name=name, line=dict(color=color, **line)), row=row, col=1)

    add(t, df["position_mm"].to_numpy(), "rig position_mm", rig_color, 1, width=1)
    if has_ref:
        add(t, df["position_ref_mm"].to_numpy(), "rig reference", REF_COLOR, 1, width=2)
    add(t, df["actual_current_A"].to_numpy(), "rig actual_current_A", rig_color, 2, width=1)
    if sim is not None:
        add(sim.t, sim["position_mm"], "simulated position_mm", sim_color, 1, width=1)
        add(sim.t, sim["actual_current_A"], "simulated actual_current_A", sim_color, 2, width=1)
    fig.update_xaxes(title_text="time (s)", row=2, col=1)
    if x_range:
        fig.update_xaxes(range=list(x_range))
    for a in fig.layout.annotations:
        a.font, a.x, a.xanchor = dict(color=INK_2, size=12), 0, "left"
    _layout(fig, 640)
    fig.update_layout(uirevision="log")
    return fig
