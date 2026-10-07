"""The Dash app: create_app() builds it, `python -m vca_sim.app` serves it on http://127.0.0.1:8050.

Callbacks are thin: each reads the inputs, calls a plain function of AppState (testable without a
browser), and returns what it gives back. The flow is

    controller / reference / preset / drive / sensor selection -> render their fields
    any field change (or the Run button) -> simulate -> run-version + 1 -> figures redraw
    Pin / Remove -> pinned list -> figures redraw with the ticked pinned runs
    Compute buttons -> tracking per frequency / loop gain for the current and ticked pinned runs
"""

import glob
import json
import tempfile
import traceback
from pathlib import Path

from dash import Dash, html, dcc, Input, Output, State, ALL, ctx, no_update

from .. import analysis as AN
from ..controller import discover, load_controller
from ..loop import simulate
from ..plant import PRESETS, STRUCTURES
from ..replay import scenario_from_log, write_rig_csv, reference_from_log_name
from . import forms, layout, plots
from .runs import RunStore, CURRENT_COLOR

FIELDS = {"type": forms.FIELD, "group": ALL, "name": ALL, "kind": ALL}
DEFAULT_CONTROLLER = "Position PI (firmware)"
OPEN_LOOP_NAME = "Open-loop current"


class AppState:
    """Everything the server keeps between callbacks (one user, one process)."""

    def __init__(self, root, controllers_dir):
        self.root = Path(root)
        self.controllers_dir = Path(controllers_dir)
        self.store = RunStore()
        self.registry = {}
        self.x_range = None                 # visible window of the time traces
        self.log = None                     # (path, VcaLog, sim result or None)
        self.log_range = None
        self.reload_controllers()

    def reload_controllers(self):
        self.registry = {name: load_controller(path) for name, path in discover(self.controllers_dir).items()}
        if not self.registry:
            raise RuntimeError(f"no controller scripts found in {self.controllers_dir}")

    def log_options(self):
        paths = sorted(set(glob.glob(str(self.root / "data" / "**" / "voice_coil_log_*.csv"), recursive=True) +
                           glob.glob(str(self.root / "gcsc_data" / "**" / "voice_coil_log_*.csv"), recursive=True)))
        return [{"label": str(Path(p).relative_to(self.root)), "value": p} for p in paths]


# ---------------------------------------------------------------- plain functions behind the callbacks

def table(rows):
    """A list of dicts as an HTML table."""
    if not rows:
        return None
    cols = list(rows[0])
    return html.Table(className="table", children=[
        html.Thead(html.Tr([html.Th(c) for c in cols])),
        html.Tbody([html.Tr([html.Td(_cell(r.get(c))) for c in cols]) for r in rows])])


def _cell(v):
    if isinstance(v, float):
        return f"{v:.4g}"
    if isinstance(v, bool):
        return "yes" if v else "no"
    return "" if v is None else str(v)


def run_status(run):
    """Status line and warnings of a run, as page elements."""
    r, sc = run.result, run.scenario
    head = (f"{r.status.upper()}: {sc.controller.NAME} on {sc.plant.label}"
            + (f" (modified: {', '.join(sc.plant.modified)})" if sc.plant.modified else "")
            + f", {sc.duration_s:g} s simulated in {r.elapsed_s:.2f} s")
    items = [html.Div(head, className="status-ok" if r.ok else "status-bad")]
    if r.message:
        items.append(html.Div(r.message, className="status-bad"))
    items += [html.Div("Warning: " + w, className="status-warn") for w in r.warnings]
    return items


def form_complete(state, controller, ref_type, field_ids):
    """True when the fields on the page belong to the selected controller and reference. After a
    selection changes, its fields are re-rendered by a separate callback; until then the old ones are
    still on the page and a run would mix the two."""
    cls = state.registry.get(controller)
    unit_refs = forms.REF_DEFAULTS[cls.REF_UNIT] if cls else {}
    if cls is None or ref_type not in unit_refs:
        return False
    names = {}
    for i in field_ids:
        names.setdefault(i["group"], set()).add(i["name"])
    want_ref = {s.name for s in forms.specs_from_dataclass("ref", unit_refs[ref_type])}
    return (names.get("ctrl", set()) == {p.name for p in cls.PARAMS} and names.get("ref", set()) == want_ref
            and all(g in names for g in ("run", "plant", "drive", "laser", "accel", "host")))


def do_run(state, controller, ref_type, preset, field_ids, values):
    """Simulate the scenario described by the form. Returns (ok, status elements)."""
    try:
        cls = state.registry[controller]
        sc = forms.build_scenario(cls, ref_type, preset, forms.collect(field_ids, values))
        res = simulate(sc)
    except forms.FormError as e:
        return False, [html.Div(f"Error: {e}", className="status-bad")]
    except Exception as e:                                       # a controller script raising, for example
        return False, [html.Div(f"Error while simulating: {type(e).__name__}: {e}", className="status-bad"),
                       html.Pre(traceback.format_exc(limit=4), className="settings")]
    run = state.store.set_current(sc, res)
    return True, run_status(run)


def compute_tracking(state, visible, freqs, amp, dur, periods):
    key = (tuple(freqs), amp, dur, periods)
    series = []
    for run in state.store.shown(visible):
        if run.scenario.controller.REF_UNIT != "mm":
            continue
        if key not in run.tracking:
            run.tracking[key] = AN.tracking_response(run.scenario, freqs, amplitude_mm=amp, duration_s=dur,
                                                     window_periods=periods)
        series.append((run, run.tracking[key]))
    return series


def compute_loop_gain(state, visible, rms, period, lines):
    key = (rms, period, lines)
    series = []
    for run in state.store.shown(visible):
        if key not in run.loopgain:
            run.loopgain[key] = AN.loop_gain(run.scenario, rms_A=rms, period_s=period, n_lines=lines)
        series.append((run, run.loopgain[key]))
    return series


def load_log_and_replay(state, path, replay):
    """Load a rig log; with replay, repeat its experiment in the simulator with the current run's
    plant, drive and sensors. Returns a status text."""
    from vca_log import load_log
    lg = load_log(path)
    sim, msg = None, f"{Path(path).name}: {len(lg.run)} rows"
    if replay:
        if reference_from_log_name(path) is None:
            msg += "; not replayed: the command cannot be rebuilt from this file name (only chirp / chirpsched logs)"
        else:
            ol = state.registry.get(OPEN_LOOP_NAME) or load_controller(state.controllers_dir / "open_loop_current.py")
            cur = state.store.current
            kw = {} if cur is None else dict(drive=cur.scenario.drive, laser=cur.scenario.laser, accel=cur.scenario.accel)
            plant = cur.scenario.plant if cur is not None else PRESETS["greybox_C"]
            sim = simulate(scenario_from_log(path, ol, plant, **kw))
            msg += f"; replayed on {plant.label}: {sim.status}" + (f" ({sim.message})" if sim.message else "")
            msg += "".join(f"; warning: {w}" for w in sim.warnings)
    state.log, state.log_range = (path, lg, sim), None
    return msg


def only_relayout(graph_id):
    """True when a graph's zoom/resize event is the only thing that triggered this callback. A new run
    and Plotly's first 'autosize' event can arrive in the same request; that must still redraw."""
    return set(ctx.triggered_prop_ids) == {f"{graph_id}.relayoutData"}


# ---------------------------------------------------------------- the app

def create_app(root=".", controllers_dir=None):
    state = AppState(root, controllers_dir or Path(root) / "controllers")
    names = list(state.registry)
    app = Dash(__name__, title="VCA simulator", assets_folder=str(Path(__file__).parent / "assets"),
               suppress_callback_exceptions=True)
    default = DEFAULT_CONTROLLER if DEFAULT_CONTROLLER in names else names[0]
    app.layout = layout.build(names, default, state.registry[default].REF_UNIT, state.log_options())
    app.state = state

    # --- selections -> fields

    @app.callback(Output("ctrl-fields", "children"), Output("ctrl-description", "children"),
                  Output("ref-select", "options"), Output("ref-select", "value"), Output("ctrl-select", "options"),
                  Input("ctrl-select", "value"), Input("ctrl-reload", "n_clicks"), State("ref-select", "value"))
    def render_controller(name, _reload, ref_type):
        if ctx.triggered_id == "ctrl-reload":
            state.reload_controllers()
        if name not in state.registry:
            name = next(iter(state.registry))
        cls = state.registry[name]
        refs = list(forms.REF_DEFAULTS[cls.REF_UNIT])
        desc = f"{cls.DESCRIPTION} Reference in {cls.REF_UNIT}."
        return (forms.widgets("ctrl", forms.specs_from_controller(cls)), desc, refs,
                ref_type if ref_type in refs else refs[0], list(state.registry))

    @app.callback(Output("ref-fields", "children"), Input("ref-select", "value"), Input("ctrl-select", "value"))
    def render_reference(ref_type, name):
        unit = state.registry[name].REF_UNIT if name in state.registry else "mm"
        defaults = forms.REF_DEFAULTS[unit]
        spec = defaults.get(ref_type) or next(iter(defaults.values()))
        return forms.widgets("ref", forms.specs_from_dataclass("ref", spec), ref_unit=unit)

    @app.callback(Output("plant-fields", "children"), Output("plant-notes", "children"), Input("plant-select", "value"))
    def render_plant(key):
        p = PRESETS[key]
        equation = dcc.Markdown(f"$$\n{STRUCTURES[p.structure].latex}\n$$", mathjax=True, className="equation")
        return forms.widgets("plant", forms.specs_from_plant(p)), equation

    @app.callback(Output("drive-fields", "children"), Input("drive-select", "value"))
    def render_drive(key):
        return forms.widgets("drive", forms.specs_from_dataclass("drive", forms.DRIVE_PRESETS[key]))

    @app.callback(Output("laser-fields", "children"), Output("accel-fields", "children"), Input("sensor-select", "value"))
    def render_sensors(key):
        laser, accel = forms.SENSOR_PRESETS[key]
        return (forms.widgets("laser", forms.specs_from_dataclass("laser", laser)),
                forms.widgets("accel", forms.specs_from_dataclass("accel", accel)))

    # --- run

    # The selections are Inputs too (not State): whichever of a selection and its re-rendered fields
    # arrives last triggers the run, so the first run cannot be lost to the order of the renders.
    @app.callback(Output("run-version", "data"), Output("status", "children"),
                  Input("run-button", "n_clicks"), Input(FIELDS, "value"),
                  Input("ctrl-select", "value"), Input("ref-select", "value"), Input("plant-select", "value"),
                  State("auto-run", "value"), State("run-version", "data"))
    def run(_n, values, controller, ref_type, preset, auto, version):
        if ctx.triggered_id != "run-button" and ctx.triggered_id is not None and not auto:
            return no_update, no_update
        ids = [item["id"] for item in ctx.inputs_list[1]]
        if not form_complete(state, controller, ref_type, ids):
            return no_update, no_update                          # the form is still being re-rendered
        ok, status = do_run(state, controller, ref_type, preset, ids, values)
        return (version + 1) if ok else no_update, status

    # --- pinned runs

    @app.callback(Output("pinned-visible", "options"), Output("pinned-visible", "value"), Output("pin-message", "children"),
                  Input("pin-button", "n_clicks"), Input("pin-remove", "n_clicks"),
                  State("pin-label", "value"), State("pinned-visible", "value"))
    def pins(_pin, _remove, label, visible):
        """Also runs on page load, so a reloaded page lists the runs the server still holds."""
        visible = list(visible or []) if ctx.triggered_id is not None else list(state.store.pinned)
        msg = ""
        try:
            if ctx.triggered_id is None:
                pass
            elif ctx.triggered_id == "pin-button":
                run = state.store.pin(label or "")
                visible.append(run.id)
                msg = f"pinned as {run.label}"
            else:
                state.store.remove([i for i in state.store.pinned if i not in visible])
        except ValueError as e:
            msg = str(e)
        options = [{"label": html.Span([html.Span("■ ", style={"color": r.color}), r.label]), "value": r.id}
                   for r in state.store.pinned.values()]
        return options, [v for v in visible if v in state.store.pinned], msg

    # --- time traces

    @app.callback(Output("time-graph", "figure"), Output("metrics", "children"),
                  Input("run-version", "data"), Input("pinned-visible", "value"), Input("time-panels", "value"),
                  Input("time-graph", "relayoutData"))
    def time_traces(_v, visible, panels, relayout):
        if only_relayout("time-graph"):
            new = plots.x_range_from_relayout(relayout, state.x_range)
            if new == state.x_range:
                return no_update, no_update
            state.x_range = new
        runs = state.store.shown(visible)
        fig = plots.time_figure(runs, state.x_range, panels or ())
        rows = [{"run": r.label, **AN.tracking_metrics(r.result)} for r in runs]
        return fig, table(rows)

    # --- tracking per frequency

    @app.callback(Output("freq-graph", "figure"), Output("freq-table", "children"),
                  Input("freq-button", "n_clicks"), State("freq-list", "value"), State("freq-amp", "value"),
                  State("freq-dur", "value"), State("freq-periods", "value"), State("pinned-visible", "value"))
    def tracking(n, freq_text, amp, dur, periods, visible):
        if not n:
            return plots.empty("Press Compute"), None
        try:
            freqs = [float(f) for f in str(freq_text).replace(";", ",").split(",") if f.strip()]
            series = compute_tracking(state, visible, freqs, float(amp), float(dur), int(periods))
        except (ValueError, TypeError) as e:
            return plots.empty(f"Error: {e}"), None
        if not series:
            return plots.empty("No run with a position reference (an open-loop current controller has none)"), None
        kp = state.store.current.scenario.drive.kp_A if state.store.current else None
        rows = [{"run": run.label, **q} for run, rs in series for q in rs]
        return plots.tracking_figure(series, kp), table(rows)

    # --- loop gain

    @app.callback(Output("lg-graph", "figure"), Output("lg-table", "children"),
                  Input("lg-button", "n_clicks"), State("lg-rms", "value"), State("lg-period", "value"),
                  State("lg-lines", "value"), State("pinned-visible", "value"))
    def stability(n, rms, period, lines, visible):
        if not n:
            return plots.empty("Press Compute"), None
        try:
            series = compute_loop_gain(state, visible, float(rms), float(period), int(lines))
        except (ValueError, TypeError) as e:
            return plots.empty(f"Error: {e}"), None
        if not series:
            return plots.empty("No run yet"), None
        return plots.stability_figure(series), table(plots.margins_rows(series))

    # --- rig logs

    @app.callback(Output("log-graph", "figure"), Output("log-status", "children"),
                  Input("log-button", "n_clicks"), Input("log-graph", "relayoutData"),
                  State("log-select", "value"), State("log-replay", "value"))
    def logs(n, relayout, path, replay):
        if only_relayout("log-graph"):
            new = plots.x_range_from_relayout(relayout, state.log_range)
            if state.log is None or new == state.log_range:
                return no_update, no_update
            state.log_range = new
            return plots.log_figure(state.log[1], state.log[2], state.log_range, sim_color=CURRENT_COLOR), no_update
        if not n or not path:
            return plots.empty("Choose a log and press Load"), ""
        try:
            msg = load_log_and_replay(state, path, bool(replay))
        except Exception as e:
            return plots.empty("Could not load the log"), f"Error: {type(e).__name__}: {e}"
        return plots.log_figure(state.log[1], state.log[2], None, sim_color=CURRENT_COLOR), msg

    # --- export and settings

    @app.callback(Output("export-download", "data"), Input("export-button", "n_clicks"), prevent_initial_call=True)
    def export(_n):
        cur = state.store.current
        if cur is None:
            return no_update
        path = Path(tempfile.mkdtemp()) / f"voice_coil_log_sim_{cur.scenario.controller.NAME.split()[0].lower()}_{cur.id}.csv"
        write_rig_csv(cur.result, path)
        return dcc.send_file(str(path))

    @app.callback(Output("settings-select", "options"), Output("settings-select", "value"),
                  Input("run-version", "data"), Input("pinned-visible", "options"), State("settings-select", "value"))
    def settings_options(_v, _opts, value):
        runs = state.store.shown(list(state.store.pinned))
        options = [{"label": r.label, "value": r.id} for r in runs]
        ids = [o["value"] for o in options]
        return options, value if value in ids[1:] else (ids[0] if ids else None)

    @app.callback(Output("settings-view", "children"), Input("settings-select", "value"))
    def settings_view(run_id):
        run = state.store.get(run_id) if run_id else None
        return "" if run is None else json.dumps(run.result.settings, indent=2, default=str)

    return app
