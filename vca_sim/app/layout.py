"""Page layout: settings on the left, results on the right in four tabs."""

from dash import dcc, html

from ..plant import PRESETS, DEFAULT_PRESET
from . import forms
from .plots import PANELS


def tool(label, component, width=150):
    """A labelled input in a toolbar, at a fixed width (Dash 4 inputs otherwise fill the row)."""
    return html.Div(className="tool-field", style={"width": f"{width}px"}, children=[html.Label(label), component])


def section(title, *children, open_=True):
    return html.Details(open=open_, className="section", children=[html.Summary(title), *children])


def sidebar(controller_names, default_controller, ref_unit):
    return html.Div(className="sidebar", children=[
        html.H1("VCA simulator"),
        section("Plant model",
                dcc.Dropdown(id="plant-select", options=[{"label": p.label, "value": k} for k, p in PRESETS.items()],
                             value=DEFAULT_PRESET, clearable=False),
                html.Div(id="plant-notes", className="note"),
                html.Div(id="plant-fields")),
        section("Controller",
                html.Div(className="row", children=[
                    dcc.Dropdown(id="ctrl-select", options=controller_names, value=default_controller,
                                 clearable=False, className="grow"),
                    html.Button("Reload scripts", id="ctrl-reload", className="small")]),
                html.Div(id="ctrl-description", className="note"),
                html.Div(id="ctrl-fields")),
        section("Reference",
                dcc.Dropdown(id="ref-select", options=list(forms.REF_DEFAULTS[ref_unit]),
                             value=next(iter(forms.REF_DEFAULTS[ref_unit])), clearable=False),
                html.Div(id="ref-fields")),
        section("Run",
                html.Div(forms.widgets("run", forms.specs_run())),
                dcc.Checklist(id="auto-run", options=[{"label": " re-run on every change", "value": "on"}],
                              value=["on"], className="check"),
                html.Button("Run", id="run-button", className="primary")),
        section("Sensors",
                dcc.Dropdown(id="sensor-select", options=list(forms.SENSOR_PRESETS),
                             value=list(forms.SENSOR_PRESETS)[0], clearable=False),
                html.H4("Laser"), html.Div(id="laser-fields"),
                html.H4("Accelerometer"), html.Div(id="accel-fields"), open_=False),
        section("Drive",
                dcc.Dropdown(id="drive-select", options=list(forms.DRIVE_PRESETS), value=list(forms.DRIVE_PRESETS)[0],
                             clearable=False),
                html.Div(id="drive-fields"), open_=False),
        section("Host safety",
                html.Div(forms.widgets("host", forms.specs_from_dataclass("host", forms.HostConfig()))), open_=False),
    ])


def main_panel(log_options):
    return html.Div(className="main", children=[
        html.Div(id="status", className="status"),
        html.Div(className="toolbar", children=[
            dcc.Input(id="pin-label", type="text", placeholder="label for the pinned run", className="grow"),
            html.Button("Pin current run", id="pin-button"),
            html.Span(id="pin-message", className="note"),
            html.Button("Export CSV", id="export-button"),
            dcc.Download(id="export-download"),
        ]),
        html.Div(className="toolbar", children=[
            html.Span("Pinned runs (shown when ticked):", className="note"),
            dcc.Checklist(id="pinned-visible", options=[], value=[], inline=True, className="pins"),
            html.Button("Remove unticked", id="pin-remove", className="small"),
        ]),
        dcc.Tabs(id="tabs", value="time", children=[
            dcc.Tab(label="Time traces", value="time", children=[
                dcc.Checklist(id="time-panels", options=list(PANELS), value=["position", "error", "current"],
                              inline=True, className="check"),
                dcc.Loading(dcc.Graph(id="time-graph", config={"displaylogo": False})),
                html.Div(id="metrics", className="table-wrap"),
            ]),
            dcc.Tab(label="Tracking per frequency", value="freq", children=[
                html.P("Stepped sine position references, one run per frequency with the current settings. "
                       "The last window of `periods` cycles is compared with the one before it: an open marker "
                       "means the response had not settled. Position references only. The laser curve near 50 Hz includes "
                       "the simulated mains pickup; the true curve does not.", className="note"),
                html.Div(className="toolbar", children=[
                    tool("frequencies (Hz)", dcc.Input(id="freq-list", value="35, 40, 45, 50, 55", type="text"), 260),
                    tool("amplitude (mm)", dcc.Input(id="freq-amp", value=0.5, type="number", step="any")),
                    tool("run length (s)", dcc.Input(id="freq-dur", value=2.0, type="number", step="any")),
                    tool("periods", dcc.Input(id="freq-periods", value=20, type="number", step=1)),
                    html.Button("Compute", id="freq-button", className="primary"),
                ]),
                dcc.Loading(dcc.Graph(id="freq-graph", config={"displaylogo": False})),
                html.Div(id="freq-table", className="table-wrap"),
            ]),
            dcc.Tab(label="Stability", value="stab", children=[
                html.P("Loop gain at the controller output: the scenario is run twice with the same noise, once "
                       "with a multisine added to the controller output; L = -dU_ctrl / dU_sent. The one-cycle delay, "
                       "drive and sensors are inside L. For a nonlinear loop this is the linearisation along the "
                       "run: try two injection levels; if the curves differ, L depends on amplitude.",
                       className="note"),
                html.Div(className="toolbar", children=[
                    tool("injection rms (A)", dcc.Input(id="lg-rms", value=0.2, type="number", step="any")),
                    tool("period (s)", dcc.Input(id="lg-period", value=1.0, type="number", step="any")),
                    tool("lines", dcc.Input(id="lg-lines", value=60, type="number", step=1)),
                    html.Button("Compute", id="lg-button", className="primary"),
                ]),
                dcc.Loading(dcc.Graph(id="lg-graph", config={"displaylogo": False})),
                html.Div(id="lg-table", className="table-wrap"),
            ]),
            dcc.Tab(label="Rig logs", value="logs", children=[
                html.P("A rig log, and for open-loop chirp logs the same experiment repeated in the simulator with "
                       "the current plant, drive and sensor settings (the command is rebuilt from the file name).",
                       className="note"),
                html.Div(className="toolbar", children=[
                    dcc.Dropdown(id="log-select", options=log_options, value=None, placeholder="choose a log",
                                 className="grow"),
                    dcc.Checklist(id="log-replay", options=[{"label": " replay in simulator", "value": "on"}],
                                  value=["on"], className="check"),
                    html.Button("Load", id="log-button", className="primary"),
                ]),
                html.Div(id="log-status", className="note"),
                dcc.Loading(dcc.Graph(id="log-graph", config={"displaylogo": False})),
            ]),
            dcc.Tab(label="Run settings", value="settings", children=[
                html.P("Everything that defines a run, as stored with it.", className="note"),
                dcc.Dropdown(id="settings-select", clearable=False),
                html.Pre(id="settings-view", className="settings"),
            ]),
        ]),
        dcc.Store(id="run-version", data=0),
    ])


def build(controller_names, default_controller, ref_unit, log_options):
    return html.Div(className="page", children=[sidebar(controller_names, default_controller, ref_unit),
                                                main_panel(log_options)])
