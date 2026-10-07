"""Input forms: settings specs -> widgets, and widget values -> a Scenario.

Every input in the app is one *field* with a pattern-matching id
{"type": "field", "group": <group>, "name": <name>, "kind": <kind>}, so one callback can collect all
of them, whichever controller, reference or preset is on screen. The groups are:

    ctrl   the controller's PARAMS            ref    the reference's fields
    plant  the preset's parameters            run    duration, idle window, seed, substeps
    drive, laser, accel, host                 the corresponding config dataclasses

Kinds: "number", "int", "bool", "choice", "literal" (a Python literal typed as text, for tuples).
"""

import ast
import dataclasses
import math

import numpy as np
from dataclasses import dataclass

from dash import dcc, html

from .. import references as R
from ..drive import DriveConfig, IDEAL_DRIVE
from ..loop import HostConfig, Scenario
from ..plant import PRESETS, STRUCTURES
from ..sensors import LaserConfig, AccelConfig, NOISE_FREE_LASER, NOISE_FREE_ACCEL

FIELD = "field"

# Reference shapes offered in the app, with defaults per reference unit (FromArray is for scripts).
REF_DEFAULTS = {
    "mm": {
        "Sine": R.Sine(0.0, 0.5, 45.0),
        "Steps": R.Steps(((0.0, 0.0), (1.0, 1.0), (3.0, -1.0)), 0.0),
        "Chirp": R.Chirp(0.0, 0.5, 35.0, 55.0, 4.0, 0.5),
        "SineBlocks": R.SineBlocks(((35.0, 0.5), (45.0, 0.5), (55.0, 0.5)), 0.2, 1.0, 1.5),
        "Constant": R.Constant(0.0),
    },
    "A": {
        "Sine": R.Sine(0.0, 5.0, 45.0, unit="A"),
        "Steps": R.Steps(((0.0, 0.0), (0.5, 1.0), (1.5, -1.0), (2.5, 0.0)), 0.0, unit="A"),
        "Chirp": R.Chirp(0.0, 5.0, 10.0, 55.0, 4.0, 0.0, stop="hard", unit="A"),
        "ScheduledChirp": R.ScheduledChirp(((0.0, 2.0), (1.5, 5.0)), 1.0, 10.0, 55.0, 4.0, unit="A"),
        "SineBlocks": R.SineBlocks(((35.0, 5.0), (45.0, 5.0), (55.0, 5.0)), 0.2, 1.0, 1.5, unit="A"),
        "Constant": R.Constant(0.0, unit="A"),
    },
}
REF_UNITS = {"offset": "ref", "amplitude": "ref", "value": "ref", "freq_hz": "Hz", "f0_hz": "Hz", "f1_hz": "Hz",
             "duration_s": "s", "start_s": "s", "ramp_s": "s", "hold_s": "s", "period_s": "s"}
REF_HELP = {"points": "((t_s, value), ...)", "blocks": "((freq_hz, amplitude), ...)",
            "schedule": "((t_s, amplitude), ...)", "stop": "zero_crossing: finish the cycle; hard: cut at duration"}
CHOICES = {("ref", "stop"): ("zero_crossing", "hard")}

DRIVE_PRESETS = {"measured (calibrated)": DriveConfig(), "ideal": IDEAL_DRIVE}
SENSOR_PRESETS = {"realistic (calibrated)": (LaserConfig(), AccelConfig()),
                  "noise-free": (NOISE_FREE_LASER, NOISE_FREE_ACCEL)}
RUN_DEFAULTS = {"duration_s": 3.0, "idle_s": 0.5, "start_position_mm": 0.0, "seed": 0, "substeps": 1}
RUN_UNITS = {"duration_s": "s", "idle_s": "s", "start_position_mm": "mm"}
RUN_HELP = {"idle_s": "0 A window before t = 0 (firmware: 3 s)", "substeps": "RK4 steps per 0.5 ms cycle",
            "start_position_mm": "true position at the start of the run, at rest; the mass settles from here "
                                 "towards the model's own rest position during the idle window",
            "seed": "noise seed; same seed, same noise"}

UNITS = {
    "drive": {"kp_A": "A", "shelf_tau_s": "s", "lag_s": "s", "delay_ticks": "cycles", "report_noise_A": "A"},
    "laser": {"delay_s": "s", "noise_mm": "mm", "mains_hz": "Hz", "mains_amp_mm": "mm", "mains3_amp_mm": "mm",
              "max_mm": "mm", "min_mm": "mm"},
    "accel": {"delay_s": "s", "lowpass_hz": "Hz", "scale_V_per_g": "V/g", "bias_V": "V", "noise_V": "V"},
    "host": {"trip_min_mm": "mm", "trip_max_mm": "mm", "trip_cycles": "cycles", "stroke_limit_mm": "mm"},
}


@dataclass(frozen=True)
class FieldSpec:
    name: str
    value: object
    kind: str
    unit: str = ""
    help: str = ""
    choices: tuple = ()


def field_id(group, name, kind):
    return {"type": FIELD, "group": group, "name": name, "kind": kind}


def _kind_of(value):
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "number"
    if isinstance(value, tuple):
        return "literal"
    return "choice"


# ---------------------------------------------------------------- specs

def specs_from_dataclass(group, obj, skip=("unit", "label")):
    out = []
    for f in dataclasses.fields(obj):
        if f.name in skip:
            continue
        v = getattr(obj, f.name)
        kind = _kind_of(v)
        unit = UNITS.get(group, {}).get(f.name, "") if group != "ref" else REF_UNITS.get(f.name, "")
        out.append(FieldSpec(f.name, v, kind, unit, REF_HELP.get(f.name, "") if group == "ref" else "",
                             CHOICES.get((group, f.name), ())))
    return out


def specs_from_controller(cls):
    return [FieldSpec(p.name, p.default, {"float": "number"}.get(p.kind, p.kind), p.unit, p.help, tuple(p.choices))
            for p in cls.PARAMS]


def specs_from_plant(model):
    units = STRUCTURES[model.structure].param_units
    return [FieldSpec(n, v, "number", units.get(n, "")) for n, v in model.params.items()]


def specs_run():
    return [FieldSpec(n, v, _kind_of(v), RUN_UNITS.get(n, ""), RUN_HELP.get(n, "")) for n, v in RUN_DEFAULTS.items()]


# ---------------------------------------------------------------- widgets

def _fmt(v):
    """Numbers shown without float noise but at full precision."""
    return float(repr(v)) if isinstance(v, float) else v


def widget(group, spec: FieldSpec, ref_unit="mm"):
    """One labelled input for a field spec."""
    fid = field_id(group, spec.name, spec.kind)
    if spec.kind in ("number", "int"):
        ctl = dcc.Input(id=fid, type="number", value=_fmt(spec.value), debounce=True,
                        step=1 if spec.kind == "int" else "any", className="field-input")
    elif spec.kind == "bool":
        ctl = dcc.Checklist(id=fid, options=[{"label": "", "value": "on"}], value=["on"] if spec.value else [],
                            className="field-check")
    elif spec.kind == "choice":
        ctl = dcc.Dropdown(id=fid, options=list(spec.choices), value=spec.value, clearable=False,
                           className="field-choice")
    else:
        ctl = dcc.Input(id=fid, type="text", value=repr(spec.value), debounce=True, className="field-input wide")
    unit = ref_unit if spec.unit == "ref" else spec.unit
    return html.Div(className="field", title=spec.help, children=[
        html.Label(spec.name, className="field-label"), ctl, html.Span(unit, className="field-unit")])


def widgets(group, specs, ref_unit="mm"):
    return [widget(group, s, ref_unit) for s in specs]


# ---------------------------------------------------------------- values -> objects

class FormError(ValueError):
    """A field value that cannot be used; the message names the field."""


def parse(group, name, kind, raw):
    try:
        if kind == "bool":
            return bool(raw)
        if kind == "choice":
            return raw
        if kind == "literal":
            return ast.literal_eval(raw)
        if raw is None or raw == "":
            raise ValueError("is empty")
        v = int(raw) if kind == "int" else float(raw)
        if isinstance(v, float) and not math.isfinite(v):
            raise ValueError("is not a finite number")
        return v
    except (ValueError, SyntaxError) as e:
        raise FormError(f"{group}.{name}: {raw!r} {e}") from None


def collect(ids, values):
    """{group: {name: parsed value}} from the pattern-matching ids and values of all fields."""
    out = {}
    for fid, raw in zip(ids, values):
        out.setdefault(fid["group"], {})[fid["name"]] = parse(fid["group"], fid["name"], fid["kind"], raw)
    return out


def _tupleify(v):
    """Lists from a literal (e.g. [[0, 0], [1, 1]]) as nested tuples, so references stay hashable."""
    return tuple(_tupleify(x) for x in v) if isinstance(v, (list, tuple)) else float(v) if isinstance(v, int) else v


def build_scenario(controller_cls, ref_type, preset_key, form):
    """A Scenario from the selections and the collected field values. Raises FormError on bad input."""
    try:
        ref_fields = {k: _tupleify(v) for k, v in form.get("ref", {}).items()}
        reference = R.REFERENCE_TYPES[ref_type](**ref_fields, unit=controller_cls.REF_UNIT)
        reference.evaluate(np.array([0.0, 1e-3]))                       # catch malformed tables now
        plant = PRESETS[preset_key].with_params(**form.get("plant", {}))
        run = {**RUN_DEFAULTS, **form.get("run", {})}
        if run["duration_s"] <= 0 or run["idle_s"] < 0:
            raise FormError("run: duration must be > 0 and idle >= 0")
        controller_cls.resolve_params(form.get("ctrl", {}))
        return Scenario(
            controller=controller_cls, controller_params=form.get("ctrl", {}), reference=reference, plant=plant,
            drive=DriveConfig(**form.get("drive", {})), laser=LaserConfig(**form.get("laser", {})),
            accel=AccelConfig(**form.get("accel", {})), host=HostConfig(**form.get("host", {})),
            duration_s=run["duration_s"], idle_s=run["idle_s"], seed=int(run["seed"]),
            substeps=max(1, int(run["substeps"])), x0_mm=float(run["start_position_mm"]))
    except FormError:
        raise
    except (TypeError, ValueError, IndexError, KeyError) as e:
        raise FormError(str(e)) from None
