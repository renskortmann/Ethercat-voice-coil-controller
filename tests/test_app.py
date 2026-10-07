"""The app: forms round-trip into scenarios, figures build, and the callbacks work when called through
Dash's own HTTP endpoint the way the browser calls them."""

import json

import numpy as np
import pytest

from vca_sim import simulate, PRESETS
from vca_sim.app import forms, plots
from vca_sim.app.main import create_app, compute_tracking, compute_loop_gain, form_complete, FIELDS


# ---------------------------------------------------------------- helpers

def browser_value(spec):
    """What the browser sends for a field rendered from this spec."""
    if spec.kind == "bool":
        return ["on"] if spec.value else []
    if spec.kind == "literal":
        return repr(spec.value)
    return spec.value


def default_fields(cls, ref_type, preset):
    groups = [("ctrl", forms.specs_from_controller(cls)),
              ("ref", forms.specs_from_dataclass("ref", forms.REF_DEFAULTS[cls.REF_UNIT][ref_type])),
              ("run", forms.specs_run()), ("plant", forms.specs_from_plant(PRESETS[preset])),
              ("drive", forms.specs_from_dataclass("drive", list(forms.DRIVE_PRESETS.values())[0])),
              ("laser", forms.specs_from_dataclass("laser", forms.SENSOR_PRESETS["realistic (calibrated)"][0])),
              ("accel", forms.specs_from_dataclass("accel", forms.SENSOR_PRESETS["realistic (calibrated)"][1])),
              ("host", forms.specs_from_dataclass("host", forms.HostConfig()))]
    ids, values = [], []
    for g, specs in groups:
        for s in specs:
            ids.append(forms.field_id(g, s.name, s.kind))
            values.append(browser_value(s))
    return ids, values


@pytest.fixture(scope="module")
def app(root):
    return create_app(root)


@pytest.fixture(scope="module")
def client(app):
    return app.server.test_client()


@pytest.fixture(scope="module")
def deps(client):
    return client.get("/_dash-dependencies").get_json()


def call(client, deps, output_part, inputs, state=(), changed=()):
    """POST one callback the way dash-renderer does. inputs/state: lists of (id, prop, value), or for a
    pattern-matching input a list of such tuples."""
    dep = next(d for d in deps if output_part in d["output"])

    def item(x):
        return [item(y) for y in x] if isinstance(x, list) else {"id": x[0], "property": x[1], "value": x[2]}

    outputs = [{"id": o["id"], "property": o["property"]} for o in _outputs(dep)]
    payload = {"output": dep["output"], "outputs": outputs if len(outputs) > 1 else outputs[0],
               "inputs": [item(x) for x in inputs], "state": [item(x) for x in state],
               "changedPropIds": list(changed)}
    r = client.post("/_dash-update-component", json=payload)
    if r.status_code == 204:
        return None
    assert r.status_code == 200, r.data[:2000]
    return r.get_json()["response"]


def _array(x):
    """A trace array as JSON: a plain list, or Plotly's base64 typed array {"dtype", "bdata"}."""
    if isinstance(x, dict):
        import base64
        return np.frombuffer(base64.b64decode(x["bdata"]), dtype=x["dtype"])
    return np.asarray(x, float)


def _outputs(dep):
    """Output id/property pairs of a dependency, parsed from its 'output' string."""
    out = dep["output"]
    parts = out.strip(".").split("...") if out.startswith("..") else [out]
    res = []
    for p in parts:
        i, prop = p.rsplit(".", 1)
        res.append({"id": json.loads(i) if i.startswith("{") else i, "property": prop})
    return res


# ---------------------------------------------------------------- forms

def test_every_controller_and_reference_round_trips(app):
    st = app.state
    for name, cls in st.registry.items():
        for ref_type in forms.REF_DEFAULTS[cls.REF_UNIT]:
            ids, values = default_fields(cls, ref_type, "greybox_C")
            assert form_complete(st, name, ref_type, ids)
            sc = forms.build_scenario(cls, ref_type, "greybox_C", forms.collect(ids, values))
            sc = sc.__class__(**{**sc.__dict__, "duration_s": 0.2, "idle_s": 0.0})
            assert simulate(sc).status in ("ok", "trip"), (name, ref_type)


def test_plant_edit_is_recorded_and_bad_input_named(app):
    cls = app.state.registry["Position PI (firmware)"]
    ids, values = default_fields(cls, "Sine", "linear_msd")
    k = next(n for n, i in enumerate(ids) if i["group"] == "plant" and i["name"] == "c")
    values[k] = 197.0
    sc = forms.build_scenario(cls, "Sine", "linear_msd", forms.collect(ids, values))
    assert sc.plant.params["c"] == 197.0 and sc.plant.modified == ("c",)
    values[k] = None
    with pytest.raises(forms.FormError, match="plant.c"):
        forms.collect(ids, values)


def test_form_not_complete_while_switching_controller(app):
    ids, _ = default_fields(app.state.registry["Position PI (firmware)"], "Sine", "greybox_C")
    assert not form_complete(app.state, "Open-loop current", "Sine", ids)


# ---------------------------------------------------------------- figures

def test_decimate_keeps_extremes():
    t = np.linspace(0, 10, 200001)
    y = np.sin(2 * np.pi * 45 * t)
    y[123456] = 5.0
    tt, yy = plots.decimate(t, y, n=500)
    assert len(tt) <= 1000 and yy.max() == 5.0 and np.all(np.diff(tt) >= 0)
    tt, yy = plots.decimate(t, y, 2.0, 2.1)
    assert tt.min() <= 2.0 and tt.max() >= 2.1 and len(tt) == 2003    # short window: every sample kept


def test_relayout_parsing():
    assert plots.x_range_from_relayout({"xaxis3.range[0]": 1, "xaxis3.range[1]": 2}) == (1.0, 2.0)
    assert plots.x_range_from_relayout({"xaxis.autorange": True}, (1, 2)) is None
    assert plots.x_range_from_relayout({"autosize": True}, (1, 2)) == (1, 2)


# ---------------------------------------------------------------- callbacks through HTTP

def test_pages_serve(client):
    for url in ("/", "/_dash-layout", "/_dash-dependencies"):
        assert client.get(url).status_code == 200


def test_run_pin_and_analyse_through_callbacks(app, client, deps):
    st = app.state
    name = "Position PI (firmware)"
    out = call(client, deps, "ctrl-fields.children", [("ctrl-select", "value", name), ("ctrl-reload", "n_clicks", None)],
               [("ref-select", "value", None)], ["ctrl-select.value"])
    assert out["ref-select"]["value"] == "Sine"
    ids, values = default_fields(st.registry[name], "Sine", "greybox_C")
    fields = [(i, "value", v) for i, v in zip(ids, values)]
    out = call(client, deps, "run-version.data",
               [("run-button", "n_clicks", 1), fields, ("ctrl-select", "value", name), ("ref-select", "value", "Sine"),
                ("plant-select", "value", "greybox_C")],
               [("auto-run", "value", ["on"]), ("run-version", "data", 0)], ["run-button.n_clicks"])
    assert out["run-version"]["data"] == 1, out
    assert st.store.current.result.ok

    out = call(client, deps, "pinned-visible.options", [("pin-button", "n_clicks", 1), ("pin-remove", "n_clicks", None)],
               [("pin-label", "value", "baseline"), ("pinned-visible", "value", [])], ["pin-button.n_clicks"])
    pin_id = out["pinned-visible"]["value"][0]
    assert st.store.pinned[pin_id].label == "baseline"

    out = call(client, deps, "time-graph.figure",
               [("run-version", "data", 1), ("pinned-visible", "value", [pin_id]),
                ("time-panels", "value", ["position", "error", "current"]), ("time-graph", "relayoutData", None)],
               changed=["run-version.data"])
    fig = out["time-graph"]["figure"]
    names = [tr["name"] for tr in fig["data"]]
    assert "true position" in names and any("[baseline]" in n for n in names)

    # zooming re-thins the traces to the visible window
    out = call(client, deps, "time-graph.figure",
               [("run-version", "data", 1), ("pinned-visible", "value", []),
                ("time-panels", "value", ["position"]),
                ("time-graph", "relayoutData", {"xaxis.range[0]": 1.0, "xaxis.range[1]": 1.1})],
               changed=["time-graph.relayoutData"])
    xs = _array(out["time-graph"]["figure"]["data"][1]["x"])
    assert min(xs) >= 0.999 and max(xs) <= 1.101


def test_tracking_and_loop_gain_on_store(app):
    st = app.state
    if st.store.current is None:
        pytest.skip("needs the run from the previous test")
    series = compute_tracking(st, list(st.store.pinned), [40.0, 50.0], 0.5, 1.0, 10)
    assert series and all(q["status"] == "ok" for _, rows in series for q in rows)
    plots.tracking_figure(series, 60.0)
    lgs = compute_loop_gain(st, [], 0.2, 1.0, 30)
    assert lgs[0][1].status == "ok"
    assert plots.margins_rows(lgs)
    plots.stability_figure(lgs)


def test_rig_log_replay(app, root):
    from vca_sim.app.main import load_log_and_replay
    path = str(next((root / "data" / "test").glob("voice_coil_log_chirp_*.csv")))
    msg = load_log_and_replay(app.state, path, True)
    assert "replayed" in msg and app.state.log[2] is not None
    fig = plots.log_figure(app.state.log[1], app.state.log[2], (100.0, 100.3))
    assert len(fig.data) == 4
