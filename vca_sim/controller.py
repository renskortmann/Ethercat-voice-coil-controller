"""The controller plug-in interface.

A controller is a Python class in its own script (see controllers/ for examples). It is shaped like
the C loop so that it can later be written in C behind the same interface:

    C                                         Python
    ctrl_init(state, params, dt)              __init__(params, dt)  (then reset())
    ctrl_reset(state)                         reset()
    ctrl_step(state, const in_t*, out_t*)     step(m) -> u_A  or  (u_A, signals)
    ctrl_end_of_run(state, log)  [optional]   end_of_run(log)       [optional, not real-time]

step() is called once per 0.5 ms cycle, idle window included (m.active is False there, and the host
sends 0 A whatever step returns, so filters and observers can warm up as the firmware's notch does).
Its output is sent to the drive at the start of the NEXT cycle, exactly as on the rig. The host, not
the controller, owns the safety chain: non-finite guard, clamp to the drive peak current, position
trip. Any output limit of the strategy itself (like the PI's +-10 A) belongs in the controller.

Units follow the log columns: mm, A, V, s.
"""

import importlib.util
import inspect
import math
from dataclasses import dataclass, asdict
from pathlib import Path


@dataclass(frozen=True)
class Param:
    """One tunable parameter. The app builds an input widget from this."""
    name: str
    default: float
    min: float = -math.inf
    max: float = math.inf
    unit: str = ""
    kind: str = "float"          # "float", "int", "bool" or "choice"
    choices: tuple = ()
    help: str = ""

    def coerce(self, value):
        """Convert and range-check a value for this parameter."""
        if self.kind == "bool":
            return bool(value)
        if self.kind == "choice":
            if value not in self.choices:
                raise ValueError(f"{self.name}: {value!r} is not one of {self.choices}")
            return value
        v = int(value) if self.kind == "int" else float(value)
        if not (self.min <= v <= self.max):
            raise ValueError(f"{self.name} = {v} is outside {self.min} .. {self.max} {self.unit}")
        return v


class Measurement:
    """What the controller receives each cycle. The host reuses one instance for the whole run."""
    __slots__ = ("k", "t_s", "dt_s", "active",
                 "position_mm", "accel_V", "actual_current_A", "u_applied_prev_A",
                 "ref", "ref_vel", "ref_acc", "ref_freq_hz", "ref_preview")

    def __init__(self, dt_s):
        self.k, self.t_s, self.dt_s, self.active = 0, 0.0, dt_s, False
        self.position_mm = self.accel_V = self.actual_current_A = self.u_applied_prev_A = 0.0
        self.ref = self.ref_vel = self.ref_acc = 0.0
        self.ref_freq_hz = math.nan
        self.ref_preview = ()      # reference at cycles k+1 .. k+PREVIEW_TICKS (shorter at the end of the run)


class Controller:
    """Base class. Subclass it in a script and override the class attributes and step()."""
    NAME = "unnamed"
    DESCRIPTION = ""
    PARAMS = ()              # tuple of Param
    SIGNALS = ()             # names of the extra signals step() returns; they become log columns
    REF_UNIT = "mm"          # unit of the reference this controller expects: "mm" or "A"
    PREVIEW_TICKS = 0        # how many future reference samples step() needs (feedforward)

    def __init__(self, params=None, dt=0.0005):
        self.dt = dt
        self.p = self.resolve_params(params or {})
        self.reset()

    @classmethod
    def resolve_params(cls, overrides):
        """Defaults with overrides applied, every value checked against its Param."""
        spec = {p.name: p for p in cls.PARAMS}
        unknown = [n for n in overrides if n not in spec]
        if unknown:
            raise ValueError(f"{cls.NAME}: unknown parameters {unknown}; it has {list(spec)}")
        return {n: p.coerce(overrides.get(n, p.default)) for n, p in spec.items()}

    def reset(self):
        """Clear all state. Called once by __init__ and again before every run."""

    def step(self, m: Measurement):
        raise NotImplementedError

    def end_of_run(self, log):
        """Optional hook after a run (e.g. an ILC update between trials). log is the SimResult."""

    @classmethod
    def describe(cls):
        return {"name": cls.NAME, "description": cls.DESCRIPTION, "ref_unit": cls.REF_UNIT,
                "params": [asdict(p) for p in cls.PARAMS], "signals": list(cls.SIGNALS)}


def load_controller(path):
    """Import a controller script and return its Controller subclass.

    The script must define exactly one Controller subclass, or name the one to use in CONTROLLER.
    """
    path = Path(path)
    spec = importlib.util.spec_from_file_location(f"vca_controller_{path.stem}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, "CONTROLLER"):
        return module.CONTROLLER
    found = [obj for obj in vars(module).values()
             if inspect.isclass(obj) and issubclass(obj, Controller) and obj is not Controller
             and obj.__module__ == module.__name__]
    if len(found) != 1:
        raise ValueError(f"{path}: expected one Controller subclass, found {[c.__name__ for c in found]}; "
                         "set CONTROLLER = <class> to choose")
    return found[0]


def discover(folder="controllers"):
    """{controller NAME: script path} for every loadable script in a folder."""
    out = {}
    for p in sorted(Path(folder).glob("*.py")):
        if p.name.startswith("_"):
            continue
        out[load_controller(p).NAME] = p
    return out
