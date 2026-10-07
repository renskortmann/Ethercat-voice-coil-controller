"""Plant models of the voice coil: one moving mass on magnetic end springs, driven by coil current.

Every model has the same form, in SI units (x in m, v in m/s, i in A, forces in N):

    m a = Kf(x) i - c v - Fs(x)

Kf(x) is the motor constant and Fs(x) the restoring force of the magnetic springs (subtracted, so a
stiff spring has dFs/dx > 0). A *structure* defines the shape of Kf and Fs and the names of its
parameters; a *preset* is a structure with fitted values, the notebook cell they came from, and the
range of motion the fit data covered (its validity envelope).

The presets are copied from the notebooks, which are left untouched. If a model is refitted there,
update its preset here by hand.
"""

import math
from dataclasses import dataclass, field, replace

M_KG = 51.5   # moving mass, kg (fixed in every fit)

# Datasheet motor constant shape, Kf(z) = 283 - 0.0198 z - 0.578 z^2 N/A with z in mm, written in metres:
# 0.0198 z = 19.8 x and 0.578 z^2 = 578000 x^2.
_DS_K0, _DS_K1, _DS_K2 = 283.0, 19.8, 578000.0


@dataclass(frozen=True)
class Envelope:
    """Where a model was fitted. Outside it the model is an extrapolation."""
    x_min_mm: float
    x_max_mm: float
    f_min_hz: float
    f_max_hz: float
    note: str = ""


@dataclass(frozen=True)
class Structure:
    """A model shape: its parameter names and a factory for the Kf(x) and Fs(x) functions."""
    key: str
    equation: str
    param_names: tuple
    param_units: dict
    make_forces: object   # callable(params dict) -> (kf(x), fs(x)), both plain float functions
    latex: str = ""       # the equation for display (MathJax), in the parameter names above


def _poly_forces(p):
    """Polynomial Kf and spring: Kf = kf0 + kf1 x + kf2 x^2, Fs = F0 + k1 x + ... + k5 x^5 (Horner form)."""
    kf0, kf1, kf2 = p["kf0"], p["kf1"], p["kf2"]
    F0, k1, k2, k3, k4, k5 = p["F0"], p["k1"], p["k2"], p["k3"], p["k4"], p["k5"]

    def kf(x):
        return kf0 + x * (kf1 + x * kf2)

    def fs(x):
        return F0 + x * (k1 + x * (k2 + x * (k3 + x * (k4 + x * k5))))

    return kf, fs


def _datasheet_shape_forces(p):
    """Kf = Gamma0 times the datasheet curve normalised to 1 at the centre; cubic polynomial spring."""
    g = p["Gamma0"] / _DS_K0
    k1, k2, k3 = p["k1"], p["k2"], p["k3"]

    def kf(x):
        return g * (_DS_K0 - _DS_K1 * x - _DS_K2 * x * x)

    def fs(x):
        return x * (k1 + x * (k2 + x * k3))

    return kf, fs


def _magnets_forces(p):
    """Kf = Gamma + Gamma1 x; spring of two unequal repelling magnets at gap g0 from the centre."""
    G, G1, C_R, C_L, g0 = p["Gamma"], p["Gamma1"], p["C_R"], p["C_L"], p["g0"]

    def kf(x):
        return G + G1 * x

    def fs(x):
        return C_R / (g0 - x) ** 2 - C_L / (g0 + x) ** 2   # singular at x = +-g0 (the magnets touch)

    return kf, fs


_COMMON_UNITS = {"m": "kg", "c": "N s/m"}

STRUCTURES = {
    s.key: s for s in (
        Structure(
            "poly",
            "m a = (kf0 + kf1 x + kf2 x^2) i - c v - (F0 + k1 x + k2 x^2 + k3 x^3 + k4 x^4 + k5 x^5)",
            ("m", "c", "kf0", "kf1", "kf2", "F0", "k1", "k2", "k3", "k4", "k5"),
            {**_COMMON_UNITS, "kf0": "N/A", "kf1": "N/(A m)", "kf2": "N/(A m^2)", "F0": "N",
             "k1": "N/m", "k2": "N/m^2", "k3": "N/m^3", "k4": "N/m^4", "k5": "N/m^5"},
            _poly_forces,
            r"\begin{aligned} m\ddot{x} &= \left(k_{f0} + k_{f1}x + k_{f2}x^2\right) i - c\dot{x} \\"
            r" &\quad - \left(F_0 + k_1x + k_2x^2 + k_3x^3 + k_4x^4 + k_5x^5\right) \end{aligned}",
        ),
        Structure(
            "datasheet_shape",
            "m a = Gamma0 s(x) i - c v - (k1 x + k2 x^2 + k3 x^3),  s(x) = datasheet Kf(x) / 283",
            ("m", "c", "Gamma0", "k1", "k2", "k3"),
            {**_COMMON_UNITS, "Gamma0": "N/A", "k1": "N/m", "k2": "N/m^2", "k3": "N/m^3"},
            _datasheet_shape_forces,
            r"\begin{aligned} m\ddot{x} &= \Gamma_0\, s(x)\, i - c\dot{x} - \left(k_1x + k_2x^2 + k_3x^3\right) \\"
            r" s(x) &= \frac{283 - 0.0198z - 0.578z^2}{283}, \quad z = 1000x \end{aligned}",
        ),
        Structure(
            "magnets",
            "m a = (Gamma + Gamma1 x) i - c v - (C_R / (g0 - x)^2 - C_L / (g0 + x)^2)",
            ("m", "c", "Gamma", "Gamma1", "C_R", "C_L", "g0"),
            {**_COMMON_UNITS, "Gamma": "N/A", "Gamma1": "N/(A m)", "C_R": "N m^2", "C_L": "N m^2", "g0": "m"},
            _magnets_forces,
            r"\begin{aligned} m\ddot{x} &= \left(\Gamma + \Gamma_1 x\right) i - c\dot{x} \\"
            r" &\quad - \left(\frac{C_R}{(g_0 - x)^2} - \frac{C_L}{(g_0 + x)^2}\right) \end{aligned}",
        ),
    )
}


@dataclass(frozen=True)
class PlantModel:
    """A structure with parameter values. Immutable: use with_params() for a modified copy."""
    key: str
    label: str
    structure: str
    params: dict
    source: str
    envelope: Envelope
    notes: str = ""
    fitted_on_rig: bool = False           # fitted with the drive's reported current as input
    modified: tuple = field(default=())   # names of parameters changed from the preset values

    def __post_init__(self):
        names = STRUCTURES[self.structure].param_names
        missing = [n for n in names if n not in self.params]
        extra = [n for n in self.params if n not in names]
        if missing or extra:
            raise ValueError(f"{self.key}: missing parameters {missing}, unknown parameters {extra}")

    def with_params(self, **changes):
        """Copy with some parameters changed; the changed names are recorded in `modified`."""
        unknown = [n for n in changes if n not in self.params]
        if unknown:
            raise ValueError(f"{self.key} has no parameters {unknown}; it has {list(self.params)}")
        changed = {n: float(v) for n, v in changes.items() if float(v) != self.params[n]}
        return replace(self, params={**self.params, **changed},
                       modified=tuple(sorted(set(self.modified) | set(changed))))

    def forces(self):
        """(kf(x), fs(x)) as plain float functions, x in metres."""
        return STRUCTURES[self.structure].make_forces(self.params)

    def make_acc(self):
        """acc(x, v, i) in m/s^2, a closure over plain floats so it is fast in a scalar loop."""
        kf, fs = self.forces()
        m, c = self.params["m"], self.params["c"]

        def acc(x, v, i):
            return (kf(x) * i - c * v - fs(x)) / m

        return acc

    def equilibrium_m(self):
        """Rest position at 0 A: the root of Fs(x) = 0 nearest the centre (bisection on +-15 mm)."""
        _, fs = self.forces()
        if fs(0.0) == 0.0:
            return 0.0
        # walk outwards from the centre in 0.1 mm steps to the first sign change, then bisect
        step, f0 = 1e-4, fs(0.0)
        for n in range(1, 151):
            for x in (n * step, -n * step):
                if fs(x) * f0 <= 0.0:
                    lo, hi = (0.0, x) if x > 0 else (x, 0.0)
                    for _ in range(60):
                        mid = 0.5 * (lo + hi)
                        if fs(lo) * fs(mid) <= 0.0:
                            hi = mid
                        else:
                            lo = mid
                    return 0.5 * (lo + hi)
        raise ValueError(f"{self.key}: no rest position within +-15 mm (spring force never crosses zero)")

    def stiffness(self, x_m):
        """Small-signal stiffness dFs/dx at x, N/m (central difference)."""
        _, fs = self.forces()
        h = 1e-7
        return (fs(x_m + h) - fs(x_m - h)) / (2 * h)

    def small_signal(self):
        """Linearisation at the rest position: stiffness, motor constant, natural frequency, damping ratio."""
        x0 = self.equilibrium_m()
        kf, _ = self.forces()
        k = self.stiffness(x0)
        m, c = self.params["m"], self.params["c"]
        wn = math.sqrt(k / m) if k > 0 else float("nan")
        return {"x0_mm": x0 * 1e3, "k_N_per_m": k, "Kf_N_per_A": kf(x0), "f_n_Hz": wn / (2 * math.pi),
                "zeta": c / (2 * m * wn) if k > 0 else float("nan"),
                "static_gain_mm_per_A": kf(x0) / k * 1e3 if k > 0 else float("nan")}

    def describe(self):
        """Settings as a plain dict, for run records."""
        return {"key": self.key, "structure": self.structure, "params": dict(self.params),
                "modified": list(self.modified), "source": self.source}


def _poly(**p):
    """Poly-structure parameters with every unused coefficient set to 0."""
    base = dict(m=M_KG, c=0.0, kf0=0.0, kf1=0.0, kf2=0.0, F0=0.0, k1=0.0, k2=0.0, k3=0.0, k4=0.0, k5=0.0)
    return {**base, **p}


_GREYBOX_ENVELOPE = Envelope(-5.7, 6.5, 10.0, 55.0,
                             "scheduled chirp 10-55 Hz, 2 Oct (train -5.7..+4.5 mm, test -4.4..+6.5 mm)")
_X_SYSID = 0.02   # the 23 Sep notebook fits against x / 0.02 m; its coefficients are converted below

PRESETS = {
    p.key: p for p in (
        PlantModel(
            "greybox_C", "Greybox C: unequal magnets + Kf(x)", "magnets",
            dict(m=M_KG, c=2123.652514570132, Gamma=234.0207758472696, Gamma1=5548.149144456808,
                 C_R=0.16892310375739375, C_L=0.1664659861264795, g0=22.3e-3),
            "vca_greybox_fit.ipynb cell 17 (p_C, full precision from a headless rerun)",
            _GREYBOX_ENVELOPE,
            "Best on test data (NRMSE 0.587 at 1000 steps). Singular at +-22.3 mm. Kf fitted against the "
            "drive's reported actual current, so pair it with the measured drive model.",
            fitted_on_rig=True,
        ),
        PlantModel(
            "greybox_A", "Greybox A: cubic spring + Kf(x)", "poly",
            _poly(c=2133.477855084445, kf0=235.3263581609583, kf1=6535.5681020840075,
                  k1=58562.664482357395, k2=-970698.5101312505, k3=0.0),
            "vca_greybox_fit.ipynb cell 11 (p_p3; k3 at its lower bound 0)",
            _GREYBOX_ENVELOPE,
            "No hardening term (k3 = 0); Kf(x) halves near -18 mm. Used by the notebook's EKF.",
            fitted_on_rig=True,
        ),
        PlantModel(
            "greybox_B", "Greybox B: cubic spring + datasheet Kf shape", "datasheet_shape",
            dict(m=M_KG, c=2104.9042425533835, Gamma0=231.0441789007533,
                 k1=64264.61900344632, k2=5401291.674981891, k3=0.0),
            "vca_greybox_fit.ipynb cell 14 (p_B; k3 at its lower bound 0)",
            _GREYBOX_ENVELOPE,
            "Negative stiffness below about -5.95 mm (k2 > 0, k3 = 0): diverges if driven there.",
            fitted_on_rig=True,
        ),
        PlantModel(
            "sysid_poly5", "Sys id model C (23 Sep): quintic spring + quadratic Kf", "poly",
            _poly(c=1912.609342, kf0=249.943321, kf1=-11.077225 / _X_SYSID, kf2=-164.924431 / _X_SYSID ** 2,
                  F0=26.531859, k1=799.024948 / _X_SYSID, k3=-2055.526698 / _X_SYSID ** 3,
                  k5=2043.350267 / _X_SYSID ** 5),
            "vca_system_identification.ipynb cell 46 (Phase 5 p_final, scaled units converted to SI)",
            Envelope(-16.0, 16.0, 1.0, 55.0, "6.5 A chirp 1-55 Hz, 23 Sep; position VAF 94 %, in-band holdout 22 %"),
            "Negative stiffness at roughly 8.7-12.9 mm. Acceleration fit is poor (VAF 36 %).",
            fitted_on_rig=True,
        ),
        PlantModel(
            "linear_msd", "Linear mass-spring-damper", "poly",
            _poly(c=1160.0, kf0=250.0, k1=35500.0),
            "docs/position-control.md on origin/position_control: k 35-40 kN/m, Kf 250 N/A; c = datasheet",
            Envelope(-10.0, 10.0, 0.0, 100.0, "idealised; for analytic checks and controller sanity tests"),
            "Damping is the main unknown: estimates range 197-2134 N s/m. Edit c to see its effect.",
        ),
        PlantModel(
            "datasheet", "Datasheet values", "datasheet_shape",
            dict(m=M_KG, c=1160.0, Gamma0=283.0, k1=M_KG * (2 * math.pi * 4.5) ** 2, k2=0.0, k3=0.0),
            "vca_identification_plan.ipynb cell 2: Kf(z), c = 1160 N s/m, resonance about 4.5 Hz (sets k1)",
            Envelope(-18.0, 18.0, 0.0, 100.0, "datasheet, not fitted to this rig"),
            "Linear spring chosen to give the 4.5 Hz resonance; datasheet Kf(x) shape.",
        ),
    )
}

DEFAULT_PRESET = "greybox_C"
