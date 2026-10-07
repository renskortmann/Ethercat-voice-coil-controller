"""The 2 kHz closed-loop engine: plant, drive, sensors, host safety chain and a controller plug-in.

One cycle k (time t_k = k dt - idle_s) does what the firmware's position loop does, in its order:

  1. read the sensors: reported current, laser position and accelerometer, each showing the true
     motion some time ago (their delays);
  2. send the command the controller computed in the previous cycle (one-cycle delay);
  3. host position trip on the raw laser position (firmware: 3 cycles outside +-10 mm);
  4. controller step on this cycle's measurement; its output waits for the next cycle;
  5. log the row;
  6. integrate the plant over [t_k, t_k+1] with the drive's coil current (RK4).

Everything inside the loop is plain Python floats: a numpy scalar per tick would make it several
times slower. Noise is generated up front from the scenario's seed, in a fixed order, so two runs that
differ only in an injected signal see exactly the same noise (the loop-gain measurement relies on it).
"""

import math
import time
from dataclasses import asdict, dataclass, field, replace

import numpy as np

from . import references
from .controller import Measurement
from .drive import Drive, DriveConfig
from .plant import PRESETS, DEFAULT_PRESET, PlantModel
from .sensors import Laser, Accelerometer, LaserConfig, AccelConfig

DT_S = 0.0005   # firmware CYCLE_TIME_MS


@dataclass(frozen=True)
class HostConfig:
    """The firmware's safety layers that sit outside the controller."""
    trip_enable: bool = True
    trip_min_mm: float = -10.0       # main.h POS_TRIP_MIN_MM
    trip_max_mm: float = 10.0        # main.h POS_TRIP_MAX_MM
    trip_cycles: int = 3             # main.h POS_TRIP_CYCLES
    stroke_limit_mm: float = 21.0    # simulator guard: mechanical end stops at about +-20-22 mm (true position)


@dataclass(frozen=True)
class Scenario:
    """Everything that defines one run. Immutable: use dataclasses.replace() for variations."""
    controller: type                                   # a Controller subclass
    controller_params: dict = field(default_factory=dict)
    reference: object = references.Constant()
    plant: PlantModel = PRESETS[DEFAULT_PRESET]
    drive: DriveConfig = DriveConfig()
    laser: LaserConfig = LaserConfig()
    accel: AccelConfig = AccelConfig()
    host: HostConfig = HostConfig()
    duration_s: float = 5.0          # active part, from t = 0
    idle_s: float = 0.0              # 0 A window before t = 0 (firmware BIAS_IDLE_S = 3 s)
    seed: int = 0
    substeps: int = 1                # RK4 steps per cycle
    dt_s: float = DT_S
    x0_mm: float = None              # initial position at rest; None = the plant's rest position
    injection_A: np.ndarray = None   # optional signal added to the controller output, one value per cycle

    @property
    def n_ticks(self):
        return int(round((self.idle_s + self.duration_s) / self.dt_s))

    def time(self):
        return np.arange(self.n_ticks) * self.dt_s - self.idle_s

    def settings(self):
        """Complete settings as a plain dict, stored with every result so a run can be reproduced."""
        return {
            "controller": self.controller.describe()["name"],
            "controller_params": self.controller.resolve_params(self.controller_params),
            "reference": references.describe(self.reference),
            "plant": self.plant.describe(),
            "drive": self.drive.describe(), "laser": self.laser.describe(), "accel": self.accel.describe(),
            "host": asdict(self.host),
            "duration_s": self.duration_s, "idle_s": self.idle_s, "seed": self.seed,
            "substeps": self.substeps, "dt_s": self.dt_s, "x0_mm": self.x0_mm,
            "injection": None if self.injection_A is None else f"{len(self.injection_A)} samples, "
                                                               f"rms {np.sqrt(np.mean(np.square(self.injection_A))):.4g} A",
        }


@dataclass
class SimResult:
    settings: dict
    columns: dict                 # name -> np.ndarray, one value per logged cycle
    status: str                   # "ok", "trip", "stroke" or "nonfinite"
    message: str
    warnings: list
    signal_names: tuple           # controller signals, logged as "ctrl_<name>"
    ref_unit: str
    elapsed_s: float

    def __getitem__(self, name):
        return self.columns[name]

    @property
    def t(self):
        return self.columns["t_s"]

    @property
    def ok(self):
        return self.status == "ok"

    def to_frame(self):
        import pandas as pd
        return pd.DataFrame(self.columns)


def _delay_split(delay_s, dt):
    """Delay in whole cycles and the fraction of a cycle left over."""
    d = max(delay_s, 0.0) / dt
    n = int(math.floor(d + 1e-9))
    return n, d - n


def simulate(sc: Scenario, controller=None) -> SimResult:
    """Run one scenario. Pass a controller instance to keep state across runs (e.g. learning control):
    it is reset() first, so only state that reset() leaves alone carries over."""
    t_wall = time.perf_counter()
    dt, n = sc.dt_s, sc.n_ticks
    t = sc.time()

    ctrl = controller if controller is not None else sc.controller(sc.controller_params, dt)
    ctrl.reset()
    ref = sc.reference.evaluate(t)
    if ref.unit != ctrl.REF_UNIT:
        raise ValueError(f"controller {ctrl.NAME} expects a reference in {ctrl.REF_UNIT}, "
                         f"the scenario's reference is in {ref.unit}")

    # --- noise, always drawn in the same order so it does not depend on the other settings ---
    rng = np.random.default_rng(sc.seed)
    laser_white = (rng.standard_normal(n) * sc.laser.noise_mm).tolist()
    ph1, ph3 = rng.uniform(0, 2 * math.pi, 2)
    accel_white = (rng.standard_normal(n) * sc.accel.noise_V).tolist()
    current_noise = (rng.standard_normal(n) * sc.drive.report_noise_A).tolist()

    drive = Drive(sc.drive, current_noise)
    laser = Laser(sc.laser, laser_white, float(ph1), float(ph3), dt, -sc.idle_s)
    accel = Accelerometer(sc.accel, accel_white, dt)
    acc = sc.plant.make_acc()

    inj = [0.0] * n if sc.injection_A is None else np.asarray(sc.injection_A, float).tolist()
    if len(inj) < n:
        inj += [0.0] * (n - len(inj))

    # --- state and history of the true motion at the cycle boundaries (for the sensor delays) ---
    x = (sc.plant.equilibrium_m() if sc.x0_mm is None else sc.x0_mm * 1e-3)
    v = 0.0
    X, V, A = [x], [v], [acc(x, v, 0.0)]
    nl, fl = _delay_split(sc.laser.delay_s, dt)
    na, fa = _delay_split(sc.accel.delay_s, dt)

    r, rd, rdd = ref.r.tolist(), ref.r_dot.tolist(), ref.r_ddot.tolist()
    rf = ref.freq_hz.tolist()
    tl = t.tolist()
    n_prev = int(ctrl.PREVIEW_TICKS)
    n_sig = len(ctrl.SIGNALS)

    cols = {k: [] for k in ("t_s", "position_mm", "accel_V", "actual_current_A", "target_current_A",
                            "u_ctrl_A", "u_cmd_A", "ref", "x_true_mm", "i_true_A")}
    sig_cols = [[] for _ in range(n_sig)]

    host = sc.host
    sub = max(1, int(sc.substeps))
    h = dt / sub
    stroke = host.stroke_limit_mm * 1e-3
    m = Measurement(dt)
    u_next, trip_count = 0.0, 0
    status, message = "ok", ""

    for k in range(n):
        tk = tl[k]

        # 1. sensors (each sees the true motion at t_k minus its delay)
        i_true = drive.current_now()
        i_rep = drive.report(i_true, k)
        j = k - nl
        if fl == 0.0 or j - 1 < 0:
            xd = X[max(j, 0)]
        else:
            # cubic Hermite between the boundaries j-1 and j, at fraction s from j-1
            s = 1.0 - fl
            x0_, x1_, m0, m1 = X[j - 1], X[j], V[j - 1] * dt, V[j] * dt
            s2, s3 = s * s, s * s * s
            xd = (2 * s3 - 3 * s2 + 1) * x0_ + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * x1_ + (s3 - s2) * m1
        pos = laser.read(xd * 1e3, k)
        ja = k - na
        if fa == 0.0 or ja - 1 < 0:
            ad = A[max(ja, 0)]
        else:
            ad = A[ja - 1] + (1.0 - fa) * (A[ja] - A[ja - 1])
        acc_v = accel.read(ad, k)

        # 2. send last cycle's command
        wire = drive.send(u_next)

        # 3. host position trip on the raw laser value (the firmware does not log the tripping row)
        if host.trip_enable:
            if pos < host.trip_min_mm or pos > host.trip_max_mm:
                trip_count += 1
                if trip_count >= host.trip_cycles:
                    status = "trip"
                    message = (f"position trip at t = {tk:.4f} s: {pos:+.3f} mm outside "
                               f"{host.trip_min_mm} .. {host.trip_max_mm} mm for {trip_count} cycles")
                    break
            else:
                trip_count = 0

        # 4. controller
        active = tk >= 0.0
        m.k, m.t_s, m.active = k, tk, active
        m.position_mm, m.accel_V, m.actual_current_A, m.u_applied_prev_A = pos, acc_v, i_rep, wire
        m.ref, m.ref_vel, m.ref_acc, m.ref_freq_hz = r[k], rd[k], rdd[k], rf[k]
        if n_prev:
            m.ref_preview = r[k + 1:k + 1 + n_prev]
        out = ctrl.step(m)
        if isinstance(out, tuple):
            u_c, sig = float(out[0]), out[1]
        else:
            u_c, sig = float(out), ()
        u_next = (u_c if active else 0.0) + inj[k]   # host sends 0 A in the idle window

        # 5. log
        cols["t_s"].append(tk); cols["position_mm"].append(pos); cols["accel_V"].append(acc_v)
        cols["actual_current_A"].append(i_rep); cols["target_current_A"].append(wire)
        cols["u_ctrl_A"].append(u_c); cols["u_cmd_A"].append(u_next); cols["ref"].append(r[k])
        cols["x_true_mm"].append(X[k] * 1e3); cols["i_true_A"].append(i_true)
        for q in range(n_sig):
            sig_cols[q].append(float(sig[q]) if q < len(sig) else math.nan)

        # 6. plant over this cycle; coil current i(s) = level + a1 exp(-s / t1) + a2 exp(-s / t2)
        level, a1, t1, a2, t2 = drive.segment()
        for js in range(sub):
            s0, sm, s1 = js * h, (js + 0.5) * h, (js + 1) * h
            i0 = im = i1 = level
            if t1 != 0.0:
                i0 += a1 * math.exp(-s0 / t1)
                im += a1 * math.exp(-sm / t1)
                i1 += a1 * math.exp(-s1 / t1)
            if t2 != 0.0:
                i0 += a2 * math.exp(-s0 / t2)
                im += a2 * math.exp(-sm / t2)
                i1 += a2 * math.exp(-s1 / t2)
            k1x, k1v = v, acc(x, v, i0)
            k2x, k2v = v + 0.5 * h * k1v, acc(x + 0.5 * h * k1x, v + 0.5 * h * k1v, im)
            k3x, k3v = v + 0.5 * h * k2v, acc(x + 0.5 * h * k2x, v + 0.5 * h * k2v, im)
            k4x, k4v = v + h * k3v, acc(x + h * k3x, v + h * k3v, i1)
            x += h / 6.0 * (k1x + 2 * k2x + 2 * k3x + k4x)
            v += h / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)
        drive.advance(dt)
        if not (math.isfinite(x) and math.isfinite(v)):
            status, message = "nonfinite", f"model diverged (non-finite state) after t = {tk:.4f} s"
            break
        if abs(x) > stroke:
            status = "stroke"
            message = (f"true position {x * 1e3:+.2f} mm passed the {host.stroke_limit_mm} mm stroke guard "
                       f"after t = {tk:.4f} s")
            break
        X.append(x); V.append(v); A.append(acc(x, v, drive.current_now()))

    columns = {k_: np.asarray(v_, float) for k_, v_ in cols.items()}
    for name, vals in zip(ctrl.SIGNALS, sig_cols):
        columns[f"ctrl_{name}"] = np.asarray(vals, float)
    result = SimResult(sc.settings(), columns, status, message, [], tuple(ctrl.SIGNALS), ref.unit,
                       time.perf_counter() - t_wall)
    result.warnings = _warnings(sc, result, ref)
    ctrl.end_of_run(result)
    return result


def _warnings(sc, res, ref):
    """Things the user should know about this run: model validity, saturation."""
    w = []
    if not len(res.t):
        return w
    act = res.t >= 0
    env = sc.plant.envelope
    x = res["x_true_mm"][act]
    if len(x):
        out = (x < env.x_min_mm) | (x > env.x_max_mm)
        if out.any():
            w.append(f"position outside the {sc.plant.key} model's fitted range "
                     f"({env.x_min_mm:+.1f} .. {env.x_max_mm:+.1f} mm) for {100 * out.mean():.1f} % of the run "
                     f"(min {x.min():+.2f}, max {x.max():+.2f} mm): extrapolation")
    f = ref.freq_hz[:len(res.t)][act]
    f = f[np.isfinite(f)]
    if len(f) and ((f < env.f_min_hz) | (f > env.f_max_hz)).any():
        w.append(f"reference frequency {f.min():.1f}-{f.max():.1f} Hz leaves the model's fitted band "
                 f"{env.f_min_hz:g}-{env.f_max_hz:g} Hz")
    u = res["u_cmd_A"][act]
    if len(u) and np.abs(u).max() >= sc.drive.kp_A:
        w.append(f"command reached the drive peak current +-{sc.drive.kp_A:g} A and was clipped")
    if sc.plant.fitted_on_rig and sc.drive.gain_hf == sc.drive.gain_dc and sc.drive.delay_ticks == 0:
        w.append(f"{sc.plant.key} was fitted on the rig's reported current; an ideal drive overstates "
                 "the force above 10 Hz by about 15 % and leaves out about 1.5 ms of delay")
    return w


def variant(sc: Scenario, **changes) -> Scenario:
    """Shorthand for dataclasses.replace on a scenario."""
    return replace(sc, **changes)
