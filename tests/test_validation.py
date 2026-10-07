"""Validation against known results: the greybox notebook's numbers, the analytic PI loop, and the
PI behaviour described in docs/position-control.md (origin/position_control)."""

import math
from pathlib import Path

import numpy as np
import pytest

from vca_sim import (PRESETS, Scenario, simulate, DriveConfig, IDEAL_DRIVE, NOISE_FREE_LASER, NOISE_FREE_ACCEL,
                     HostConfig, references as R)
from vca_sim import analysis as AN, replay as RP

ROOT = Path(__file__).resolve().parents[1]
TEST_SCHED = next((ROOT / "data" / "test").glob("voice_coil_log_chirpsched_*.csv"), None)


@pytest.mark.skipif(TEST_SCHED is None, reason="data/test logs not present")
@pytest.mark.parametrize("key, expected", [("greybox_A", (0.003, 0.180, 0.642, 0.594)),
                                           ("greybox_B", (0.003, 0.183, 0.803, 0.783)),
                                           ("greybox_C", (0.003, 0.180, 0.640, 0.587))])
def test_greybox_nrmse_reproduced(key, expected):
    """The presets reproduce vca_greybox_fit.ipynb's printed test NRMSE at k = 1, 10, 100, 1000."""
    d = RP.load_greybox(TEST_SCHED)
    e = RP.rollout_nrmse(PRESETS[key], d, d["sweep"], H=1000)
    got = [e[k] for k in (1, 10, 100, 1000)]
    assert np.allclose(got, expected, atol=0.0015), got


def _linear_scenario(PI, params, drive, plant=None, **kw):
    return Scenario(PI, params, R.Constant(0.0), plant or PRESETS["linear_msd"], drive, NOISE_FREE_LASER,
                    NOISE_FREE_ACCEL, **kw)


@pytest.mark.parametrize("delay_ticks", [0, 2])
def test_loop_gain_matches_analytic(PI, delay_ticks):
    drive = DriveConfig(gain_dc=1.0, gain_hf=1.0, shelf_tau_s=0.0, delay_ticks=delay_ticks,
                        quantise_command=False, quantise_report=False, report_noise_A=0.0)
    sc = _linear_scenario(PI, {"Kp": 1.0, "Ki": 0.12}, drive)
    lg = AN.loop_gain(sc, rms_A=0.05)
    La = AN.msd_pi_loop_gain(lg.f_hz, sc.plant.params, 1.0, 0.12, sc.dt_s, delay_ticks, sc.laser.delay_s)
    assert lg.periodic
    assert np.max(np.abs(lg.L / La - 1)) < 1e-3


def test_pi_mode_near_12_hz(PI):
    """position-control.md: at Kp = 1 the closed loop has a mode near 12 Hz (crossover of L)."""
    lg = AN.loop_gain(_linear_scenario(PI, {"Kp": 1.0, "Ki": 0.12}, IDEAL_DRIVE), rms_A=0.05)
    fc = [c["f_Hz"] for c in lg.margins["crossovers"]]
    assert len(fc) == 1 and 10.5 < fc[0] < 12.5


def _ringing_growth(res):
    """Oscillation amplitude in the last third of the run over that in the first third (after 1 s)."""
    x, t = res["x_true_mm"], res.t
    T = t[-1]
    a = np.ptp(x[(t > 1.0) & (t < 1 + (T - 1) / 3)])
    b = np.ptp(x[t > T - (T - 1) / 3])
    return b / a


@pytest.mark.parametrize("factor, stable", [(0.5, True), (2.0, False)])
def test_pi_ki_stability_limit(PI, factor, stable):
    """position-control.md: Ki < c (k + Kp Kf 1000) / (m Kf 1000), 0.62 A/(mm s) for c = 197 at Kp = 0.02."""
    plant = PRESETS["linear_msd"].with_params(c=197.0)
    p = plant.params
    limit = p["c"] * (p["k1"] + 0.02 * p["kf0"] * 1000) / (p["m"] * p["kf0"] * 1000)
    assert limit == pytest.approx(0.62, abs=0.005)
    sc = _linear_scenario(PI, {"Kp": 0.02, "Ki": factor * limit}, IDEAL_DRIVE, plant, duration_s=25.0,
                          x0_mm=1.0, host=HostConfig(trip_enable=False))
    growth = _ringing_growth(simulate(sc))
    assert (growth < 0.9) if stable else (growth > 1.1), growth


def test_pi_time_constant(PI):
    """position-control.md: at Kp 0.02, Ki 0.2 the loop follows a step with a time constant of about 0.8 s."""
    sc = Scenario(PI, {"Kp": 0.02, "Ki": 0.2}, R.Steps(((0.0, 0.0), (1.0, 1.0))), PRESETS["linear_msd"],
                  IDEAL_DRIVE, NOISE_FREE_LASER, NOISE_FREE_ACCEL, duration_s=6.0)
    res = simulate(sc)
    x, t = res["x_true_mm"], res.t
    t63 = t[np.argmax((t > 1.0) & (x >= 0.632))] - 1.0
    assert 0.6 < t63 < 1.05, t63
