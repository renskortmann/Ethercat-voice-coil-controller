"""Unit checks of the simulator core: plant presets, integrator, drive, timing, controllers, export."""

import math

import numpy as np
import pytest

from vca_sim import (PRESETS, Scenario, simulate, DriveConfig, IDEAL_DRIVE, NOISE_FREE_LASER, NOISE_FREE_ACCEL,
                     HostConfig, Controller, Param, discover, references as R)
from vca_sim import calibration as C, replay as RP
from vca_sim.drive import Drive


def test_presets_build_and_rest():
    for key, p in PRESETS.items():
        ss = p.small_signal()
        assert abs(ss["x0_mm"]) < 1.0, key
        assert 3.5 < ss["f_n_Hz"] < 6.0, key


def test_with_params_records_changes():
    p = PRESETS["linear_msd"].with_params(c=197.0)
    assert p.params["c"] == 197.0 and p.modified == ("c",)
    assert PRESETS["linear_msd"].params["c"] == 1160.0          # the preset itself is untouched
    with pytest.raises(ValueError):
        PRESETS["linear_msd"].with_params(nonsense=1.0)


def test_integrator_undamped_spring(OL):
    """Notebook cell 1 self-check: undamped spring released from 1 mm follows cos(w t)."""
    k = 1e5
    plant = PRESETS["linear_msd"].with_params(c=0.0, k1=k)
    sc = Scenario(OL, reference=R.Constant(0.0, unit="A"), plant=plant, drive=IDEAL_DRIVE, laser=NOISE_FREE_LASER,
                  accel=NOISE_FREE_ACCEL, host=HostConfig(trip_enable=False), duration_s=0.45, x0_mm=1.0)
    res = simulate(sc)
    w = math.sqrt(k / plant.params["m"])
    assert np.allclose(res["x_true_mm"], np.cos(w * res.t), atol=1e-7)


def test_drive_class_matches_vectorised_model():
    cfg = DriveConfig(gain_dc=1.01, gain_hf=0.85, shelf_tau_s=0.025, lag_s=3e-4, delay_ticks=2,
                      quantise_command=False, quantise_report=False, report_noise_A=0.0)
    cmd = np.random.default_rng(1).standard_normal(2000)
    d, rep = Drive(cfg, [0.0] * len(cmd)), []
    for k, c in enumerate(cmd):
        rep.append(d.report(d.current_now(), k))
        d.send(c)
        d.advance(5e-4)
    assert np.abs(np.array(rep) - C.drive_report(cmd, 1.01, 0.85, 0.025, 2, 3e-4)).max() < 1e-12


def test_command_conversion_like_firmware():
    d = Drive(DriveConfig(), [0.0])
    step = 60.0 / 32768
    assert d.to_wire(float("nan")) == 0.0
    assert d.to_wire(1e9) == pytest.approx(32767 * step)       # symmetric clamp, no int16 wrap
    assert d.to_wire(-1e9) == pytest.approx(-32767 * step)
    assert d.to_wire(1.0) == pytest.approx(round(1.0 / step) * step)


def test_open_loop_sends_in_the_same_cycle(OL):
    """Firmware current mode sends experiment_target_current_A(t_k) in cycle k; the playback controller
    uses one sample of preview to undo the engine's one-cycle delay."""
    sc = Scenario(OL, reference=R.Sine(0.0, 2.0, 40.0, unit="A"), drive=IDEAL_DRIVE, duration_s=0.1)
    res = simulate(sc)
    assert np.allclose(res["target_current_A"][1:], res["ref"][1:], atol=1e-12)


def test_controller_discovery_and_params(root, PI):
    found = discover(root / "controllers")
    assert set(found) >= {"Position PI (firmware)", "Open-loop current"}
    with pytest.raises(ValueError):
        PI({"Kp": -1.0})
    with pytest.raises(ValueError):
        PI({"Kq": 1.0})


def test_reference_unit_mismatch_is_refused(PI):
    with pytest.raises(ValueError):
        simulate(Scenario(PI, reference=R.Constant(0.0, unit="A"), duration_s=0.01))


def test_same_seed_same_run_and_noise_independent_of_injection(PI):
    sc = Scenario(PI, reference=R.Constant(0.0), duration_s=0.2, seed=3)
    a, b = simulate(sc), simulate(sc)
    assert np.array_equal(a["position_mm"], b["position_mm"])
    zero = simulate(Scenario(PI, reference=R.Constant(0.0), duration_s=0.2, seed=3, injection_A=np.zeros(400)))
    assert np.array_equal(a["position_mm"], zero["position_mm"])


def test_position_trip(PI):
    """The host trips after 3 cycles outside +-10 mm, as the firmware does."""
    sc = Scenario(PI, {"Kp": 1.0, "Ki": 5.0}, R.Steps(((0.0, 0.0), (0.05, 15.0))), duration_s=2.0)
    res = simulate(sc)
    assert res.status == "trip"
    # cycles 1 and 2 outside the window are logged; the third trips and is not logged (as in the firmware)
    pos = res["position_mm"]
    assert (pos[-2:] > 10.0).all() and pos[-3] <= 10.0


def test_stroke_guard_and_envelope_warning(OL):
    sc = Scenario(OL, reference=R.Constant(8.0, unit="A"), plant=PRESETS["greybox_A"], duration_s=1.0,
                  host=HostConfig(trip_enable=False))
    res = simulate(sc)
    assert res.status == "stroke"
    sc = Scenario(OL, reference=R.Constant(2.5, unit="A"), plant=PRESETS["greybox_C"], duration_s=0.5,
                  host=HostConfig(trip_enable=False))
    assert any("fitted range" in w for w in simulate(sc).warnings)


def test_rig_csv_round_trip(tmp_path, PI):
    import sys
    from vca_log import load_log
    res = simulate(Scenario(PI, reference=R.Sine(0.0, 1.0, 5.0), duration_s=0.5, idle_s=0.2))
    path = RP.write_rig_csv(res, tmp_path / "voice_coil_log_sim_test.csv")
    lg = load_log(path)
    assert len(lg.run) == int(np.sum(res.t >= 0)) and len(lg.idle) == int(np.sum(res.t < 0))
    assert np.allclose(lg.run["position_mm"], res["position_mm"][res.t >= 0], atol=1e-4)
    raw = np.loadtxt(path, delimiter=",", skiprows=2)            # how vca_greybox_fit.load reads logs
    assert raw.shape[0] == len(res.t) and raw[0, 0] == pytest.approx(res.t[0])


def test_reference_from_log_names():
    ch = RP.reference_from_log_name("voice_coil_log_chirp_6.5A_1.0to55.0Hz_120.0s_20261002_093641.csv")
    assert (ch.amplitude, ch.f0_hz, ch.f1_hz, ch.duration_s, ch.stop) == (6.5, 1.0, 55.0, 120.0, "hard")
    sch = RP.reference_from_log_name(
        "voice_coil_log_chirpsched_0.0s2.0A-30.0s6.0A-60.0s15.0A-r30.0s_10.0to55.0Hz_120.0s_20261002_102726.csv")
    assert sch.schedule == ((0.0, 2.0), (30.0, 6.0), (60.0, 15.0)) and sch.ramp_s == 30.0
