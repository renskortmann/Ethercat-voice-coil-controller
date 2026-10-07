# Simulation tool (`vca_sim`)

`vca_sim` simulates the rig offline, so you can try a control strategy before it goes to the bench.

The simulation has five parts:
- **Plant:** the identified grey-box model, selectable and editable.
- **Drive:** the AMC drive, calibrated on the chirp logs.
- **Sensors:** the laser and the accelerometer.
- **Host:** the firmware's timing and safety chain.
- **Controller:** your controller, as a plug-in script.

The core is plain Python and can be used from notebooks. A browser app sits on top of it.

`vca_sim_validation.ipynb` shows what was checked and what was found.

## The app

There are three ways to start it. Each one starts the app and opens it in your browser at http://127.0.0.1:8050:

- **Explorer:** double-click `run_simulator.bat` in the repository root. Close its window to stop the app.
- **VS Code:** press Ctrl+F5 (Run > Start Without Debugging), with "VCA simulator" selected in the Run and Debug view.
- **Terminal:** run `.venv/Scripts/python -m vca_sim.app` from the repository root.

If the app is already running, starting it again just opens the browser. Use `--port` to pick another port and `--no-browser` to skip opening the browser.

### Settings (left)

- Controller script and its parameters. "Reload scripts" picks up new or edited files in `controllers/`.
- Reference.
- Run length.
- Plant preset. Its values are editable, and any edit is recorded with the run.
- Drive, sensors and host safety.

With "re-run on every change" ticked, each change re-simulates. A 3 s run takes about 0.2 s.

### Results (right)

- **Pin current run:** keeps the run with its own colour and overlays it in every tab. Untick a pinned run to hide it; "Remove unticked" deletes the hidden ones. Up to 7 pinned runs.
- **Time traces:** stacked panels for position, error, current, accelerometer and the controller's signals. Zooming re-thins the traces to the visible window, so individual cycles show at full resolution. The zoom stays when you re-run.
- **Tracking per frequency:** stepped sines across a list of frequencies. It shows amplitude and phase of the true and the laser position, and peak current against the current the mass alone needs and against the drive's peak current.
- **Stability:** loop gain, phase and sensitivity, with a margins table (crossovers, phase/delay/gain margins, modulus margin), for the current run and the ticked pinned runs.
- **Rig logs:** any `voice_coil_log_*.csv` under `data/` or `gcsc_data/`. Plain and scheduled chirp logs are also repeated in the simulator with the current plant, drive and sensors, so you can check the model against the rig.
- **Run settings:** the complete settings stored with each run.
- **Export CSV:** writes the current run in the rig log format.

The app holds runs in memory on the server: it is meant for one person on their own machine, and the runs are lost when it stops.

## Quick start

```python
from vca_sim import *
from vca_sim import references as R, analysis as AN

PI = load_controller("controllers/position_pi.py")
sc = Scenario(PI, {"Kp": 1.0, "Ki": 0.12}, R.Sine(0.0, 0.5, 45.0), PRESETS["greybox_C"], duration_s=2.0)
res = simulate(sc)                        # about 0.3 s per simulated 10 s
res.status, res.warnings, AN.tracking_metrics(res, settle_s=1.0)
res.to_frame()                            # every cycle: position_mm, accel_V, actual_current_A, u_cmd_A, x_true_mm, ...
AN.loop_gain(sc).margins                  # stability margins of the closed loop
```

`Scenario` is immutable. Use `dataclasses.replace(sc, ...)` to make a variant, for example:
- `plant=PRESETS["linear_msd"].with_params(c=197)`
- `drive=IDEAL_DRIVE`
- `laser=NOISE_FREE_LASER`

`result.settings` records everything needed to repeat a run.

## Writing a controller

Put a script in `controllers/` that defines one subclass of `vca_sim.Controller`. `controllers/position_pi.py` is a complete example: it is a copy of the firmware's `pi_update`.

```python
from vca_sim.controller import Controller, Param

class MyController(Controller):
    NAME = "My controller"
    PARAMS = (Param("gain", 1.0, 0.0, 10.0, "A/mm"),)    # the app builds inputs from these
    SIGNALS = ("ff_A",)                                 # extra signals, logged as ctrl_ff_A
    REF_UNIT = "mm"                                     # or "A" for a current reference
    PREVIEW_TICKS = 0                                   # future reference samples, for feedforward

    def reset(self):                                    # clear state; called before every run
        ...

    def step(self, m):                                  # once per 0.5 ms cycle
        u = self.p["gain"] * (m.ref - m.position_mm)
        return u, (0.0,)                                # current in A, sent to the drive next cycle
```

### What `step` receives

`m` carries the same signals the firmware has, in the units of the log columns:
- `position_mm`
- `accel_V` (raw volts, including the bias)
- `actual_current_A`
- `u_applied_prev_A`
- `ref`, `ref_vel`, `ref_acc`, `ref_freq_hz` and `ref_preview`
- `t_s`
- `active`, which is False in the idle window. There the host sends 0 A whatever `step` returns.

### Rules

- **Timing:** the output reaches the drive one cycle later, exactly as on the rig.
- **Safety stays with the host:** the non-finite guard, the clamp to ±KP and the position trip (±10 mm, 3 cycles) are the host's job. A strategy's own output limit belongs in the controller.
- **Keep it C-ready:** the method set maps one-to-one onto `ctrl_init` / `ctrl_reset` / `ctrl_step` / `ctrl_end_of_run`. Keep state in plain numbers and fixed-size arrays.
- **Learning controllers:** for ILC and similar, pass the same instance to `simulate(sc, controller=ctrl)` for each trial. `end_of_run(result)` is called after every run.

## Models and defaults

| Part | Default | Source |
|---|---|---|
| Plant | `greybox_C` (two unequal magnets, Kf(x)) | `vca_greybox_fit.ipynb` cell 17 |
| Drive | KP 60 A; 2 cycles of delay; shelf with gain 1.01 at DC and 0.849 above about 6 Hz, τ 25.6 ms | `calibration.fit_drive` on the 2 Oct plain chirps |
| Laser | 1.15 ms delay; 8 µm steps; 24 µm white noise; 92 µm at 50 Hz and 49 µm at 150 Hz; out of range above +16.44 mm | Chain comparison and idle windows |
| Accelerometer | 1.25 ms delay plus a 78 Hz low-pass; bias 0.655 V; 5.4 mV noise; scale 0.0578 × 0.65 V/g | In-band and greybox notebooks |

The other presets are `greybox_A`, `greybox_B`, `sysid_poly5`, `linear_msd` and `datasheet`. Each preset records its source and the range it was fitted on. A run that leaves that range gets a warning.

## Known limits

- The plant presets are weakest in the application band. They move 10–20 % less than the rig at 38–46 Hz, and the 35.5 Hz laser-mount mode is in none of them.
- Damping estimates range from 197 to 2134 N·s/m. Compare presets, or edit `c`.
- About 5 % of the drive's reported current is unexplained by the drive model.
- The accelerometer scale is unresolved.
- Closed-loop behaviour has not been validated against the rig. Replay the first closed-loop log through the tool.

## Tests

```
.venv/Scripts/python -m pytest tests
```

The tests check the following:
- The greybox NRMSE matches the notebook.
- The integrator passes the notebook's self-check.
- The drive class matches the fitted model.
- The loop gain matches the analytic PI loop.
- The PI claims in `position-control.md` hold.
- Firmware timing, the trip and the export round trip behave as on the rig.
- The app's forms build valid scenarios for every controller and reference, and its callbacks work through Dash's HTTP endpoint.
