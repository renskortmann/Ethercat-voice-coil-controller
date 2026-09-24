# Plan: desktop GUI for the voice-coil controller (branch `feature/gui`)

## Context
The controller is a root/`SCHED_FIFO` C program whose run parameters are compile-time
`#define`s in `main.h` and whose results are only visible afterwards via
`scripts/plot_voice_coil_log.py`. We want a desktop GUI on the controller PC (it has a display)
to set parameters, start/stop runs, watch progress and view results — without putting any GUI
code in, or near, the real-time loop.

Architecture: GUI = separate Python process (PySide6 + pyqtgraph, LGPL/MIT — fine for in-house
use) that launches the controller binary as a subprocess, passes parameters as CLI args, reads
machine-readable status from its stdout, stops it with SIGINT, and loads the resulting CSV.
Live streaming (ring buffer + socket) is deliberately **out of scope** here — it's a follow-up
once this works.

## Part 1 — Controller: runtime configuration + machine-readable status (C)

1. **Run config struct** in `main.h`:
   `typedef struct { double cycle_time_ms, sine_freq_hz, sine_amplitude_a, run_duration_s; int rt_cpu_core; } run_config_t;`
   Add `run_config_t config;` to `Fieldbus`. Keep the existing `#define`s, renamed/used as
   *defaults* (`DEFAULT_CYCLE_TIME_MS`, …). Replace `MAX_SAMPLES` with a computed capacity
   stored in `Fieldbus` (e.g. `int max_samples`), used by `log_sample()` (`logging.c:22`).
2. **CLI parsing** in `main.c` with `getopt_long`:
   `ethercat-voice-coil-controller [--freq HZ] [--amplitude A] [--duration S] [--cycle-ms MS] [--core N] [--json] IFNAME`.
   Validate ranges (positive values, duration cap so the prefaulted buffer stays reasonable,
   cycle time within what `amc_config.c` can encode). Amplitude check against `kp_amps` after
   `fieldbus_start()` (currents already saturate in `control_loop.c`, but reject > kp up front).
   Keep the no-args adapter listing.
3. **Use the config everywhere the defines were used:**
   - `control_loop.c`: `cycle_ns`, loop condition, sine phase/amplitude, `elapsed_s`, start message.
   - `main.c`: `RT_CPU_CORE` affinity block, buffer allocation size.
   - `amc_config.c:38` (`cycle_s = CYCLE_TIME_MS / 1000.0`): `amc_slave_config()` is a SOEM
     PO2SO hook with a fixed `(context, slave)` signature, so pass the cycle time via a small
     setter (`amc_set_cycle_time_ms()`) storing a file-static, called before `fieldbus_start()`.
4. **Clean stop on SIGINT/SIGTERM:** handler sets a `volatile sig_atomic_t stop_requested`;
   the loop condition in `fieldbus_run_cyclic()` checks it (one load per cycle — RT-safe), so
   Stop/Ctrl-C goes through the existing disable-voltage → `fieldbus_stop()` → `export_csv()`
   path instead of killing the drive mid-run. Install with `sigaction` (no `SA_RESTART` needed;
   `clock_nanosleep` EINTR just ends the sleep early, loop continues/exits normally).
5. **`--json` status events:** one JSON object per line on stdout, emitted only from `main.c`
   at phase boundaries (never inside the cyclic loop, never mid-line — `cia402.c` prints
   partial lines). Events: `start` (echo of config), `phase` (`scanning`, `op_enabled`,
   `running`, `shutdown`), `result` (sample/fault counts, missed deadlines, max jitter, CSV
   paths), `error` (message). Needs `export_csv()` / `fieldbus_run_cyclic()` to hand back the
   CSV paths and summary stats (small result struct) rather than only printing them.
   Call `setvbuf(stdout, NULL, _IOLBF, 0)` so piped output isn't block-buffered.
   Human-readable printf output stays unchanged; the GUI shows non-JSON lines in a log pane.
6. **Capabilities instead of sudo:** add `scripts/setcap-controller.sh`
   (`setcap cap_net_raw,cap_ipc_lock,cap_sys_nice+ep ./ethercat-voice-coil-controller`),
   documented in README — must be re-run after every rebuild. This lets the GUI run as the
   normal user and signal the controller.

## Part 2 — GUI (Python, new `gui/` directory)

- `gui/requirements.txt`: `PySide6`, `pyqtgraph`, `numpy`. Run with `python -m gui` from the
  existing `.venv`.
- `gui/controller.py`: `ControllerProcess` wrapping `QProcess` — builds the argv, parses JSON
  lines into Qt signals (`phase`, `result`, `error`), forwards other lines as log text,
  `stop()` sends SIGINT, falls back to terminate after a timeout.
- `gui/log_data.py`: CSV loading + derived signals **moved out of**
  `scripts/plot_voice_coil_log.py` (column aliases, accelerometer V→g calibration, low-pass
  filter, newest-log lookup). The matplotlib script is refactored to import from here so both
  stay in sync.
- `gui/main_window.py`:
  - left panel: interface dropdown (populated by running the binary with no args and parsing
    the adapter list), spin boxes for freq/amplitude/duration/cycle time/core (defaults from the
    C defaults, limits matching the C validation), Run / Stop buttons, phase indicator,
    progress bar (elapsed vs duration, driven by a timer after the `running` event).
  - right: pyqtgraph stacked plots with linked X axes mirroring the script's `AXES` layout
    (current + bus voltage, ai1, ai2 g + filtered, power/energy) with show/hide checkboxes;
    a jitter/PDO-exchange plot; a fault table from the faults CSV; controller log pane.
  - "Open log…" to view any past CSV from `data/`.
- `gui/__main__.py`: entry point.

## Critical files
`main.h`, `main.c`, `control_loop.c`, `logging.c`, `amc_config.c`, `README.md`,
`scripts/plot_voice_coil_log.py`, new `gui/*`, new `scripts/setcap-controller.sh`.

## Verification
1. `cmake -B build && cmake --build build` — no warnings introduced.
2. No-args run still lists adapters; `--help`/bad values give clear errors.
3. On the RT PC: run with defaults and compare to a pre-change run via
   `scripts/benchmark-rt.sh` — jitter/missed-deadline numbers must not regress.
4. Run with non-default `--freq/--amplitude/--duration` and check the CSV (sine frequency,
   amplitude, row count ≈ duration / cycle).
5. Ctrl-C mid-run → drive disabled cleanly, CSVs written, `result` event printed.
6. `--json` output: every JSON line parses (`… | grep '^{' | python -m json.tool --json-lines`).
7. GUI without hardware: point `ControllerProcess` at a tiny fake script that emits the JSON
   events, and open an existing `data/*.csv` — plots match `plot_voice_coil_log.py`.
8. GUI on the RT PC as a normal user (after setcap): full run, Stop mid-run, results load.

## Follow-up (separate change)
Live plots: SPSC lock-free ring buffer written by the RT loop, drained by a normal-priority
publisher thread on a non-isolated core, decimated to ~100–200 Hz and sent over a Unix socket.
