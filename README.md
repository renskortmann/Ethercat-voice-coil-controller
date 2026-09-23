# ethercat-voice-coil-controller

An EtherCAT master application that runs a hardware-synchronized, sub-millisecond
current-control loop on an **Advanced Motion Controls (AMC)** servo drive to command
a voice-coil motor. Built on [SOEM 2](https://github.com/OpenEtherCATsociety/SOEM).

The program brings a CiA 402 drive up into **Cyclic Synchronous Torque (CST)** mode,
commands a sine-wave current for a fixed duration, monitors the fieldbus for faults,
and logs every cycle to CSV for offline analysis.

## What it does

1. Discovers the EtherCAT network on a given NIC and locates the AMC drive.
2. Configures the drive over SDO during PreOp→SafeOp (CST mode, interpolation period,
   PDO mappings) and reads its peak-current rating for current scaling.
3. Walks the CiA 402 state machine to **Operation Enabled**.
4. Enters real-time mode (memory locked, pinned to an isolated core, `SCHED_FIFO`)
   and runs the DC-SYNC0-synchronized cyclic loop:
   - generates the target current for the selected experiment (`EXPERIMENT_MODE`): a
     sine wave, or a step-release (hold a constant current, then release to zero)
   - exchanges process data every `CYCLE_TIME_MS`
   - validates the working counter and the drive state every cycle
   - records per-cycle timing jitter and PDO round-trip time
5. Shuts the drive down cleanly and exports samples + faults to `data/*.csv`.

All blocking I/O is kept out of the cyclic loop; diagnostics and CSV export happen
after it returns. See [docs/rt-implementation.md](docs/rt-implementation.md).

## Requirements

- Linux, ideally a `PREEMPT_RT` kernel (developed on 6.8 RT)
- [SOEM 2](https://github.com/OpenEtherCATsociety/SOEM) installed (headers under
  `soem/` and a linkable `soem` library)
- CMake ≥ 3.28, a C compiler, `libm`
- Root (or `CAP_NET_RAW` + `CAP_IPC_LOCK` + `CAP_SYS_NICE`) to open a raw socket,
  lock memory, and use real-time scheduling
- Python 3 with `matplotlib` for the plotting script (optional)

## Build

```bash
cmake -B build
cmake --build build
```

This produces the `ethercat-voice-coil-controller` executable in the project root
(build artifacts stay under `build/`). If SOEM 2 is installed in a non-standard
prefix, point CMake at it with `cmake -B build -DCMAKE_PREFIX_PATH=<install-dir>`.

## Run

```bash
sudo ./ethercat-voice-coil-controller <IFNAME>      # e.g. enp2s0
```

Run with no arguments to list available network interfaces. Output CSVs are written
to `data/`:

- `voice_coil_log_<experiment>_YYYYMMDD_HHMMSS.csv` — per-cycle samples (currents, analog inputs,
  `cycle_jitter_us`, `pdo_exchange_us`, and `position_mm`: shaft displacement from centre computed
  in the firmware from the AI1 laser voltage using the `AI1_*` calibration constants in `main.h`)
- `voice_coil_faults_<experiment>_YYYYMMDD_HHMMSS.csv` — fault events with recovery action

`<experiment>` is the compile-time experiment mode and its parameters (`EXPERIMENT_TAG` in
`main.h`), so a directory listing shows what each run was. Examples:

- `voice_coil_log_sine_15.0Hz_5.0A_20260923_132459.csv`
- `voice_coil_log_step_release_hold6.0A_ramp0.2s_dur3.0s_20260923_131834.csv`
- `voice_coil_log_chirp_3.0A_1.0to60.0Hz_30.0s_20260923_140212.csv`

## Configuration

Runtime parameters are compile-time constants in [main.h](main.h):

| Constant | Default | Meaning |
|---|---|---|
| `CYCLE_TIME_MS` | 0.5 | EtherCAT cycle period |
| `RUN_DURATION_S` | 10.0 | Total run time |
| `EXPERIMENT_MODE` | `EXPERIMENT_CHIRP` | Which setpoint profile the loop commands (see below) |
| `SINE_FREQ_HZ` | 10.0 | Sine experiment: target current waveform frequency |
| `SINE_AMPLITUDE_A` | 2.0 | Sine experiment: target current waveform amplitude |
| `HOLD_CURRENT_A` | 1.0 | Step-release experiment: constant current during the hold phase |
| `HOLD_RAMP_S` | 0.2 | Step-release experiment: linear ramp-in time at the start of the hold (0 = hard step) |
| `HOLD_DURATION_S` | 3.0 | Step-release experiment: time from loop start to release (must be < `RUN_DURATION_S`) |
| `CHIRP_AMPLITUDE_A` | 1.0 | Chirp experiment: current amplitude |
| `CHIRP_F0_HZ` | 1.0 | Chirp experiment: start frequency (> 0) |
| `CHIRP_F1_HZ` | 100.0 | Chirp experiment: end frequency (at least 10 samples per period) |
| `CHIRP_DURATION_S` | 8.0 | Chirp experiment: sweep length (must be <= `RUN_DURATION_S`) |
| `RT_CPU_CORE` | 1 | Isolated core for the cyclic loop |

Three experiments are available, selected at compile time with `EXPERIMENT_MODE`:

- `EXPERIMENT_SINE` — feedforward sine current at `SINE_FREQ_HZ` / `SINE_AMPLITUDE_A`.
- `EXPERIMENT_STEP_RELEASE` — ramp to `HOLD_CURRENT_A` over `HOLD_RAMP_S`, hold it so the
  coil settles at a fixed deflection, then at `HOLD_DURATION_S` command zero current and record
  the free (autonomous) mechanical response on the accelerometer input for the rest of the run.
  The drive stays in Operation Enabled and regulates coil current to zero, so no motor force
  acts during the ring-down. The release instant is `HOLD_DURATION_S` in the CSV `timestamp_s`
  column.
- `EXPERIMENT_CHIRP` — exponential sine sweep for system identification: amplitude
  `CHIRP_AMPLITUDE_A`, frequency f(t) = f0 · (f1/f0)^(t/T) from `CHIRP_F0_HZ` to `CHIRP_F1_HZ`
  over `CHIRP_DURATION_S`, then 0 A for the rest of the run. f(t) can be rebuilt offline from
  these constants and the CSV `timestamp_s` column.

## Real-time setup

For predictable sub-millisecond latency the host needs kernel and NIC tuning
(core isolation, C-state limits, IRQ affinity, NIC offload/coalescing off).
Details and measured results are in:

- [docs/realtime-tuning.md](docs/realtime-tuning.md) — host tuning guide: kernel command line,
  C-states, NIC selection/affinity, firmware checks, and the measured baseline
- [docs/rt-implementation.md](docs/rt-implementation.md) — design record for the application-level
  RT work: what the code does to keep the loop deterministic, and why
- [scripts/setup-ethercat-nic.sh](scripts/setup-ethercat-nic.sh) — dedicates and tunes the fieldbus NIC
- [scripts/benchmark-rt.sh](scripts/benchmark-rt.sh) — runs the controller N times under a named
  config and aggregates jitter/PDO percentiles, for comparing tuning changes

## Plotting

```bash
python scripts/plot_voice_coil_log.py [path/to/log.csv]
```

With no argument it plots the newest log in `data/`: actual/target/demand current
and the two analog inputs against a shared time axis.

## Source layout

| File | Responsibility |
|---|---|
| `main.c` | `main()` — setup, RT mode entry/exit, shutdown |
| `fieldbus.c` | EtherCAT lifecycle: init, discovery, state management |
| `amc_config.c` | AMC drive SDO configuration (PO2SOconfig hook) |
| `cia402.c` | CiA 402 state-machine bring-up |
| `control_loop.c` | Real-time cyclic loop: waveform, PDO exchange, fault + timing monitoring |
| `diagnostics.c` | Fault decoding and post-fault SDO diagnostics |
| `logging.c` | In-memory sample/fault buffers and CSV export |
| `main.h` | Shared types, PDO layouts, object indices, configuration constants |

## Reference documentation

`docs/` also holds vendor and library reference material:

- `AMC_CommManual_EtherCAT.pdf`, `AMC_Datasheet_DPEANIU-060A800.pdf` — AMC drive manual and datasheet
- `SOEM2_API.txt`, `SOEM2_Tutorial.txt`, `SOEM2_Glossary.txt` — SOEM 2 API notes
