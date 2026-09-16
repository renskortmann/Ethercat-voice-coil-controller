# Real-time Implementation

This document is the design record for the **application-level** real-time work in
`ethercat-voice-coil-controller`: what the code does to keep the cyclic loop deterministic,
and why. Host and kernel configuration — core isolation, C-states, IRQ affinity, NIC tuning,
and the measured baseline — lives in [realtime-tuning.md](realtime-tuning.md).

## Application code changes

### Deadline-critical cyclic loop isolation

The real-time scheduling and CPU affinity are applied in a narrow window around the cyclic loop only, not the entire program lifetime. This keeps the system from starving non-RT work during unbounded SDO configuration and CiA402 bring-up phases.

**File: `main.c` `main()`**

- `mlockall(MCL_CURRENT | MCL_FUTURE)` is called early to lock all process memory, preventing page faults during the loop.
- Buffers (`samples`, `faults`) are pre-allocated and prefaulted via `memset()` before entering RT mode, so the first write to each page doesn't trigger a fault.
- Immediately before `fieldbus_run_cyclic()`:
  - `sched_setaffinity()` pins the loop to `RT_CPU_CORE` (core 1 by default).
  - `sched_setscheduler()` raises scheduling to `SCHED_FIFO` at maximum priority.
  - `prctl(PR_SET_TIMERSLACK, 1)` minimizes timer coalescing.
- Immediately after the loop returns, normal scheduling and affinity are restored so file I/O and CSV export don't block the whole system.

### Blocking I/O removed from the critical path

`printf()` and `fprintf()` calls block on I/O syscalls and add unbounded latency inside the loop. All such calls have been moved out:

**File: `control_loop.c` `fieldbus_run_cyclic()`**

- Error detection (WKC errors, state drift) now only records flags and logs to the in-memory fault buffer.
- Error messages are deferred and printed by `export_csv()` after the loop completes.
- Per-cycle timing jitter is measured and recorded in every sample, not printed live.

**File: `logging.c` `log_fault()` and `export_csv()`**

- `log_fault()` only writes to the in-memory buffer; no console output.
- `export_csv()` iterates the fault buffer and prints all events to console and CSV after the loop is done, preserving the same user-facing output at a non-time-critical point.

### Jitter instrumentation

Every sample now captures the signed offset between actual and scheduled cycle completion time:

**File: `main.h`**

- `sample_log_entry_t` gains `cycle_jitter_us` (offset in microseconds; negative = early / ahead of deadline, positive = late) and `pdo_exchange_us` (see next section).

**File: `control_loop.c` `fieldbus_run_cyclic()`**

- `cycle_jitter_us` is computed after every PDO exchange and before the next sleep, via the
  `timespec_diff_us()` helper: `timespec_diff_us(&now, &next_cycle)`.
- Cycles past the deadline (jitter > 0) increment a missed-deadline counter.
- Max jitter is tracked and reported in a summary line after the loop.

**File: `logging.c` `export_csv()`**

- CSV output includes `cycle_jitter_us` (column 9) and `pdo_exchange_us` (column 10).
- Summary line prints: `missed deadlines: N (max jitter X us, max PDO exchange Y us)`.

### Split timing: PDO exchange vs. loop thread

`cycle_jitter_us` alone can't say *where* a slow cycle spent its time. `pdo_exchange_us`
measures only `ecx_send_processdata()` + `ecx_receive_processdata()` — the frame
round-trip (NIC driver, wire, slave response, housekeeping-core IRQ servicing). Comparing
the two localizes a spike:

| `pdo_exchange_us` spike | `cycle_jitter_us` spike | Interpretation |
|---|---|---|
| yes | yes (matching) | Bus / slave / NIC-driver / housekeeping-core IRQ latency — points *outward* |
| no | yes | The loop thread itself was stalled — IPI, SMI, preemption, cache/memory contention |

Compute time per cycle = `(CYCLE_TIME_MS * 1000 + cycle_jitter_us) - pdo_exchange_us`.

**File: `control_loop.c` `fieldbus_run_cyclic()`**

```c
clock_gettime(CLOCK_MONOTONIC, &pdo_start);
ecx_send_processdata(context);
wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
clock_gettime(CLOCK_MONOTONIC, &pdo_end);
pdo_exchange_us = timespec_diff_us(&pdo_end, &pdo_start);
```

### Cycle 0 deadline fix

The first cycle's jitter measurement was biased: `next_cycle` was initialized to loop-start time, so cycle 0 measured the PDO round-trip duration against the start instant, not a real deadline. Fixed by advancing `next_cycle` by one cycle before the loop starts.

**File: `control_loop.c` line ~67**

```c
clock_gettime(CLOCK_MONOTONIC, &next_cycle);
add_timespec(&next_cycle, cycle_ns / 1000);  /* Advance to first real deadline */
```

Now cycle 0 is measured against `start + CYCLE_TIME_MS` like all other cycles, and the spurious "missed deadline" flag disappears.

## Validation and deployment

### On this machine

```bash
cmake --build build
sudo ./ethercat-voice-coil-controller eno1
tail -1 data/voice_coil_log_*.csv
# Check: missed deadlines: 0; cycle_jitter_us all negative with margin;
#        pdo_exchange_us small and stable (a spike here = bus/slave, not the loop)
```

### On another target machine

1. Edit [`main.h`](../main.h) to set `RT_CPU_CORE` to the desired isolated core.
2. Follow the kernel command-line tuning in [realtime-tuning.md](realtime-tuning.md) for that machine's CPU topology.
3. Pick the EtherCAT NIC by interrupt path — see
   [EtherCAT NIC selection](realtime-tuning.md#ethercat-nic-selection) — then run
   `sudo ./scripts/setup-ethercat-nic.sh --iface <IFACE> --install-service`.
4. Build and run with the tuned kernel.
5. Validate using the `cycle_jitter_us` CSV column and the summary line, and establish a
   baseline with `sudo scripts/benchmark-rt.sh --label baseline --runs 5`.

For a system-level baseline independent of the application:

```bash
sudo apt install rt-tests
sudo cyclictest -m -p 99 -i 500 -a <RT_CPU_CORE> -t 1 -D 60
```

## References

- **Jitter field:** [`sample_log_entry_t`](../main.h) in `main.h`
- **Loop implementation:** [`fieldbus_run_cyclic()`](../control_loop.c) in `control_loop.c`
- **RT mode setup:** [`main()`](../main.c) in `main.c`
- **System tuning guide:** [realtime-tuning.md](realtime-tuning.md)
- **Kernel boot params:** `/etc/default/grub` `GRUB_CMDLINE_LINUX_DEFAULT`
- **Governor persistence:** `/etc/systemd/system/rt-perf.service`
- **NIC dedication + tuning:** [`scripts/setup-ethercat-nic.sh`](../scripts/setup-ethercat-nic.sh), persisted via `ethercat-nic@.service`
