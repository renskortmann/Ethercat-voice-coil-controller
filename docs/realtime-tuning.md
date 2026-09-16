# Real-time deployment tuning

`ethercat-voice-coil-controller` runs a 500 µs DC/SYNC0-synchronized EtherCAT cyclic loop
(`fieldbus_run_cyclic()` in [`control_loop.c`](../control_loop.c); the period is
`CYCLE_TIME_MS` in [`main.h`](../main.h)). The application does what it can on its own —
`mlockall()`, prefaulted buffers, `SCHED_FIFO` + CPU affinity + minimal timer slack around
the loop (see `main()` in [`main.c`](../main.c)) — but real-time behavior on Linux also
depends on host and kernel settings this program cannot set for itself. This doc covers
those, with concrete values for the current target machine.

## Target machine

| | |
|---|---|
| Machine | Dell OptiPlex 7010, BIOS `A29` (06/28/2018 — the final release for this model) |
| CPU | Intel Core i5-3550, 4 cores (0–3), no hyperthreading |
| Kernel | `6.8.1-1059-realtime` (PREEMPT_RT) — already the right kernel, see [Kernel choice](#kernel-choice) |
| EtherCAT NIC | `eno1` (Intel 82579LM, `e1000e`, PCH-integrated `0000:00:19.0`, **private MSI vector**) |
| Internet NIC | `enp1s2` (Intel 82544GC, `e1000`, legacy PCI, shared IRQ 18) — carries IP/DHCP, not the fieldbus |
| Launched as | `sudo ./ethercat-voice-coil-controller eno1` |

The NIC choice is deliberate — see [EtherCAT NIC selection](#ethercat-nic-selection)
below. Do not swap them back.

CPU core assignment:

| core | role |
|---|---|
| 0 | OS housekeeping — EtherCAT NIC IRQ, plus `i915` and `xhci_hcd` |
| **1** | **reserved for the RT cyclic loop** — must match `RT_CPU_CORE` in [`main.h`](../main.h) |
| 2, 3 | OS housekeeping, everything else |

The application pins the cyclic loop to `RT_CPU_CORE` via `sched_setaffinity()`. That only
helps if the kernel also keeps everything else off that core — the rest of this doc is
about making core 1 genuinely idle except for the loop.

## Kernel command line

Isolate core 1, stop its periodic tick, offload its RCU work, keep IRQs off it, and cap
idle latency. Edit `/etc/default/grub` (needs `sudo`) and set:

```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=managed_irq,domain,1 nohz_full=1 rcu_nocbs=1 irqaffinity=0,2,3 intel_idle.max_cstate=1 processor.max_cstate=1 nmi_watchdog=0 skew_tick=1 nosoftlockup tsc=nowatchdog"
```

| token | effect |
|---|---|
| `isolcpus=managed_irq,domain,1` | keep the scheduler and driver-managed IRQs off core 1. Modern syntax — no `isolcpus=1` deprecation warning. The app's explicit `sched_setaffinity()` still runs the loop on core 1. |
| `nohz_full=1` | stop the periodic scheduling-clock tick on core 1 (cores 0/2/3 stay as housekeeping) |
| `rcu_nocbs=1` | run core 1's RCU callbacks on another core |
| `irqaffinity=0,2,3` | default IRQ affinity excludes core 1 |
| `intel_idle.max_cstate=1` + `processor.max_cstate=1` | cap idle at C1. C3/C6 exit latency on this Ivy Bridge part is tens of µs — the single biggest jitter source here. |
| `nmi_watchdog=0` | drop a periodic per-CPU NMI |
| `skew_tick=1` | de-synchronise remaining ticks across cores, avoids lock-contention spikes |
| `nosoftlockup` | disable the soft-lockup detector's per-CPU work |
| `tsc=nowatchdog` | stop the periodic TSC-vs-clocksource watchdog (reads HPET on all cores) |

Deliberately **not** included: `idle=poll` (pins every core at 100%, needless heat) and
`intel_pstate=disable` (the governor below handles frequency).

Apply and reboot:

```bash
sudo nano /etc/default/grub      # edit the line above
sudo update-grub
sudo reboot
```

Verify after reboot:

```bash
cat /proc/cmdline
cat /sys/devices/system/cpu/isolated       # -> 1
cat /sys/devices/system/cpu/nohz_full      # -> 1
```

## CPU frequency scaling and idle states

Idle states are capped by the kernel command line above; confirm with

```bash
ls /sys/devices/system/cpu/cpu1/cpuidle/   # -> only state0 (POLL) and state1 (C1)
```

Frequency scaling still needs the `performance` governor pinned so the core doesn't ramp
down between cycles (`cpupower frequency-set` does not survive a reboot). Turbo is
disabled in the same unit so the core frequency is genuinely constant — it trades a little
headroom for lower jitter. Install both as a service:

```bash
sudo systemctl edit --force --full rt-perf.service
```

```ini
[Unit]
Description=Pin CPU governor to performance for RT
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/bin/cpupower frequency-set -g performance
ExecStart=/bin/sh -c 'echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo'

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now rt-perf.service
cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor   # -> performance
cat /sys/devices/system/cpu/intel_pstate/no_turbo           # -> 1
```

With this active all four cores sit at a flat 3292–3300 MHz.

Deep C-states can also be disabled in BIOS/UEFI for a belt-and-braces guarantee.

## EtherCAT NIC selection

This machine has two Intel NICs. The fieldbus runs on the one with the better host-side
latency path:

| | `eno1` — 82579LM | `enp1s2` — 82544GC |
|---|---|---|
| Driver | `e1000e` | `e1000` |
| Location | PCH LAN-on-motherboard, DMI-integrated | add-in card on legacy 33/66 MHz PCI |
| Interrupt | **private MSI vector** | **legacy IO-APIC line 18, shared with `i801_smbus`** |
| Coalescing control | yes (`rx-usecs`) | none exposed |

`eno1` wins on the two things that matter for PDO jitter: a private MSI vector (no
shared-handler demux on every SMBus transaction) and an integrated DMA path (no legacy-PCI
bus arbitration). `enp1s2` carries internet instead — its shared IRQ is harmless there.

## NIC interrupt affinity

`eno1` has a **private MSI vector**, so unlike `enp1s2` it shares no handler with another
device. The IRQ number is assigned at boot and **changes across BIOS and kernel updates**
(it moved from 26 to 32 with BIOS A29), so always look it up rather than hardcoding it:

```bash
grep eno1 /proc/interrupts                  # -> e.g. "32: ... IR-PCI-MSI-0000:00:19.0 0-edge eno1"
cat /proc/irq/<N>/smp_affinity_list         # must not include core 1
```

`irqaffinity=0,2,3` on the kernel command line already keeps it off the RT core, so the
requirement is met without per-IRQ configuration. To pin it explicitly anyway:

```bash
echo 0 | sudo tee /proc/irq/<N>/smp_affinity_list
```

`irqbalance` is not installed on this machine, so nothing will move it back. If it ever
gets installed, ban core 1 in `/etc/default/irqbalance`:

```
IRQBALANCE_BANNED_CPUS=00000002
```

(bit 1 set = core 1).

SOEM does its packet TX/RX **inline on the loop thread** — there is no separate SOEM RX
thread to prioritise. Under PREEMPT_RT the `e1000e` NAPI poll runs in the context of the
`irq/<N>-eno1` thread, so that thread is the whole host-side RX completion path and
`pdo_exchange_us` measures it end to end.

## NIC RT tuning

[`scripts/setup-ethercat-nic.sh`](../scripts/setup-ethercat-nic.sh) dedicates the interface
to the fieldbus and pins its link/offload settings: unmanages it from NetworkManager (no
DHCP retries), disables IPv6, EEE/LPI, pause frames and interrupt coalescing, turns off the
segmentation/receive offloads (batching adds jitter; TSO on the 82579 family is also tied to
"Detected Hardware Unit Hang"), and fixes the link at 100/full so it can't spend 2–4 s
renegotiating mid-run. `--install-service` persists it across reboots via
`ethercat-nic@.service`.

```bash
sudo ./scripts/setup-ethercat-nic.sh --iface eno1 --install-service
```

**On link speed:** the fieldbus link is 100 Mb/s because **EtherCAT is 100BASE-TX full
duplex only** — the slave controller silicon has no gigabit PHY, so forcing `speed 1000`
simply fails to link, whatever the host NIC advertises. It would also buy nothing: the PDOs
are 12 B out / 14 B in, which hits the 64-byte Ethernet minimum, so a round trip is ~13 µs
of wire time at 100 Mb/s against a measured `pdo_exchange_us` of ~61 µs. The rate is not the
bottleneck. (`enp1s2` negotiating 1000 Mb/s is the internet NIC and unrelated.)

## Firmware / BIOS

BIOS A29 was verified clean for RT use. The check that matters is SMI/SMM latency, which
Linux can neither see nor control:

```bash
sudo hwlatdetect --duration=60s --cpu-list 1
```

A29 result: **max 13 µs, 2 samples over the 10 µs threshold**, both in adjacent windows at
test start and nothing for the remaining ~57 s. That is a light, non-periodic handler and
is negligible against a 500 µs cycle. Re-run this after any future BIOS change — it is the
only way to catch a firmware latency regression.

Note that a BIOS update may reset setup options and renumber MSI vectors. After one,
re-check `/proc/cmdline`, the governor, `no_turbo`, and the NIC IRQ number.

## Memory locking

Already satisfied on this machine: the program runs as root (`sudo`), so `mlockall()`
succeeds, and `/etc/security/limits.d/realtime.conf` grants the `realtime` group
`memlock unlimited`.

To run it **without** `sudo`, grant just the locking capability and make sure the user's
`memlock` limit is high enough:

```bash
sudo setcap cap_ipc_lock,cap_net_raw+ep ./ethercat-voice-coil-controller
ulimit -l                                               # must be large / unlimited
```

`cap_net_raw` is required because SOEM opens a raw socket on the NIC.

## Kernel choice

Already done — the target runs a `PREEMPT_RT` kernel (`uname -r` →
`6.8.1-1059-realtime`). Vanilla Linux with the tuning above still shows occasional
multi-millisecond scheduling-latency spikes under load; `PREEMPT_RT` is what gets
worst-case jitter into the sub-100 µs range this loop needs.

## Measuring

Every exported `data/voice_coil_log_*.csv` carries two instrumentation columns:

| column | meaning |
|---|---|
| `cycle_jitter_us` | signed offset of actual vs. scheduled cycle time |
| `pdo_exchange_us` | time inside `ecx_send_processdata` + `ecx_receive_processdata` |

and the run prints a `missed deadlines: N (max jitter X us)` summary line.

**Interpreting `cycle_jitter_us`:** its mean is around **−435 µs**, which is a fixed phase
offset against SYNC0, not jitter. Only the spread around that mean is jitter. And on this
machine the two columns are almost perfectly correlated — `r(cycle_jitter_us,
pdo_exchange_us) ≈ 0.98–1.00` — so cycle jitter here is essentially *all* EtherCAT frame
round-trip time, not OS scheduling latency. Tune against `pdo_exchange_us`.

### Aggregating across runs

A single 10 s run is **not** enough to judge a tuning change. The body of the distribution
settles quickly but the tail is a handful of rare events, and run-to-run variance there is
large enough to manufacture convincing improvements that do not exist (see
[Tried and rejected](#tried-and-rejected)). Use [`scripts/benchmark-rt.sh`](../scripts/benchmark-rt.sh),
which pools several runs for the percentile table while reporting per-run maxima separately
so tail variance stays visible:

```bash
sudo scripts/benchmark-rt.sh --label baseline --runs 5
# ... apply a tuning change by hand ...
sudo scripts/benchmark-rt.sh --label candidate --runs 5
sudo scripts/benchmark-rt.sh --compare baseline candidate
```

It snapshots the config state (cmdline, governor, IRQ affinities, every IRQ thread's
priority) to `data/benchmark/<label>.meta` so a result can be traced back to what was
actually set. Treat anything under a few µs of separation as noise.

### Current baseline

Pooled over 5 runs / 89,995 cycles with all tuning above active, 500 µs budget:

| metric | value | % of cycle |
|---|---|---|
| `pdo_exchange_us` p50 | 60.7 µs | 12% |
| `pdo_exchange_us` p99 | 67.3 µs | 13% |
| `pdo_exchange_us` p99.9 | 166.7 µs | 33% |
| `pdo_exchange_us` max | 276.4 µs | 55% |
| cycles > 150 µs | 98 / 89,995 (0.11%) | |
| `cycle_jitter_us` spread (p0.1–p99.9) | 112.5 µs | |
| missed deadlines / faults | **0** | |

Zero frame loss over the same runs (`ethtool -S eno1` shows `rx_packets == tx_packets`,
all error counters zero).

### System-level baseline

Independent of the application, on the isolated core with the EtherCAT link active:

```bash
sudo cyclictest -m -p 99 -i 500 -a 1 -t 1 -D 60
```

## Tried and rejected

**Raising the NIC IRQ thread's RT priority — no effect.** The `irq/<N>-eno1` thread runs
`SCHED_FIFO` at the default priority 50, the same as `irq/34-i915` and `irq/26-xhci_hcd`,
which share core 0 with it. Raising it above them looked like an obvious win:

```bash
# NB: pgrep -f self-matches the calling shell; match on the thread name instead
sudo chrt -f -p 85 $(ps -eLo tid,comm | awk '/irq\/.*-eno1/{print $1}')   # no effect
```

Measured over 5 runs each way, every percentile moved by ≲1 µs and the signs disagreed
(p99 67.3 → 68.4, max 276.4 → 262.7, cycles >150 µs 98 → 108). A single-run comparison
initially showed p99 improving 82.5 → 64.8 µs; that was entirely run-to-run variance in the
single baseline run. **Do not re-apply this, and do not trust single-run comparisons.**

The negative result is also evidence: if the spikes below were host-side contention between
these handlers, priority would have moved them.

## Open: the ~100 ms periodic interferer

The residual tail is not random. In every run, cycles with `pdo_exchange_us` above ~150 µs
arrive at gaps that are near-exact multiples of **100 ms**, at a phase that is stable within
a run but **randomly different in each run**:

| run | phase at t≈1 s | phase at t≈9 s |
|---|---|---|
| 133557 | 87.5 ms | 90.5 ms |
| 133610 | 29.0 ms | 28.0 ms |
| 133624 | 6.0 ms | 1.5 ms |
| 133637 | 63.5 ms | 58.0 ms |
| 133651 | 36.0 ms | 32.0 ms |

Amplitudes cluster at discrete levels — roughly 168, 201 and 221 µs — against a 60.7 µs
median.

The re-randomising phase argues **against** a host-side source: a kernel timer, `i915`
vblank or any free-running 10 Hz host task would land at a phase set by the host clock, and
the loop's time base is that same clock, so the phase would repeat across runs. It points
instead at something whose phase is established when the EtherCAT link starts — the drive,
the DC/SYNC0 servo, or a beat between the master's send timing and the drive's internal
update window.

Two experiments would discriminate, neither yet run:

1. **Headless.** `sudo systemctl isolate multi-user.target`, then collect a label. If the
   spikes survive with the GPU and compositor gone, the host is exonerated.
2. **Change the cycle period.** Set `CYCLE_TIME_MS` to `1.0`, rebuild, collect a label. If
   the interferer stays at 100 ms it is an absolute-time ~10 Hz source; if it scales to
   ~200 ms it is a cycle-count artifact (200 cycles) and therefore DC phasing.

This is a margin-understanding question, not a defect: worst case is 55% of the cycle
budget with zero missed deadlines.

## Development environment caveat

The primary dev environment for this project is WSL2 (Hyper-V-virtualized Linux). WSL2
cannot be used to validate any of the above: it doesn't expose raw EtherCAT NIC access in
the way SOEM needs, and Hyper-V's own scheduling adds latency and jitter outside Linux's
control. Real-time behavior — and the `cycle_jitter_us` values in the exported CSV — must
be measured on the actual bare-metal target machine, not in WSL2.
