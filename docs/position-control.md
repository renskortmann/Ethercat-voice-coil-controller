# Position control: how the loop works

In the `EXPERIMENT_POSITION_PID` mode the PC closes a position loop around the voice coil. You set a position reference in mm. Every 0.5 ms the PC:
1. reads the laser position;
2. compares it with the reference;
3. computes a coil current with a PI controller;
4. sends that current to the drive.

The drive only regulates current, as in every other experiment. Position control happens entirely on the PC.

This document describes the system in terms of signals, timing and calculations. The settings are listed at the end.

## 1. The loop in one picture

```
                        e [mm]          u [A]            i [A]             F [N]          x [mm]
 r(t) [mm] ──►(+)──────────► PI ──► clamp ±10 A ──► drive current ──► coil ──► mass + spring ──┬──►
 reference    ▲ −                                    loop (fast)      K_f       + damping       │
 generator    │                                                       ≈250 N/A                  │
              │                                                                                 │
              │  y [mm]           AI1 [V]                                                       │
              └──── V → mm  ◄──── drive analog ◄──── laser (ILD1220, 4–20 mA) ◄────────────────┘
                  conversion       input 1
```

| Signal | Meaning | Unit |
|---|---|---|
| r | position reference, from the compile-time profile | mm |
| y | measured position, from the laser | mm |
| e = r − y | position error | mm |
| u | controller output = commanded coil current | A |
| i | actual coil current, made by the drive's own current loop | A |
| F = K_f · i | force on the mover | N |
| x | true position of the mover | mm |

**Sign check.** Positive current moves the shaft towards positive `position_mm`: +3 A gave about +13.5 mm in the 23 Sep step test. So if the shaft is below the reference (e > 0), the controller pushes positive current, which moves it up. That is negative feedback, as it should be.

## 2. What each block does

### Reference generator
The reference is fixed when you compile. It is an **absolute** position in the `position_mm` frame, where 0 mm is the calibrated centre (`AI1_CENTRE_MM`). It is *not* an offset from wherever the shaft happens to rest. There are two shapes:
- **Steps** (`POS_REF_SHAPE_STEPS`): a list of (time, position) breakpoints.
  - At each breakpoint the reference moves linearly to the new value over `POS_REF_RAMP_S` (1 s). Set that to 0 for a hard step.
  - After the last breakpoint the value is held until the end of the run.
  - One breakpoint gives a constant setpoint.
- **Sine** (`POS_REF_SHAPE_SINE`): offset + amplitude · sin(2π f t).

### PI controller
Every cycle, from t = 0 on, with Δt = 0.5 ms:

```
e  = r − y
P  = Kp · e                       Kp = 0.02 A/mm
I  = I + Ki · e · Δt              Ki = 0.2 A/(mm·s)
u  = clamp(P + I, −10 A, +10 A)
```

- **What P does.** One mm of error gives 20 mA, which is 5 N of force. The magnetic spring is about 35 N/mm, so 5 N moves the shaft only about 0.14 mm. On its own, P corrects only about 12 % of an error. It adds a little extra stiffness and damping, nothing more.
- **What I does.** I does the actual tracking. As long as any error remains, the integrator keeps building current: 1 mm of error held for 1 s adds 0.2 A. It stops only when the error is zero. That is why the steady-state error goes to zero even though Kp is tiny.
- **Anti-windup.** When P + I would exceed ±10 A, the output is clamped. Without protection the integrator would keep growing during the clamp and then overshoot badly once the error reverses. The controller therefore skips any integrator step that would push further into saturation, and also limits I itself to ±10 A. With the default profile the output stays below 0.5 A, so this only matters if something goes wrong.
- **No D term (yet).** The laser signal has noise and 50 Hz mains pickup on it. Differentiating it would turn that into current noise. A later D term should act on the measured position, not on the error, so that reference steps don't cause a kick. It also needs a low-pass filter.

### Drive and plant
- **Drive.** The drive (AMC, CST mode) has its own fast current loop. Here it is treated as ideal: the actual current equals the command. Measured: a 3.0 A command gives 2.995 A.
- **Mover.** The plant is the moving mass m = 51.5 kg on the magnetic end springs, with
  - stiffness k ≈ 35–40 kN/m near the centre, which stiffens towards the ends;
  - damping c;
  - force constant K_f ≈ 250 N/A.
- **Consequences:**
  - The static gain is K_f / k ≈ **6–7 mm per A**.
  - The mechanical resonance is at **≈ 4.3 Hz**.

## 3. Timing of one 0.5 ms cycle

The PC runs one real-time loop on a dedicated CPU core, woken every 0.5 ms by an absolute timer. In position mode one cycle k does this, in order:

```
 time ──►
 deadline k                                                           deadline k+1
   │                                                                      │
   ├─ 1. write u(k−1) into the outgoing frame                             │
   ├─ 2. EtherCAT frame out and back (typ. tens of µs, logged as          │
   │     pdo_exchange_us)                                                 │
   │      ├─ drive receives u(k−1) → its current loop starts following it │
   │      └─ drive returns its latest inputs: AI1 (laser), currents, ...  │
   ├─ 3. communication checks (working counter, drive state)              │
   ├─ 4. AI1 raw → volts → y(k) in mm                                     │
   ├─ 5. position trip check on y(k)                                      │
   ├─ 6. r(k) = reference at this time; PI → u(k), stored for next cycle  │
   ├─ 7. timing measurement, then log one row (y(k), r(k), P, I, u(k))    │
   └─ 8. sleep ──────────────────────────────────────────────────────────►├─ 1. write u(k) ...
```

- **The measurement and the action are one cycle apart.** The position received in cycle k is used to compute u(k). That current goes to the drive at the start of cycle k+1, 0.5 ms later. Add the drive's internal input sampling and current-loop response, and the total delay from measurement to force is roughly 1 ms.
- **Why that delay doesn't matter here.** A 1 ms delay costs 360° · f · 0.001 s of phase:
  - **0.2°** at the loop's crossover (~0.5 Hz);
  - **1.6°** at the 4.3 Hz resonance.
  - Both are negligible.
- **When is AI1 sampled?** The drive's AI1 value is the latest sample it had when the frame passed. The drive samples on its own clock: DC SYNC0 is deliberately off, see `docs/rt-implementation.md`. The exact sampling instant within the cycle is therefore not fixed, but it is always less than a cycle old.
- **Why the controller runs after the exchange.** It runs on the position that has just arrived. So the trip check, the PI and the logged row all use the same y(k), and every CSV row is consistent: "this position gave this error gave this current".

## 4. Phases of a run

With the default profile (0 → +1 → −1 → 0 mm):

| Time | What happens |
|---|---|
| −3 … 0 s | **Idle window.** Drive enabled, 0 A, PI off (integrator held at 0). This is used to measure the accelerometer bias (see `accelerometer-bias.md`). The position trip is already active. |
| 0 s | **PI switches on.** The reference is 0 mm, but the shaft rests wherever the springs left it (+0.2 … +2.3 mm seen on 2 Oct). The PI pulls it to 0 mm in about 3–4 s. This first move is expected, because the frame is absolute. |
| 0 … 8 s | hold 0 mm |
| 8 … 9 s | ramp to +1 mm |
| 9 … 17 s | hold +1 mm |
| 17 … 18 s | ramp to −1 mm |
| 18 … 26 s | hold −1 mm |
| 26 … 27 s | ramp to 0 mm |
| 27 … 36 s | hold 0 mm until `RUN_DURATION_S` |
| 36 s | **End.** The loop stops and the drive voltage is disabled. Current drops to 0 and the springs return the shaft to rest. |

**Expected response.** The closed loop follows the reference with a time constant of about 0.8 s. A step is about 95 % complete after 2.4 s and fully settled within about 4 s, well inside the 8 s holds. Holding ±1 mm takes only about 0.15 A; pulling 2 mm from rest takes about 0.3 A. A simulation of the real controller code against the identified plant peaked at 0.47 A.

## 5. Units and scaling chain

**Position in.** AI1 raw counts → volts → millimetres:

```
V   = raw / 819.2
mm  = −1 · (6.568 · V + 22.5 − 51.7)
```

- 6.568 and 22.5 are the laser calibration (4–20 mA into the drive's input).
- 51.7 mm is the laser distance at the centre.
- The −1 makes positive = the direction positive current pushes.

The controller and the CSV log use the same conversion function, so they always agree. If the laser loses its signal (0 V), this reads **+29.2 mm**. That is outside the trip window, so a lost sensor stops the run.

**Current out.** Amps → drive units, where KP is the drive's peak current, read from the drive at startup:

```
raw = round(A · 32768 / KP), clamped to −32767 … +32767   (= ±KP)
```

The clamp limits are symmetric on purpose. The outgoing field is a signed 16-bit integer, which can hold −32768 … +32767.
- **The old bug.** The previous code clamped to +32768. That doesn't fit in the field and wraps to −32768. A full positive command would have reached the drive as a **full negative** current.
- **How the conversion is protected now.**
  - It clamps before converting to an integer.
  - It sends 0 A if the current is not a finite number.
  - It sends 0 A if KP could not be read.
  - This applies to every experiment mode.
- **How it was tested.** Before committing, off-hardware tests swept −2·KP … +2·KP in 1 mA steps for several KP values. They checked range, sign and monotonicity, and that NaN and infinity give 0.

## 6. Gain choice and tuning

**The starting gains.**
- **Kp = 0.02 A/mm** was chosen small on purpose.
- **Ki = 0.2 A/(mm·s)** is set by the resonance, not by speed.

**The constraint.** With PI control on a mass–spring–damper, the closed loop stays stable only if

```
Ki  <  c · (k + Kp·K_f·1000) / (m · K_f · 1000)        (Ki in A/(mm·s), c in N·s/m, k in N/m)
```

The system identification gives two very different damping values, so the limit is uncertain:

| Damping c (N·s/m) | Stability limit for Ki | Ki = 0.2 is | Damping ratio of the 4.4 Hz mode at Ki = 0.2 | Same at Ki = 0.5 |
|---|---|---|---|---|
| 197 (lowest estimate) | 0.62 | 32 % of limit | 0.046 | 0.013 (almost undamped) |
| 1160 (datasheet) | 3.6 | 6 % | 0.39 | 0.36 |
| 1534 (notebook, centre samples) | 4.8 | 4 % | 0.52 | 0.50 |
| 1912 (notebook, final model) | 6.0 | 3 % | 0.66 | 0.65 |

Ki = 0.2 is safe for every estimate. Ki = 0.5 would be fine if the damping is really above 1000 N·s/m, but would almost cancel the resonance damping if it is 197.

**How to tune safely, one change per run:**
1. **Raise Kp first**, e.g. ×2 at a time. Kp adds stiffness without phase lag, and it *raises* the Ki limit in the formula above. Laser noise passes through Kp to the current; at 0.02 A/mm, 0.1 mm of noise is only 2 mA.
2. **Then raise Ki**, keeping it below about a third of the limit for the lowest damping estimate.
3. **After every change:** look at the position trace for 4–5 Hz ringing after each step. Ringing that lasts longer than the previous run means the resonance is less damped: go back.
4. **Measure the real damping.** A run with this controller, or a step-release run, shows how fast the 4.4 Hz ringing decays. That tells you which damping estimate is right, and therefore how much room there is.

## 7. Safety layers

| Layer | Triggers when | What happens |
|---|---|---|
| Reference window (compile time) | a breakpoint, or sine offset ± amplitude, is outside −8 … +8 mm | the build fails with a message |
| Gain sign (compile time) | Kp or Ki negative | the build fails (a negative gain is positive feedback) |
| Trip inside reference window (compile time) | reference window not inside the trip window | the build fails |
| Startup check | KP not read from the drive, or the 10 A output limit is not below KP, or the breakpoints are out of order / ramps overlap | message, the loop never starts, drive disabled |
| PI output clamp | P + I beyond ±10 A | output held at ±10 A, integrator frozen (anti-windup) |
| Invalid number guard | reference or position not a finite number | 0 A for that cycle |
| Current conversion | any value beyond ±KP, not finite, or KP = 0 | clamped to ±KP, or 0 A |
| **Position trip** | measured position outside **−10 … +10 mm** for **3 consecutive cycles** (1.5 ms), any time including the idle window | 0 A, drive voltage disabled, run stops, `POSITION_LIMIT` fault logged and printed |
| Communication watchdog (existing) | more than 5 consecutive bad working counters | run stops |
| Drive state watchdog (existing) | drive leaves Operation Enabled (e.g. its own fault) | voltage disabled, run stops |

The 3-cycle confirmation keeps a single noise spike from stopping the run. 1.5 ms is far too short for the mover (51.5 kg) to travel any real distance.

## 8. Reading the log

The CSV in `gcsc_data/` is named, for example, `voice_coil_log_posPI_steps_0.0s0.0mm-8.0s1.0mm-17.0s-1.0mm-26.0s0.0mm-r1.0s_kp0.02_ki0.2_<date>_<time>.csv`. It has four new columns at the end (all other modes write `nan` in them):

| Column | Meaning |
|---|---|
| `position_ref_mm` | r(k), the reference this cycle (`nan` in the idle window) |
| `pid_p_A` | P term |
| `pid_i_A` | integrator after this cycle's update |
| `pid_output_A` | u(k), the clamped output. **Sent to the drive in the next cycle.** `target_current_A` (the drive's echo) shows it a row or two later. |

`position_mm` in the same row is the y(k) the controller used. Tracking error = `position_ref_mm − position_mm`.

`scripts/plot_voice_coil_log.py` draws the reference over the measured position, and the PI output on the current plot.

## 9. Where to change things

All settings are `#define`s in `main.h`. Rebuild after changing them.

| What | Setting |
|---|---|
| Select this experiment | `EXPERIMENT_MODE` = `EXPERIMENT_POSITION_PID` |
| Run length (after the 3 s idle window) | `RUN_DURATION_S` (36 s; other experiments need it longer, their checks will tell you) |
| Reference shape | `POS_REF_SHAPE` = `POS_REF_SHAPE_STEPS` or `POS_REF_SHAPE_SINE` |
| Step breakpoints | `POS_REF_STEPS(X)`: `X(time_s, position_mm)` entries, numbers with a decimal point |
| Ramp time between steps | `POS_REF_RAMP_S` |
| Sine reference | `POS_REF_SINE_OFFSET_MM`, `POS_REF_SINE_AMPLITUDE_MM`, `POS_REF_SINE_FREQ_HZ` |
| Gains | `PID_KP_A_PER_MM`, `PID_KI_A_PER_MM_S` |
| Output current limit | `PID_OUTPUT_LIMIT_A` |
| Allowed reference range | `POS_REF_MIN_MM`, `POS_REF_MAX_MM` |
| Trip window and confirmation | `POS_TRIP_MIN_MM`, `POS_TRIP_MAX_MM`, `POS_TRIP_CYCLES` |
| Laser calibration | `AI1_MM_SCALE`, `AI1_MM_OFFSET`, `AI1_CENTRE_MM`, `AI1_POSITION_SIGN` |
