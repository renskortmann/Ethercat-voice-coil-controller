/** \file control_loop.c
 * \brief Real-time cyclic loop: experiment setpoint generation, PDO exchange, fault monitoring, timing
 */

#include "main.h"

/** \brief Add microseconds to a struct timespec, handling nanosecond overflow
 *  \param ts Pointer to timespec to modify in-place
 *  \param addus Microseconds to add
 */
void
add_timespec(struct timespec *ts, int64_t addus)
{
   ts->tv_nsec += addus * 1000;
   while (ts->tv_nsec >= 1000000000)
   {
      ts->tv_sec++;
      ts->tv_nsec -= 1000000000;
   }
}

/** \brief Signed difference (end - start) between two timespecs, in microseconds
 *  \param end Later timestamp
 *  \param start Earlier timestamp
 *  \return end - start in microseconds (negative if end precedes start)
 */
static double
timespec_diff_us(const struct timespec *end, const struct timespec *start)
{
   return (double)(end->tv_sec - start->tv_sec) * 1e6 +
          (double)(end->tv_nsec - start->tv_nsec) / 1000.0;
}

/** \brief Largest raw target current magnitude. Symmetric on purpose: +32768 does not fit the int16_t
 *  RxPDO field and would wrap to -32768, i.e. a full positive command reaching the drive as full reverse. */
#define TARGET_CURRENT_RAW_MAX 32767.0

/** \brief Convert a target current in Amps to the raw RxPDO value (6071h, scale 2^15 / KP)
 *  Every step is done in double and clamped before the one integer conversion, so the result is
 *  always in range: no int16 wrap, and no undefined behaviour from converting NaN, inf or an
 *  out-of-range double to an integer.
 *  \param amps Target current in Amps
 *  \param kp_amps Drive peak current (KP); 0 if the SDO read at startup failed
 *  \return Raw value in [-32767, 32767] (+/-KP); 0 if amps is not finite or kp_amps is not > 0
 */
static int16_t
amps_to_target_current_raw(double amps, double kp_amps)
{
   if (!isfinite(amps) || !(kp_amps > 0.0))
   {
      return 0;
   }
   double raw = amps * DC2_SCALE / kp_amps;
   if (raw > TARGET_CURRENT_RAW_MAX) raw = TARGET_CURRENT_RAW_MAX;
   if (raw < -TARGET_CURRENT_RAW_MAX) raw = -TARGET_CURRENT_RAW_MAX;
   return (int16_t)lround(raw);
}

/** \brief Phase of an exponential sweep f(t) = f0 * (f1 / f0)^(t / T), i.e. equal time per octave.
 *  Shared by the current-mode chirps and the position chirp reference.
 *  \param t Time since the start of the sweep in seconds
 *  \return Phase in radians: the integral of 2 pi f(t), which is 2 pi f0 L (exp(t / L) - 1), L = T / ln(f1 / f0)
 */
static inline double
exp_chirp_phase(double t, double f0, double f1, double T)
{
   const double L = T / log(f1 / f0);
   return 2.0 * M_PI * f0 * L * (exp(t / L) - 1.0);
}

#if EXPERIMENT_MODE == EXPERIMENT_CHIRP || EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
/** \brief Unit-amplitude exponential chirp CHIRP_F0_HZ -> CHIRP_F1_HZ over CHIRP_DURATION_S
 *  \param t Time since the start of the sweep in seconds (0 <= t < CHIRP_DURATION_S)
 *  \return sin(phase(t)), in [-1, 1]
 */
static double
chirp_unit(double t)
{
   return sin(exp_chirp_phase(t, CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S));
}
#endif

#if EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
/** \brief Amplitude breakpoints from CHIRP_SCHED_TABLE, validated by chirp_sched_check() */
static const struct
{
   double t_s;      /**< Time the ramp to amp_A starts, in seconds */
   double amp_A;    /**< Amplitude after the ramp, in Amps */
} chirp_sched[] = CHIRP_SCHED_TABLE;
#define CHIRP_SCHED_N ((int)(sizeof(chirp_sched) / sizeof(chirp_sched[0])))

/** \brief Chirp amplitude envelope at time t: the last breakpoint at or before t, ramped in
 *  linearly from the previous breakpoint's amplitude over CHIRP_SCHED_RAMP_S
 *  \param t Time since the start of the sweep in seconds (t >= 0)
 *  \return Amplitude in Amps
 */
static double
chirp_sched_amplitude_A(double t)
{
   int i = 0;
   while (i + 1 < CHIRP_SCHED_N && chirp_sched[i + 1].t_s <= t)
   {
      i++;
   }
   double into_ramp_s = t - chirp_sched[i].t_s;
   if (i > 0 && CHIRP_SCHED_RAMP_S > 0.0 && into_ramp_s < CHIRP_SCHED_RAMP_S)
   {
      double prev_A = chirp_sched[i - 1].amp_A;
      return prev_A + (chirp_sched[i].amp_A - prev_A) * (into_ramp_s / CHIRP_SCHED_RAMP_S);
   }
   return chirp_sched[i].amp_A;
}

/** \brief Check CHIRP_SCHED_TABLE before the run (doubles rule out a _Static_assert)
 *  \param kp_amps Drive peak current; larger amplitudes would be clipped by the saturation
 *  \return TRUE if the table is usable, FALSE (with a message printed) otherwise
 */
static boolean
chirp_sched_check(double kp_amps)
{
   if (chirp_sched[0].t_s != 0.0)
   {
      printf("CHIRP_SCHED: first breakpoint must be at t = 0 s (is %.3f s)\n", chirp_sched[0].t_s);
      return FALSE;
   }
   for (int i = 0; i < CHIRP_SCHED_N; i++)
   {
      if (fabs(chirp_sched[i].amp_A) > kp_amps)
      {
         printf("CHIRP_SCHED: %.2f A at t = %.3f s exceeds the drive peak current %.1f A\n",
                chirp_sched[i].amp_A, chirp_sched[i].t_s, kp_amps);
         return FALSE;
      }
      if (chirp_sched[i].t_s >= CHIRP_DURATION_S)
      {
         printf("CHIRP_SCHED: breakpoint at t = %.3f s is not before CHIRP_DURATION_S (%.1f s)\n",
                chirp_sched[i].t_s, CHIRP_DURATION_S);
         return FALSE;
      }
      if (i > 0 && chirp_sched[i].t_s < chirp_sched[i - 1].t_s + CHIRP_SCHED_RAMP_S)
      {
         printf("CHIRP_SCHED: breakpoint at t = %.3f s starts before the ramp at t = %.3f s ends "
                "(times must increase by at least CHIRP_SCHED_RAMP_S = %.3f s)\n",
                chirp_sched[i].t_s, chirp_sched[i - 1].t_s, CHIRP_SCHED_RAMP_S);
         return FALSE;
      }
   }
   return TRUE;
}
#endif

#if EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
/** \brief Sine blocks from SINE_BLOCKS_TABLE, validated by sine_blocks_check() */
static const struct
{
   double f_Hz;     /**< Sine frequency in Hz */
   double amp_A;    /**< Amplitude during the hold, in Amps */
} sine_blocks[] = SINE_BLOCKS_TABLE;
#define SINE_BLOCKS_N ((int)(sizeof(sine_blocks) / sizeof(sine_blocks[0])))

/** \brief Sine-block current at time t: ramp-in, hold, ramp-out, pause per block, then 0 A
 *  \param t Time since the start of the first block in seconds (t >= 0)
 *  \return Target current in Amps
 */
static double
sine_blocks_current_A(double t)
{
   int k = (int)(t / SINE_BLOCK_PERIOD_S);
   if (k >= SINE_BLOCKS_N)
   {
      return 0.0;                                            /* all blocks done: ring-down */
   }
   double tb = t - k * SINE_BLOCK_PERIOD_S;                  /* time into block k */
   double envelope;
   if (tb < SINE_BLOCK_RAMP_S)
   {
      envelope = tb / SINE_BLOCK_RAMP_S;                     /* ramp in */
   }
   else if (tb < SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S)
   {
      envelope = 1.0;                                        /* hold */
   }
   else if (tb < 2.0 * SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S)
   {
      envelope = (2.0 * SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S - tb) / SINE_BLOCK_RAMP_S;  /* ramp out */
   }
   else
   {
      return 0.0;                                            /* pause */
   }
   return sine_blocks[k].amp_A * envelope * sin(2.0 * M_PI * sine_blocks[k].f_Hz * tb);
}

/** \brief Check SINE_BLOCKS_TABLE before the run (doubles rule out a _Static_assert)
 *  \param kp_amps Drive peak current; larger amplitudes would be clipped by the saturation
 *  \return TRUE if the table is usable, FALSE (with a message printed) otherwise
 */
static boolean
sine_blocks_check(double kp_amps)
{
   for (int i = 0; i < SINE_BLOCKS_N; i++)
   {
      if (fabs(sine_blocks[i].amp_A) > kp_amps)
      {
         printf("SINE_BLOCKS: %.2f A in block %d (%.2f Hz) exceeds the drive peak current %.1f A\n",
                sine_blocks[i].amp_A, i, sine_blocks[i].f_Hz, kp_amps);
         return FALSE;
      }
   }
   return TRUE;
}
#endif

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
/** \brief Reference breakpoints from POS_REF_STEPS_TABLE, validated by pos_ref_check() */
static const struct
{
   double t_s;      /**< Time the ramp to pos_mm starts, in seconds */
   double pos_mm;   /**< Reference after the ramp, in mm */
} pos_ref_steps[] = POS_REF_STEPS_TABLE;
#define POS_REF_STEPS_N ((int)(sizeof(pos_ref_steps) / sizeof(pos_ref_steps[0])))
#endif

#if POS_REF_SHAPE == POS_REF_SHAPE_CHIRP
/** \brief Sweep time at which the position chirp ends: the first zero crossing of the sine at or after
 *  POS_REF_CHIRP_DURATION_S, so the reference returns to the offset without a step. Computed by inverting
 *  the phase: tau = L ln(1 + phase / (2 pi f0 L)) at phase = pi * ceil(phase(T) / pi).
 *  \return Sweep time in seconds since POS_REF_CHIRP_START_S (T <= result < T + half a period of f1)
 */
static double
pos_ref_chirp_end_s(void)
{
   static double end_s = -1.0;   /* computed once; pos_ref_mm() calls this every cycle */
   if (end_s < 0.0)
   {
      const double L = POS_REF_CHIRP_DURATION_S / log(POS_REF_CHIRP_F1_HZ / POS_REF_CHIRP_F0_HZ);
      double phase_T = exp_chirp_phase(POS_REF_CHIRP_DURATION_S, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ,
                                       POS_REF_CHIRP_DURATION_S);
      double phase_end = M_PI * ceil(phase_T / M_PI);
      end_s = L * log(1.0 + phase_end / (2.0 * M_PI * POS_REF_CHIRP_F0_HZ * L));
   }
   return end_s;
}
#endif

/** \brief Position reference at time t
 *  \param t Time since the controller started in seconds (t >= 0)
 *  \return Reference position in mm (absolute position_mm frame)
 */
static double
pos_ref_mm(double t)
{
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
   /* Same breakpoint semantics as chirp_sched_amplitude_A(): last breakpoint at or before t,
    * ramped in linearly from the previous value; the last value holds to the end of the run. */
   int i = 0;
   while (i + 1 < POS_REF_STEPS_N && pos_ref_steps[i + 1].t_s <= t)
   {
      i++;
   }
   double into_ramp_s = t - pos_ref_steps[i].t_s;
   if (i > 0 && POS_REF_RAMP_S > 0.0 && into_ramp_s < POS_REF_RAMP_S)
   {
      double prev_mm = pos_ref_steps[i - 1].pos_mm;
      return prev_mm + (pos_ref_steps[i].pos_mm - prev_mm) * (into_ramp_s / POS_REF_RAMP_S);
   }
   return pos_ref_steps[i].pos_mm;
#elif POS_REF_SHAPE == POS_REF_SHAPE_CHIRP
   double tau = t - POS_REF_CHIRP_START_S;
   if (tau <= 0.0 || tau >= pos_ref_chirp_end_s())
   {
      return POS_REF_CHIRP_OFFSET_MM;                        /* hold before and after the sweep */
   }
   return POS_REF_CHIRP_OFFSET_MM + POS_REF_CHIRP_AMPLITUDE_MM *
          sin(exp_chirp_phase(tau, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ, POS_REF_CHIRP_DURATION_S));
#else
   return POS_REF_SINE_OFFSET_MM + POS_REF_SINE_AMPLITUDE_MM * sin(2.0 * M_PI * POS_REF_SINE_FREQ_HZ * t);
#endif
}

/** \brief Check the controller settings before the run (doubles rule out some _Static_asserts)
 *  \param kp_amps Drive peak current; the PI output limit must stay below it
 *  \return TRUE if usable, FALSE (with a message printed) otherwise
 */
static boolean
pos_ref_check(double kp_amps)
{
   if (!(kp_amps > 0.0))
   {
      printf("POSITION PID: drive peak current KP not read (%.1f A); refusing to run\n", kp_amps);
      return FALSE;
   }
   if (PID_OUTPUT_LIMIT_A >= kp_amps)
   {
      printf("POSITION PID: PID_OUTPUT_LIMIT_A %.2f A is not below the drive peak current %.1f A\n",
             PID_OUTPUT_LIMIT_A, kp_amps);
      return FALSE;
   }
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
   if (pos_ref_steps[0].t_s != 0.0)
   {
      printf("POS_REF_STEPS: first breakpoint must be at t = 0 s (is %.3f s)\n", pos_ref_steps[0].t_s);
      return FALSE;
   }
   for (int i = 0; i < POS_REF_STEPS_N; i++)
   {
      if (pos_ref_steps[i].pos_mm < POS_REF_MIN_MM || pos_ref_steps[i].pos_mm > POS_REF_MAX_MM)
      {
         printf("POS_REF_STEPS: %.3f mm at t = %.3f s is outside %.1f .. %.1f mm\n",
                pos_ref_steps[i].pos_mm, pos_ref_steps[i].t_s, POS_REF_MIN_MM, POS_REF_MAX_MM);
         return FALSE;
      }
      if (pos_ref_steps[i].t_s >= RUN_DURATION_S)
      {
         printf("POS_REF_STEPS: breakpoint at t = %.3f s is not before RUN_DURATION_S (%.1f s)\n",
                pos_ref_steps[i].t_s, RUN_DURATION_S);
         return FALSE;
      }
      if (i > 0 && pos_ref_steps[i].t_s < pos_ref_steps[i - 1].t_s + POS_REF_RAMP_S)
      {
         printf("POS_REF_STEPS: breakpoint at t = %.3f s starts before the ramp at t = %.3f s ends "
                "(times must increase by at least POS_REF_RAMP_S = %.3f s)\n",
                pos_ref_steps[i].t_s, pos_ref_steps[i - 1].t_s, POS_REF_RAMP_S);
         return FALSE;
      }
   }
#endif
   return TRUE;
}

/** \brief Clamp x to [-limit, limit] (NaN passes through; callers check isfinite) */
static double
clamp_abs(double x, double limit)
{
   if (x > limit) return limit;
   if (x < -limit) return -limit;
   return x;
}

#if POS_NOTCH_ENABLE
/** \brief Position notch filter: biquad notch at POS_NOTCH_FREQ_HZ (RBJ cookbook, bilinear transform at
 *  the cycle rate), Direct Form II transposed. DC gain is exactly 1, so a constant position passes
 *  unchanged. */
static struct
{
   double b0, b1, b2, a1, a2;   /**< Coefficients normalised to a0 = 1 */
   double z1, z2;               /**< Filter state */
   boolean seeded;              /**< FALSE until the first sample has set the state */
} notch;

/** \brief Compute the notch coefficients; call once before the cyclic loop */
static void
notch_init(void)
{
   const double fs_Hz = 1000.0 / CYCLE_TIME_MS;
   double w0 = 2.0 * M_PI * POS_NOTCH_FREQ_HZ / fs_Hz;
   double alpha = sin(w0) / (2.0 * POS_NOTCH_Q);
   double a0 = 1.0 + alpha;
   notch.b0 = 1.0 / a0;
   notch.b1 = -2.0 * cos(w0) / a0;
   notch.b2 = 1.0 / a0;
   notch.a1 = -2.0 * cos(w0) / a0;
   notch.a2 = (1.0 - alpha) / a0;
   notch.seeded = FALSE;
}

/** \brief Set the state as if the input had been constant at x forever, so the output starts at x
 *  without a transient */
static void
notch_seed(double x)
{
   notch.z2 = (notch.b2 - notch.a2) * x;
   notch.z1 = (notch.b1 - notch.a1) * x + notch.z2;
   notch.seeded = TRUE;
}

/** \brief Filter one position sample
 *  \param x Raw position in mm
 *  \return Notched position in mm
 */
static double
notch_step(double x)
{
   if (!notch.seeded)
   {
      notch_seed(x);
   }
   double y = notch.b0 * x + notch.z1;
   notch.z1 = notch.b1 * x - notch.a1 * y + notch.z2;
   notch.z2 = notch.b2 * x - notch.a2 * y;
   if (!isfinite(y))
   {
      /* Cannot happen with a valid int16 AI1 reading; restart from the raw sample. */
      notch_seed(x);
      y = x;
   }
   return y;
}
#endif

/** \brief PI integrator in Amps. Starts at 0 and is only updated from t = 0, so the idle window
 *  leaves it at 0. */
static double pi_integrator_A = 0.0;

/** \brief One PI update: e = r - y, P = Kp e, I += Ki e dt, u = sat(P + I)
 *  Anti-windup by conditional integration: while P + I is beyond the output limit, integrator steps
 *  that would push it further out are dropped. The integrator is also clamped to the output limit.
 *  A non-finite input or result (which a valid AI1 reading cannot produce) gives 0 A.
 *  \param r_mm Reference position in mm
 *  \param y_mm Measured position in mm (from this cycle's PDO exchange)
 *  \return Reference, P, I and the saturated output; output_A is sent to the drive next cycle
 */
static pid_log_t
pi_update(double r_mm, double y_mm)
{
   const double dt_s = CYCLE_TIME_MS / 1000.0;
   if (!isfinite(r_mm) || !isfinite(y_mm))
   {
      /* Cannot happen with a valid int16 AI1 reading; command 0 A and leave the integrator alone. */
      return (pid_log_t){ .position_ref_mm = r_mm, .p_A = 0.0, .i_A = pi_integrator_A, .output_A = 0.0 };
   }
   double e_mm = r_mm - y_mm;
   double p_A = PID_KP_A_PER_MM * e_mm;
   double i_A = pi_integrator_A + PID_KI_A_PER_MM_S * e_mm * dt_s;
   double unsat_A = p_A + i_A;
   if ((unsat_A > PID_OUTPUT_LIMIT_A && e_mm > 0.0) || (unsat_A < -PID_OUTPUT_LIMIT_A && e_mm < 0.0))
   {
      i_A = pi_integrator_A;                                 /* saturated: hold the integrator */
   }
   i_A = clamp_abs(i_A, PID_OUTPUT_LIMIT_A);
   if (!isfinite(i_A))
   {
      i_A = 0.0;
   }
   pi_integrator_A = i_A;

   double u_A = clamp_abs(p_A + i_A, PID_OUTPUT_LIMIT_A);
   if (!isfinite(u_A))
   {
      u_A = 0.0;
   }
   return (pid_log_t){ .position_ref_mm = r_mm, .p_A = p_A, .i_A = i_A, .output_A = u_A };
}
#else
/** \brief Target current for this cycle, in Amps, for the experiment selected by EXPERIMENT_MODE
 *  This is the only experiment-specific code in the cyclic loop. It must stay cheap and
 *  allocation-free: it runs once per cycle inside the real-time loop.
 *  \param elapsed_s Elapsed loop time in seconds (cycle_count * cycle time)
 *  \return Target current in Amps (converted to raw drive units by the caller)
 */
static double
experiment_target_current_A(double elapsed_s)
{
#if EXPERIMENT_MODE == EXPERIMENT_SINE
   return SINE_AMPLITUDE_A * sin(2.0 * M_PI * SINE_FREQ_HZ * elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_STEP_RELEASE
   if (elapsed_s >= HOLD_DURATION_S)
   {
      return 0.0;                                            /* released: free response */
   }
   if (HOLD_RAMP_S > 0.0 && elapsed_s < HOLD_RAMP_S)
   {
      return HOLD_CURRENT_A * (elapsed_s / HOLD_RAMP_S);     /* ramp in */
   }
   return HOLD_CURRENT_A;                                    /* hold */
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP
   if (elapsed_s >= CHIRP_DURATION_S)
   {
      return 0.0;                                            /* sweep done: ring-down */
   }
   return CHIRP_AMPLITUDE_A * chirp_unit(elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
   if (elapsed_s >= CHIRP_DURATION_S)
   {
      return 0.0;                                            /* sweep done: ring-down */
   }
   return chirp_sched_amplitude_A(elapsed_s) * chirp_unit(elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_PRBS
   static uint16_t lfsr = 1;      /* 15-bit LFSR state; nonzero seed, fixed so every run repeats */
   static long bit_index = 0;     /* index of the PRBS bit held in lfsr */
   if (elapsed_s >= PRBS_DURATION_S)
   {
      return 0.0;                                            /* sequence done: ring-down */
   }
   /* Cycle index from elapsed time; rounding absorbs floating-point error in elapsed_s. */
   long target_bit = lround(elapsed_s * 1000.0 / CYCLE_TIME_MS) / PRBS_HOLD_CYCLES;
   while (bit_index < target_bit)
   {
      /* Fibonacci LFSR, polynomial x^15 + x^14 + 1 (maximal length, period 32767) */
      uint16_t bit = ((lfsr >> 14) ^ (lfsr >> 13)) & 1u;
      lfsr = (uint16_t)(((lfsr << 1) | bit) & 0x7FFFu);
      bit_index++;
   }
   return (lfsr & 1u) ? PRBS_AMPLITUDE_A : -PRBS_AMPLITUDE_A;
#elif EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
   return sine_blocks_current_A(elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_NOISE
   (void)elapsed_s;
   return 0.0;                                               /* noise floor: drive enabled, no excitation */
#else
#error "Unknown EXPERIMENT_MODE"
#endif
}
#endif /* EXPERIMENT_MODE == EXPERIMENT_POSITION_PID */

/** \brief Run the real-time control loop: generate experiment setpoints, exchange PDO, monitor faults
 *  Operates for BIAS_IDLE_S (0 A, t < 0) + RUN_DURATION_S seconds at CYCLE_TIME_MS intervals, synchronized via DC SYNC0.
 *  Detects WKC errors and CiA402 state drift; logs samples and faults to in-memory buffers.
 *
 *  No blocking I/O (printf/fprintf) happens inside the cycle loop itself: it would add
 *  unbounded latency right when the loop needs to stay on schedule. Per-cycle timing jitter
 *  is instead recorded into each sample (see cycle_jitter_us), and one-time error messages
 *  are deferred to just after the loop exits.
 *  \param fieldbus Fieldbus context (sample and fault buffers populated)
 *  \return TRUE on completion (normal or fault-triggered shutdown), FALSE on internal error
 */
boolean
fieldbus_run_cyclic(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   rx_pdo_t *rx = (rx_pdo_t *)grp->outputs;
   tx_pdo_t *tx = (tx_pdo_t *)grp->inputs;
   int expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;

   struct timespec next_cycle, now, pdo_start, pdo_end;
   int64_t cycle_ns = (int64_t)(CYCLE_TIME_MS * 1000000);
   double elapsed_s = -BIAS_IDLE_S;  /* negative during the bias idle window; 0 = experiment start */
   double target_current_A;
   double cycle_jitter_us;
   double pdo_exchange_us;
   double max_pdo_exchange_us = 0.0;
   int wkc;
   int wkc_error_count = 0;
   int cycle_count = 0;
   uint16_t current_state;
   uint16_t state_drift_statusword = 0;
   boolean fault_detected = FALSE;
   boolean wkc_threshold_exceeded = FALSE;
   boolean state_drift_detected = FALSE;
   int missed_deadline_count = 0;
   double max_jitter_us = 0.0;
#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
   double next_current_A = 0.0;   /* PI output from the previous cycle's measurement; 0 A until the first update */
   pid_log_t pid;                 /* this cycle's controller values, for the log */
   double position_mm;            /* raw, used by the trip check */
   double position_filt_mm;       /* notched (POS_NOTCH_ENABLE) or raw, used by the PI */
   int pos_trip_count = 0;        /* consecutive cycles outside the trip window */
   boolean pos_trip_detected = FALSE;
   double pos_trip_position_mm = 0.0;
#endif

#if EXPERIMENT_MODE == EXPERIMENT_SINE
   printf("\nExperiment: sine, %.1f Hz, %.2f A amplitude\n", SINE_FREQ_HZ, SINE_AMPLITUDE_A);
#elif EXPERIMENT_MODE == EXPERIMENT_STEP_RELEASE
   printf("\nExperiment: step-release, hold %.2f A (ramp %.2f s) until t = %.2f s, then release to 0 A\n",
          HOLD_CURRENT_A, HOLD_RAMP_S, HOLD_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP
   printf("\nExperiment: exponential chirp, %.2f A, %.2f -> %.2f Hz over %.2f s, then 0 A\n",
          CHIRP_AMPLITUDE_A, CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
   printf("\nExperiment: exponential chirp, scheduled amplitude, %.2f -> %.2f Hz over %.2f s, then 0 A\n",
          CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S);
   for (int i = 0; i < CHIRP_SCHED_N; i++)
   {
      printf("  t = %7.2f s -> %.2f A%s\n", chirp_sched[i].t_s, chirp_sched[i].amp_A,
             i > 0 && CHIRP_SCHED_RAMP_S > 0.0 ? " (linear ramp)" : "");
   }
   if (CHIRP_SCHED_RAMP_S > 0.0)
   {
      printf("  ramp time %.2f s\n", CHIRP_SCHED_RAMP_S);
   }
   if (!chirp_sched_check(fieldbus->kp_amps))
   {
      return FALSE;
   }
#elif EXPERIMENT_MODE == EXPERIMENT_PRBS
   printf("\nExperiment: PRBS, +/-%.2f A, bandwidth %.1f Hz (%d cycles per bit) for %.2f s, then 0 A\n",
          PRBS_AMPLITUDE_A, PRBS_BANDWIDTH_HZ, PRBS_HOLD_CYCLES, PRBS_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
   printf("\nExperiment: %d sine blocks, each %.1f s ramp-in, %.1f s hold, %.1f s ramp-out, %.1f s at 0 A\n",
          SINE_BLOCKS_N, SINE_BLOCK_RAMP_S, SINE_BLOCK_HOLD_S, SINE_BLOCK_RAMP_S, SINE_BLOCK_PAUSE_S);
   for (int i = 0; i < SINE_BLOCKS_N; i++)
   {
      double hold_start_s = i * SINE_BLOCK_PERIOD_S + SINE_BLOCK_RAMP_S;
      printf("  block %d: %6.2f Hz, %5.2f A, hold t = %7.2f .. %7.2f s\n", i, sine_blocks[i].f_Hz,
             sine_blocks[i].amp_A, hold_start_s, hold_start_s + SINE_BLOCK_HOLD_S);
   }
   if (!sine_blocks_check(fieldbus->kp_amps))
   {
      return FALSE;
   }
#elif EXPERIMENT_MODE == EXPERIMENT_NOISE
   printf("\nExperiment: noise, 0 A with the drive enabled for %.2f s\n", RUN_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
   printf("\nExperiment: position PI, Kp = %.4f A/mm, Ki = %.4f A/(mm s), output limit +/-%.2f A (drive KP %.1f A)\n",
          PID_KP_A_PER_MM, PID_KI_A_PER_MM_S, PID_OUTPUT_LIMIT_A, fieldbus->kp_amps);
   printf("  reference window %.1f .. %.1f mm, trip outside %.1f .. %.1f mm (raw position) for %d cycles\n",
          POS_REF_MIN_MM, POS_REF_MAX_MM, POS_TRIP_MIN_MM, POS_TRIP_MAX_MM, POS_TRIP_CYCLES);
#if POS_NOTCH_ENABLE
   printf("  position filter: notch at %.2f Hz, Q = %.2f (PI uses the notched position)\n",
          POS_NOTCH_FREQ_HZ, POS_NOTCH_Q);
   notch_init();
#else
   printf("  position filter: off (PI uses the raw position)\n");
#endif
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
   printf("  reference: absolute steps, last value held until t = %.2f s\n", RUN_DURATION_S);
   for (int i = 0; i < POS_REF_STEPS_N; i++)
   {
      printf("  t = %7.2f s -> %+.3f mm%s\n", pos_ref_steps[i].t_s, pos_ref_steps[i].pos_mm,
             i > 0 && POS_REF_RAMP_S > 0.0 ? " (linear ramp)" : "");
   }
   if (POS_REF_RAMP_S > 0.0)
   {
      printf("  ramp time %.2f s\n", POS_REF_RAMP_S);
   }
#elif POS_REF_SHAPE == POS_REF_SHAPE_CHIRP
   printf("  reference: absolute chirp, %+.3f mm + %.3f mm * sin(phase), exponential sweep %.2f -> %.2f Hz\n",
          POS_REF_CHIRP_OFFSET_MM, POS_REF_CHIRP_AMPLITUDE_MM, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ);
   printf("  hold %+.3f mm until t = %.2f s, sweep until t = %.3f s (next zero crossing after %.2f s), "
          "then hold until t = %.2f s\n", POS_REF_CHIRP_OFFSET_MM, POS_REF_CHIRP_START_S,
          POS_REF_CHIRP_START_S + pos_ref_chirp_end_s(), POS_REF_CHIRP_START_S + POS_REF_CHIRP_DURATION_S,
          RUN_DURATION_S);
#else
   printf("  reference: absolute sine, %+.3f mm + %.3f mm * sin(2 pi %.3f Hz t)\n",
          POS_REF_SINE_OFFSET_MM, POS_REF_SINE_AMPLITUDE_MM, POS_REF_SINE_FREQ_HZ);
#endif
   if (!pos_ref_check(fieldbus->kp_amps))
   {
      return FALSE;
   }
#endif
   printf("Bias window: %.1f s at 0 A before the experiment (t < 0) to measure the accelerometer offset\n",
          BIAS_IDLE_S);
   printf("Starting %.0f-second cyclic loop (%.0f s idle + %.0f s experiment)... expected WKC: %d\n",
          BIAS_IDLE_S + RUN_DURATION_S, BIAS_IDLE_S, RUN_DURATION_S, expected_wkc);

   /* SYNC0 stays off: this loop paces itself from CLOCK_MONOTONIC and never phase-locks to the
    * slave's DC clock, so with SYNC0 enabled the frame-arrival phase walks at the two oscillators'
    * ppm difference until it crosses the SYNC0 edge and the drive trips on a sync/comm error.
    * Current-loop-only CST needs no DC sync; the drive runs SM-synchronous instead. */
   ecx_dcsync0(context, fieldbus->amc_slave_index, FALSE, 0, 0);

   clock_gettime(CLOCK_MONOTONIC, &next_cycle);
   /* Advance to the first real deadline (loop start + one cycle) before entering the loop,
    * so cycle 0's jitter is measured against an actual deadline rather than the loop-start
    * instant, which would always register as a spurious missed deadline. */
   add_timespec(&next_cycle, cycle_ns / 1000);

   while (elapsed_s < RUN_DURATION_S)
   {
      /* Bias idle window (t < 0): hold 0 A with the drive enabled, so the accelerometer offset is
       * measured in the same electrical conditions as the run. The experiment functions only ever
       * see t >= 0 (PRBS derives its bit index from elapsed_s and assumes it is non-negative). */
#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
      /* Closed loop: send the PI output computed at the end of the previous cycle from the position
       * received then. The PI runs right after the exchange below, so measurement, trip check, PI
       * and log row all use the same received position. */
      target_current_A = next_current_A;
#else
      if (elapsed_s < 0.0)
      {
         target_current_A = 0.0;
      }
      else
      {
         target_current_A = experiment_target_current_A(elapsed_s);
      }
#endif

      rx->target_current = amps_to_target_current_raw(target_current_A, fieldbus->kp_amps);

      /* Send and receive process data. Time this separately from the rest of the cycle:
       * it's the frame round-trip (NIC driver + wire + slave + housekeeping-core IRQ servicing), so a
       * spike here points outward at the bus/slave/host, while a spike in cycle_jitter_us
       * without a matching pdo_exchange_us spike points at the loop thread itself. */
      clock_gettime(CLOCK_MONOTONIC, &pdo_start);
      ecx_send_processdata(context);
      wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
      clock_gettime(CLOCK_MONOTONIC, &pdo_end);
      pdo_exchange_us = timespec_diff_us(&pdo_end, &pdo_start);
      if (pdo_exchange_us > max_pdo_exchange_us)
      {
         max_pdo_exchange_us = pdo_exchange_us;
      }

      /* Working Counter validation: detect frame loss or slave response failure */
      if (wkc < expected_wkc)
      {
         wkc_error_count++;
         log_fault(fieldbus, elapsed_s, FAULT_WKC_ERROR, wkc, RECOVERY_AUTO_RECOVER);
         if (wkc_error_count > 5)
         {
            log_fault(fieldbus, elapsed_s, FAULT_WKC_ERROR, wkc, RECOVERY_SHUTDOWN_INITIATED);
            fault_detected = TRUE;
            wkc_threshold_exceeded = TRUE;
            break;
         }
      }
      else
      {
         wkc_error_count = 0;
      }

      /* State machine validation: ensure drive remains in Operation Enabled */
      current_state = tx->statusword & STATUS_WORD_MASK;
      if (current_state != STATE_OPERATION_ENABLED)
      {
         log_fault(fieldbus, elapsed_s, FAULT_STATE_DRIFT, current_state, RECOVERY_SHUTDOWN_INITIATED);
         rx->controlword = CTRL_DISABLE_VOLT;
         fault_detected = TRUE;
         state_drift_detected = TRUE;
         state_drift_statusword = tx->statusword;
         break;
      }

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
      /* Raw position from this cycle's exchange. The trip checks it unfiltered over the whole run,
       * idle window included: a laser fault (signal lost reads about +29 mm) or a shaft outside the
       * window stops the run. */
      position_mm = ai1_raw_to_position_mm(tx->ai1_value);
      if (position_mm < POS_TRIP_MIN_MM || position_mm > POS_TRIP_MAX_MM)
      {
         pos_trip_count++;
         if (pos_trip_count >= POS_TRIP_CYCLES)
         {
            rx->target_current = 0;
            rx->controlword = CTRL_DISABLE_VOLT;
            /* Send it now: after the loop come printouts and SDO diagnostics, during which the
             * drive would otherwise keep applying the last commanded current. */
            fieldbus_roundtrip(fieldbus);
            log_fault(fieldbus, elapsed_s, FAULT_POSITION_LIMIT, (uint32_t)pos_trip_count,
                      RECOVERY_SHUTDOWN_INITIATED);
            fault_detected = TRUE;
            pos_trip_detected = TRUE;
            pos_trip_position_mm = position_mm;
            break;
         }
      }
      else
      {
         pos_trip_count = 0;
      }

      /* The notch runs from the first cycle, so it has settled long before the PI starts at t = 0. */
#if POS_NOTCH_ENABLE
      position_filt_mm = notch_step(position_mm);
#else
      position_filt_mm = position_mm;
#endif

      if (elapsed_s < 0.0)
      {
         /* Bias idle window: controller off, 0 A, integrator stays 0. */
         pid = (pid_log_t){ .position_ref_mm = NAN, .p_A = 0.0, .i_A = 0.0, .output_A = 0.0 };
      }
      else
      {
         pid = pi_update(pos_ref_mm(elapsed_s), position_filt_mm);
      }
      pid.position_filt_mm = position_filt_mm;
      next_current_A = pid.output_A;
#endif

      /* Measure cycle timing jitter (actual time vs. scheduled deadline) before logging the
       * sample, so it's captured in the same row instead of being printed live. */
      clock_gettime(CLOCK_MONOTONIC, &now);
      cycle_jitter_us = timespec_diff_us(&now, &next_cycle);
      if (cycle_jitter_us > 0.0)
      {
         missed_deadline_count++;
         if (cycle_jitter_us > max_jitter_us)
         {
            max_jitter_us = cycle_jitter_us;
         }
      }

      /* Log sample */
#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
      log_sample(fieldbus, elapsed_s, tx, cycle_jitter_us, pdo_exchange_us, &pid);
#else
      log_sample(fieldbus, elapsed_s, tx, cycle_jitter_us, pdo_exchange_us, NULL);
#endif

      /* Wait for next cycle using absolute-time sleep */
      if (cycle_jitter_us <= 0.0)
      {
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_cycle, NULL);
      }

      add_timespec(&next_cycle, cycle_ns / 1000);
      elapsed_s = (double)cycle_count * CYCLE_TIME_MS / 1000.0 - BIAS_IDLE_S;
      cycle_count++;
   }

   if (wkc_threshold_exceeded)
   {
      printf("ERROR: WKC errors exceeded threshold, initiating shutdown\n");
   }
   if (state_drift_detected)
   {
      printf("ERROR: Drive dropped out of OPERATION_ENABLED state (0x%04X), initiating shutdown\n",
             state_drift_statusword);
   }
#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
   if (pos_trip_detected)
   {
      printf("ERROR: POSITION TRIP at t = %.3f s: position %+.3f mm outside %.1f .. %.1f mm for %d cycles, "
             "0 A and voltage disabled\n",
             elapsed_s, pos_trip_position_mm, POS_TRIP_MIN_MM, POS_TRIP_MAX_MM, POS_TRIP_CYCLES);
   }
#endif
   printf("Cyclic loop finished. Samples: %d, Faults: %d, missed deadlines: %d "
          "(max jitter %.1f us, max PDO exchange %.1f us)\n",
          fieldbus->sample_count, fieldbus->fault_count, missed_deadline_count,
          max_jitter_us, max_pdo_exchange_us);

   /* Read drive diagnostic objects via SDO after cyclic loop exits (PDO idle) */
   if (fault_detected)
   {
      read_drive_status_sdo(fieldbus, elapsed_s);
   }

   return TRUE;
}
