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

#if EXPERIMENT_MODE == EXPERIMENT_CHIRP || EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
/** \brief Unit-amplitude exponential chirp CHIRP_F0_HZ -> CHIRP_F1_HZ over CHIRP_DURATION_S
 *  \param t Time since the start of the sweep in seconds (0 <= t < CHIRP_DURATION_S)
 *  \return sin(phase(t)), in [-1, 1]
 */
static double
chirp_unit(double t)
{
   /* Phase is the integral of f(t) = f0 * exp(t / L), with L = T / ln(f1 / f0). */
   const double L = CHIRP_DURATION_S / log(CHIRP_F1_HZ / CHIRP_F0_HZ);
   return sin(2.0 * M_PI * CHIRP_F0_HZ * L * (exp(t / L) - 1.0));
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
      printf("CHIRP_SCHED_TABLE: first breakpoint must be at t = 0 s (is %.3f s)\n", chirp_sched[0].t_s);
      return FALSE;
   }
   for (int i = 0; i < CHIRP_SCHED_N; i++)
   {
      if (fabs(chirp_sched[i].amp_A) > kp_amps)
      {
         printf("CHIRP_SCHED_TABLE: %.2f A at t = %.3f s exceeds the drive peak current %.1f A\n",
                chirp_sched[i].amp_A, chirp_sched[i].t_s, kp_amps);
         return FALSE;
      }
      if (chirp_sched[i].t_s >= CHIRP_DURATION_S)
      {
         printf("CHIRP_SCHED_TABLE: breakpoint at t = %.3f s is not before CHIRP_DURATION_S (%.1f s)\n",
                chirp_sched[i].t_s, CHIRP_DURATION_S);
         return FALSE;
      }
      if (i > 0 && chirp_sched[i].t_s < chirp_sched[i - 1].t_s + CHIRP_SCHED_RAMP_S)
      {
         printf("CHIRP_SCHED_TABLE: breakpoint at t = %.3f s starts before the ramp at t = %.3f s ends "
                "(times must increase by at least CHIRP_SCHED_RAMP_S = %.3f s)\n",
                chirp_sched[i].t_s, chirp_sched[i - 1].t_s, CHIRP_SCHED_RAMP_S);
         return FALSE;
      }
   }
   return TRUE;
}
#endif

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
#else
#error "Unknown EXPERIMENT_MODE"
#endif
}

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
   int32_t target_current_raw;
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
      if (elapsed_s < 0.0)
      {
         target_current_A = 0.0;
      }
      else
      {
         target_current_A = experiment_target_current_A(elapsed_s);
      }

      /* Convert to raw Int32 (scale: 2^15 / KP) */
      target_current_raw = (int32_t)round((target_current_A * DC2_SCALE) / fieldbus->kp_amps);
      if (target_current_raw > DC2_SCALE) target_current_raw = DC2_SCALE;  /* Saturate to max DC2 */
      if (target_current_raw < -DC2_SCALE) target_current_raw = -DC2_SCALE;  /* Saturate to min -DC2 */

      rx->target_current = target_current_raw;

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
      log_sample(fieldbus, elapsed_s, tx, cycle_jitter_us, pdo_exchange_us);

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
