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
#else
#error "Unknown EXPERIMENT_MODE"
#endif
}

/** \brief Run the real-time control loop: generate experiment setpoints, exchange PDO, monitor faults
 *  Operates for RUN_DURATION_S seconds at CYCLE_TIME_MS intervals, synchronized via DC SYNC0.
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
   double elapsed_s = 0.0;
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
#endif
   printf("Starting %.0f-second cyclic loop... expected WKC: %d\n", RUN_DURATION_S, expected_wkc);

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
      /* Compute target current for the selected experiment */
      target_current_A = experiment_target_current_A(elapsed_s);

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
      elapsed_s = (double)cycle_count * CYCLE_TIME_MS / 1000.0;
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
