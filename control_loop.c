/** \file control_loop.c
 * \brief Real-time cyclic loop: experiment setpoint generation, PDO exchange, fault monitoring, timing
 */

#include "main.h"
#include "open_loop_current.h"
#include "closed_loop_position.h"

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
   double next_current_A = 0.0;   /* controller output from the previous cycle's measurement; 0 A until the first update */
   pid_log_t pid;                 /* this cycle's controller values, for the log */
   double position_mm;            /* raw, used by the trip check */
   boolean pos_trip_detected = FALSE;
   double pos_trip_position_mm = 0.0;
#endif

   /* Print the experiment and check its settings (closed_loop_position.c / open_loop_current.c) */
#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
   if (!closed_loop_setup(fieldbus->kp_amps))
#else
   if (!open_loop_setup(fieldbus->kp_amps))
#endif
   {
      return FALSE;
   }
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
      /* Closed loop: send the controller output computed at the end of the previous cycle from the
       * position received then. The controller runs right after the exchange below, so measurement,
       * trip check, controller and log row all use the same received position. */
      target_current_A = next_current_A;
#else
      if (elapsed_s < 0.0)
      {
         target_current_A = 0.0;
      }
      else
      {
         target_current_A = open_loop_current_A(elapsed_s);
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
      if (closed_loop_trip_update(position_mm))
      {
         rx->target_current = 0;
         rx->controlword = CTRL_DISABLE_VOLT;
         /* Send it now: after the loop come printouts and SDO diagnostics, during which the
          * drive would otherwise keep applying the last commanded current. */
         fieldbus_roundtrip(fieldbus);
         log_fault(fieldbus, elapsed_s, FAULT_POSITION_LIMIT, (uint32_t)POS_TRIP_CYCLES,
                   RECOVERY_SHUTDOWN_INITIATED);
         fault_detected = TRUE;
         pos_trip_detected = TRUE;
         pos_trip_position_mm = position_mm;
         break;
      }

      /* Filters and controller (closed_loop_position.c); the output is sent in the next cycle */
      pid = closed_loop_step(elapsed_s, position_mm, target_current_A);
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
