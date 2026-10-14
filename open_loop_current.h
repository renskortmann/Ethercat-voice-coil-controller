/** \file open_loop_current.h
 * \brief Open-loop current experiments: a compile-time current waveform per EXPERIMENT_MODE
 *
 * Settings in open_loop_settings.h.
 */

#ifndef OPEN_LOOP_CURRENT_H
#define OPEN_LOOP_CURRENT_H

#include "main.h"
#include "open_loop_settings.h"

boolean open_loop_setup(double kp_amps);
double open_loop_current_A(double t_s);

/** \brief Cycles per PRBS bit: longest hold with 0.4 / T_bit >= PRBS_BANDWIDTH_HZ (13 at 60 Hz, 0.5 ms). */
#define PRBS_HOLD_CYCLES    ((int)(400.0 / (PRBS_BANDWIDTH_HZ * CYCLE_TIME_MS)))
/** \brief Length of one sine block: ramp-in, hold, ramp-out, pause */
#define SINE_BLOCK_PERIOD_S (2.0 * SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S + SINE_BLOCK_PAUSE_S)

/** \brief File-name labels generated from the X-macro tables. CHIRP_SCHED_NAME is e.g.
 *  "0.0s6.0A-60.0s15.0A-r30.0s" (each breakpoint as <time>s<amplitude>A, then the ramp time);
 *  SINE_BLOCKS_NAME is e.g. "10.0Hz5.0A-20.0Hz10.0A-" (each block as <freq>Hz<amplitude>A). */
#define CHIRP_SCHED_LABEL_(t, a)  STRINGIFY(t) "s" STRINGIFY(a) "A-"
#define CHIRP_SCHED_NAME          CHIRP_SCHED(CHIRP_SCHED_LABEL_) "r" STRINGIFY(CHIRP_SCHED_RAMP_S) "s"
#define SINE_BLOCKS_LABEL_(f, a)  STRINGIFY(f) "Hz" STRINGIFY(a) "A-"
#define SINE_BLOCKS_NAME          SINE_BLOCKS(SINE_BLOCKS_LABEL_)

/** \brief Experiment mode + parameters as a filename-safe tag, built at compile time from the
 *  settings so the values are never duplicated by hand. Used by export_csv() to name the
 *  CSV files, e.g. voice_coil_log_sine_15.0Hz_5.0A_YYYYMMDD_HHMMSS.csv. The position-control tag
 *  is in closed_loop_position.h. */
#if EXPERIMENT_MODE == EXPERIMENT_SINE
#define EXPERIMENT_TAG "sine_" STRINGIFY(SINE_FREQ_HZ) "Hz_" STRINGIFY(SINE_AMPLITUDE_A) "A"
#elif EXPERIMENT_MODE == EXPERIMENT_STEP_RELEASE
#define EXPERIMENT_TAG "step_release_hold" STRINGIFY(HOLD_CURRENT_A) "A_ramp" STRINGIFY(HOLD_RAMP_S) \
                       "s_dur" STRINGIFY(HOLD_DURATION_S) "s"
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP
#define EXPERIMENT_TAG "chirp_" STRINGIFY(CHIRP_AMPLITUDE_A) "A_" STRINGIFY(CHIRP_F0_HZ) "to" \
                       STRINGIFY(CHIRP_F1_HZ) "Hz_" STRINGIFY(CHIRP_DURATION_S) "s"
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
#define EXPERIMENT_TAG "chirpsched_" CHIRP_SCHED_NAME "_" STRINGIFY(CHIRP_F0_HZ) "to" \
                       STRINGIFY(CHIRP_F1_HZ) "Hz_" STRINGIFY(CHIRP_DURATION_S) "s"
#elif EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
#define EXPERIMENT_TAG "sineblocks_" SINE_BLOCKS_NAME "r" STRINGIFY(SINE_BLOCK_RAMP_S) "s_h" \
                       STRINGIFY(SINE_BLOCK_HOLD_S) "s_p" STRINGIFY(SINE_BLOCK_PAUSE_S) "s"
#elif EXPERIMENT_MODE == EXPERIMENT_NOISE
#define EXPERIMENT_TAG "noise_0A_" STRINGIFY(RUN_DURATION_S) "s"
#elif EXPERIMENT_MODE == EXPERIMENT_PRBS
#define EXPERIMENT_TAG "prbs_" STRINGIFY(PRBS_AMPLITUDE_A) "A_" STRINGIFY(PRBS_BANDWIDTH_HZ) "Hz_" \
                       STRINGIFY(PRBS_DURATION_S) "s"
#elif EXPERIMENT_MODE != EXPERIMENT_POSITION_PID
#error "Unknown EXPERIMENT_MODE"
#endif

#endif /* OPEN_LOOP_CURRENT_H */
