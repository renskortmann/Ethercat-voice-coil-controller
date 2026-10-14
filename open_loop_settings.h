/** \file open_loop_settings.h
 * \brief SETTINGS: knobs for the open-loop current experiments, one section per EXPERIMENT_MODE
 *
 * Only the section of the mode selected in run_settings.h is used. The code that turns these
 * numbers into a current, and the checks on them, are in open_loop_current.c/.h. Rebuild after editing.
 */

#ifndef OPEN_LOOP_SETTINGS_H
#define OPEN_LOOP_SETTINGS_H

/* ---- EXPERIMENT_SINE ------------------------------------------------------------------------- */
/** \brief Feedforward sine experiment parameters (EXPERIMENT_SINE) */
#define SINE_FREQ_HZ        15.0 /**< Target current waveform frequency in Hz */
#define SINE_AMPLITUDE_A    5.0  /**< Target current waveform amplitude in Amps */


/* ---- EXPERIMENT_STEP_RELEASE ----------------------------------------------------------------- */
/** \brief Step-release experiment parameters (EXPERIMENT_STEP_RELEASE) */
#define HOLD_CURRENT_A      7.5  /**< Constant current during the hold phase, in Amps (sign = direction) */
#define HOLD_RAMP_S         0.2  /**< Linear ramp 0 -> HOLD_CURRENT_A at the start of the hold; 0 for a hard step */
#define HOLD_DURATION_S     20.0  /**< Time from loop start to release, in seconds (includes the ramp) */


/* ---- EXPERIMENT_CHIRP and EXPERIMENT_CHIRP_SCHEDULED ----------------------------------------- */
/** \brief Chirp experiment parameters (EXPERIMENT_CHIRP). Instantaneous frequency is
 *  f(t) = CHIRP_F0_HZ * (CHIRP_F1_HZ / CHIRP_F0_HZ)^(t / CHIRP_DURATION_S), so every decade
 *  gets the same sweep time. After CHIRP_DURATION_S the current is 0 A for the rest of the run. */
#define CHIRP_AMPLITUDE_A   6.0   /**< Current amplitude in Amps */
#define CHIRP_F0_HZ         10.0   /**< Start frequency in Hz (> 0) */
#define CHIRP_F1_HZ         55.0 /**< End frequency in Hz */
#define CHIRP_DURATION_S    120.0   /**< Sweep length in seconds, <= RUN_DURATION_S */

/** \brief Scheduled-amplitude chirp parameters (EXPERIMENT_CHIRP_SCHEDULED). The frequency sweep
 *  uses CHIRP_F0_HZ, CHIRP_F1_HZ and CHIRP_DURATION_S above; CHIRP_AMPLITUDE_A is not used.
 *  CHIRP_SCHED lists X(time_s, amplitude_A) breakpoints: at time_s the amplitude ramps
 *  linearly from the previous breakpoint's value to amplitude_A over CHIRP_SCHED_RAMP_S, then holds.
 *  The first entry must be at t = 0 (the starting amplitude, no ramp-in); times strictly increasing,
 *  each ramp must end before the next breakpoint and the last breakpoint must be < CHIRP_DURATION_S.
 *  Checked at startup, which also rejects amplitudes above the drive peak current.
 *  Write the numbers with a decimal point (6.0, not 6): they are copied verbatim into the file name. */
#define CHIRP_SCHED(X)      X(0.0, 3.0) X(30.0, 7.0) X(60.0, 14.0)
#define CHIRP_SCHED_RAMP_S  30.0       /**< Ramp time at each breakpoint in seconds; 0 for a hard step */


/* ---- EXPERIMENT_SINE_BLOCKS ------------------------------------------------------------------ */
/** \brief Sine-block experiment parameters (EXPERIMENT_SINE_BLOCKS). SINE_BLOCKS lists
 *  X(freq_Hz, amplitude_A) blocks, run back to back. Each block: linear ramp-in over
 *  SINE_BLOCK_RAMP_S, full amplitude for SINE_BLOCK_HOLD_S (the analysis window), linear ramp-out
 *  over SINE_BLOCK_RAMP_S, then SINE_BLOCK_PAUSE_S at 0 A. The sine phase restarts at 0 at each
 *  block start. Block k's hold window is t = k * SINE_BLOCK_PERIOD_S + SINE_BLOCK_RAMP_S .. + SINE_BLOCK_HOLD_S.
 *  Amplitudes are checked against the drive peak current at startup.
 *  Write the numbers with a decimal point (6.0, not 6): they are copied verbatim into the file name. */
#define SINE_BLOCKS(X)      X(10.0, 10.0) X(20.0, 15.0) X(40.0, 20.0) X(50.0, 22.0)
#define SINE_BLOCK_RAMP_S   5.0   /**< Linear ramp-in and ramp-out time per block, in seconds (> 0) */
#define SINE_BLOCK_HOLD_S   20.0  /**< Full-amplitude time per block, in seconds */
#define SINE_BLOCK_PAUSE_S  2.0   /**< 0 A after each block (including the last), in seconds */


/* ---- EXPERIMENT_PRBS ------------------------------------------------------------------------- */
/** \brief PRBS experiment parameters (EXPERIMENT_PRBS). A 15-bit maximum-length LFSR
 *  (period 32767 bits, fixed seed) sets the current to +A or -A, each bit held for
 *  PRBS_HOLD_CYCLES cycles. The power spectrum follows sinc^2(f * T_bit) and is about 2 dB down
 *  at 0.4 / T_bit >= PRBS_BANDWIDTH_HZ. After PRBS_DURATION_S the current is 0 A. */
#define PRBS_AMPLITUDE_A    5.0   /**< Current level in Amps: output is +A or -A */
#define PRBS_BANDWIDTH_HZ   55.0  /**< Upper frequency of the flat part of the spectrum, in Hz (<= 60) */
#define PRBS_DURATION_S     120.0 /**< Sequence length in seconds, <= RUN_DURATION_S */


/* ---- EXPERIMENT_NOISE: no settings (0 A for RUN_DURATION_S) ---------------------------------- */

#endif /* OPEN_LOOP_SETTINGS_H */
