/** \file run_settings.h
 * \brief SETTINGS: which experiment runs, and for how long
 *
 * The experiment's own knobs are in open_loop_settings.h (current waveforms) or
 * closed_loop_settings.h (EXPERIMENT_POSITION_PID). Rebuild after editing.
 */

#ifndef RUN_SETTINGS_H
#define RUN_SETTINGS_H

/** \brief Experiment selection (compile-time). Only the per-cycle setpoint changes between
 *  experiments; scaling, PDO exchange, fault checks, timing and logging are shared. */
#define EXPERIMENT_SINE          0   /**< Feedforward sine current (SINE_FREQ_HZ, SINE_AMPLITUDE_A) */
#define EXPERIMENT_STEP_RELEASE  1   /**< Hold a constant current, then release to zero and record the free response */
#define EXPERIMENT_CHIRP         2   /**< Exponential current sweep CHIRP_F0_HZ -> CHIRP_F1_HZ, then 0 A */
#define EXPERIMENT_PRBS          3   /**< Pseudo-random binary current +/-PRBS_AMPLITUDE_A, then 0 A */
#define EXPERIMENT_CHIRP_SCHEDULED 4 /**< Same sweep as EXPERIMENT_CHIRP, amplitude follows CHIRP_SCHED */
#define EXPERIMENT_NOISE         5   /**< 0 A for the whole run with the drive enabled: sensor noise floor */
#define EXPERIMENT_SINE_BLOCKS   6   /**< Sequence of ramped constant-frequency sine blocks from SINE_BLOCKS */
#define EXPERIMENT_POSITION_PID  7   /**< Closed-loop position control: PI on the AI1 laser position tracks POS_REF_* */
#define EXPERIMENT_MODE          EXPERIMENT_POSITION_PID /**< Select the experiment to run (compile-time). A new mode also needs an EXPERIMENT_TAG case in open_loop_current.h. */

#define RUN_DURATION_S      150.0 /**< Experiment (excitation) phase duration in seconds, after the bias idle window */
/** 0 A rest period before the experiment starts, used to measure the accelerometer (AI2) bias.
 *  Timestamps are shifted so this window has negative time (-BIAS_IDLE_S .. 0) and the excitation
 *  still starts at t = 0. Total loop time is BIAS_IDLE_S + RUN_DURATION_S. export_csv() subtracts
 *  the mean AI2 voltage over this window to produce the ai2_corrected_V column. */
#define BIAS_IDLE_S         3.0

#endif /* RUN_SETTINGS_H */
