/** \file run_settings.h
 * \brief SETTINGS: which experiment runs, and for how long.
 *
 * The specific experiment settings are in open_loop_settings.h or closed_loop_settings.h. 
 * Rebuild after editing this file.
 */

#ifndef RUN_SETTINGS_H
#define RUN_SETTINGS_H

/** \brief Experiment types. Only the per-cycle setpoint changes between experiments. */
#define EXPERIMENT_SINE          0   /**< Feedforward sine current */
#define EXPERIMENT_STEP_RELEASE  1   /**< Hold a constant current, then release to zero and record the free response */
#define EXPERIMENT_CHIRP         2   /**< Exponential current sweep */
#define EXPERIMENT_PRBS          3   /**< Pseudo-random binary current */
#define EXPERIMENT_CHIRP_SCHEDULED 4 /**< Same sweep as EXPERIMENT_CHIRP, amplitude follows CHIRP_SCHED */
#define EXPERIMENT_NOISE         5   /**< 0 A for the whole run with the drive enabled */
#define EXPERIMENT_SINE_BLOCKS   6   /**< Sequence of ramped constant-frequency sine blocks */
#define EXPERIMENT_CLOSED_LOOP   7   /**< A controller on the AI1 laser position tracks a reference */

/** \brief Select the experiment to run (compile-time). */
#define EXPERIMENT_MODE          EXPERIMENT_CHIRP

/** \brief Select the experiment duration. */
#define RUN_DURATION_S      125.0 /**< Experiment duration in seconds, excluding the bias idle window */
#define BIAS_IDLE_S         3.0 /**< 0 A rest period before the experiment starts, to measure accelerometer bias.
 *  Timestamps are shifted so this window has negative time (-BIAS_IDLE_S .. 0) and the excitation
 *  still starts at t = 0. Total loop time is BIAS_IDLE_S + RUN_DURATION_S. */

/** \brief Select the magnet gap g0: centre to each magnet face, in mm. The model parameters in vca_model.h were fitted at 22.3 mm */
#define MAGNET_GAP_MM       12.0   /** Write it with a decimal point */
#define VCA_GAP_M           (MAGNET_GAP_MM * 1e-3)   /**< The same gap in m, for vca_model.h */

#endif /* RUN_SETTINGS_H */
