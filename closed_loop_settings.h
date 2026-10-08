/** \file closed_loop_settings.h
 * \brief SETTINGS: knobs for closed-loop position control (EXPERIMENT_POSITION_PID)
 *
 * A controller on the AI1 laser position drives the coil current so position_mm follows a
 * compile-time reference. Sections run from "changed every run" to "almost never":
 *   1. Reference          what the shaft should follow
 *   2. Controller         which controller, and its gains
 *   3. Position filters   EKF and notches between the laser and the controller
 *   4. Safety limits      reference window and position trip
 * The code is in closed_loop_position.c (reference, filters, trip) and controller_*.c (one file per
 * controller); the plant model the EKF uses is in vca_model.h. See docs/position-control.md.
 * Write the numbers with a decimal point (1.0, not 1): they are copied verbatim into the file name.
 */

#ifndef CLOSED_LOOP_SETTINGS_H
#define CLOSED_LOOP_SETTINGS_H

/* ---- 1. Reference ---------------------------------------------------------------------------- */
/** The reference is absolute position_mm (0 = AI1_CENTRE_MM), not an offset from where the shaft rests,
 *  so at t = 0 the loop pulls the shaft from rest to the first reference value. During the BIAS_IDLE_S
 *  window the output is 0 A and the controller is held at rest. */
#define POS_REF_SHAPE_STEPS 0    /**< Breakpoints from POS_REF_STEPS, linearly ramped over POS_REF_RAMP_S */
#define POS_REF_SHAPE_SINE  1    /**< POS_REF_SINE_OFFSET_MM + POS_REF_SINE_AMPLITUDE_MM * sin(2 pi f t) */
#define POS_REF_SHAPE_CHIRP 2    /**< Exponential sine sweep POS_REF_CHIRP_F0_HZ -> POS_REF_CHIRP_F1_HZ around POS_REF_CHIRP_OFFSET_MM */
#define POS_REF_SHAPE       POS_REF_SHAPE_CHIRP

/** POS_REF_SHAPE_STEPS: X(time_s, position_mm) breakpoints: at time_s the reference ramps linearly from
 *  the previous value to position_mm over POS_REF_RAMP_S, then holds; the last value is held until
 *  RUN_DURATION_S. One entry gives a constant setpoint. The first entry must be at t = 0 (the starting
 *  reference, no ramp-in) and each ramp must end before the next breakpoint. */
#define POS_REF_STEPS(X)    X(0.0, 0.0) X(5.0, 5.0) X(10.0, -2.0) X(20.0, 4.0) X(30.0, 0.0)
#define POS_REF_RAMP_S      0.0   /**< Ramp time at each breakpoint in seconds; 0 for a hard step */

/** POS_REF_SHAPE_SINE */
#define POS_REF_SINE_OFFSET_MM    0.0   /**< Sine reference centre, in mm */
#define POS_REF_SINE_AMPLITUDE_MM 5.0   /**< Sine reference amplitude, in mm */
#define POS_REF_SINE_FREQ_HZ      1.0   /**< Sine reference frequency, in Hz */

/** POS_REF_SHAPE_CHIRP: offset until POS_REF_CHIRP_START_S, then offset + amplitude * sin(phase) with the
 *  exponential sweep f(t) = f0 * (f1 / f0)^(t / T) (equal time per octave, same law as the current-mode
 *  CHIRP), then offset until RUN_DURATION_S. The sweep runs on to the next zero crossing after T (at most
 *  half a period), so the reference has no step at either end. */
#define POS_REF_CHIRP_OFFSET_MM    0.0   /**< Chirp centre, in mm */
#define POS_REF_CHIRP_AMPLITUDE_MM 3.0   /**< Chirp amplitude, in mm */
#define POS_REF_CHIRP_F0_HZ        1.0   /**< Start frequency in Hz (> 0) */
#define POS_REF_CHIRP_F1_HZ        5.0  /**< End frequency in Hz */
#define POS_REF_CHIRP_DURATION_S   60.0  /**< Sweep length T in seconds */
#define POS_REF_CHIRP_START_S      2.0   /**< Hold at the offset before the sweep starts, in seconds */


/* ---- 2. Controller --------------------------------------------------------------------------- */
#define POS_CONTROLLER_PI   0    /**< PI with conditional-integration anti-windup (controller_pi.c) */
#define POS_CONTROLLER      POS_CONTROLLER_PI
#define PID_OUTPUT_LIMIT_A  10.0  /**< Controller output saturation, +/- Amps, for every controller; must be below the drive peak current */

/** POS_CONTROLLER_PI */
#define PID_KP_A_PER_MM     1.0  /**< Proportional gain: Amps per mm of position error */
#define PID_KI_A_PER_MM_S   0.12   /**< Integral gain: Amps per mm of error per second */
/* No D term yet: the laser signal carries noise and 50 Hz pickup. A later D term should act on the
 * measurement (not the error, to avoid a kick on reference steps) through a first-order low-pass. */


/* ---- 3. Position filters --------------------------------------------------------------------- */
/** Order: raw position -> EKF (if enabled) -> notches (if enabled) -> controller. The position trip
 *  always uses the raw, unfiltered position. */

/** Extended Kalman filter: the Method C model of vca_greybox_fit.ipynb (ekf(), state [x, v], measurement
 *  x; parameters in vca_model.h), run on the raw position at the cycle rate. Model input for the step
 *  k-1 -> k is the current sent in cycle k-1; the controller uses the filtered estimate x(k|k). */
#define POS_KF_ENABLE       1     /**< 1 = controller uses the EKF position x(k|k), 0 = off */
#define POS_KF_SIG_A_M_S2   7.86  /**< Process noise: white acceleration std in m/s^2 (Method C residual on its fit data) */
#define POS_KF_SIG_Y_MM     0.0766 /**< Measurement noise std in mm (raw laser in the idle window) */

/** Notch filters against mains pickup on the laser signal (50 Hz and its 150 Hz harmonic). Notches
 *  instead of a low-pass: at 7-12 Hz a 50 Hz notch costs ~1.5 deg lag, a 150 Hz notch ~0.5 deg; the lags
 *  of cascaded notches add. With POS_KF_ENABLE the notches run on x(k|k), so the EKF still sees the raw
 *  position. */
#define POS_NOTCH_ENABLE    1     /**< 1 = controller uses the notched position, 0 = the raw position (or x(k|k)) */
/** Notches in cascade, applied in this order: X(centre frequency in Hz, quality factor Q). The -3 dB
 *  width is freq / Q; higher Q = less lag at the loop frequency (Q 5 at 50 Hz can destabilise
 *  Kp ~1 A/mm if damping is low). */
#define POS_NOTCHES(X)      X(50.0, 10.0) X(150.0, 10.0)


/* ---- 4. Safety limits ------------------------------------------------------------------------ */
#define POS_REF_MIN_MM      -8.0  /**< Lowest reference allowed by the startup check, in mm */
#define POS_REF_MAX_MM      8.0   /**< Highest reference allowed by the startup check, in mm */
#define POS_TRIP_MIN_MM     -10.0 /**< Runtime trip: 0 A and shut down below this position, in mm */
#define POS_TRIP_MAX_MM     10.0  /**< Runtime trip: 0 A and shut down above this position, in mm */
#define POS_TRIP_CYCLES     3     /**< Consecutive cycles outside the trip window before tripping (rejects single noise spikes) */

#endif /* CLOSED_LOOP_SETTINGS_H */
