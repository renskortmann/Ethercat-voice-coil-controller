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
#define POS_REF_SHAPE_CHIRP_HOLD 3 /**< The same sweep, then POS_REF_CHIRP_F1_HOLD_S at f1 before the fade-out */
#define POS_REF_SHAPE       POS_REF_SHAPE_CHIRP_HOLD

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
 *  half a period), so the reference has no step at either end. The velocity would still step there (from
 *  0 to amplitude * 2 pi f0 at the start, from amplitude * 2 pi f1 to 0 at the end), which the SMC turns into
 *  a current kick; POS_REF_CHIRP_TAPER_S fades the amplitude in and out (raised cosine) to avoid that. */
#define POS_REF_CHIRP_OFFSET_MM    0.0   /**< Chirp centre, in mm */
#define POS_REF_CHIRP_AMPLITUDE_MM 1.0   /**< Chirp amplitude, in mm */
#define POS_REF_CHIRP_F0_HZ        1.0   /**< Start frequency in Hz (> 0) */
#define POS_REF_CHIRP_F1_HZ        51.0  /**< End frequency in Hz */
#define POS_REF_CHIRP_DURATION_S   120.0  /**< Sweep length T in seconds */
#define POS_REF_CHIRP_START_S      0.0   /**< Hold at the offset before the sweep starts, in seconds */
#define POS_REF_CHIRP_TAPER_S      2.0   /**< Amplitude fade-in at the start and fade-out at the end of the sweep, in seconds; 0 = off */

/** POS_REF_SHAPE_CHIRP_HOLD: all POS_REF_CHIRP_* settings above, plus a hold at the end frequency. After the
 *  sweep (fade-in, f0 -> f1 over POS_REF_CHIRP_DURATION_S) the sine runs on at f1 with full amplitude for
 *  POS_REF_CHIRP_F1_HOLD_S, then fades out over POS_REF_CHIRP_TAPER_S, still at f1, to the next zero crossing;
 *  then offset until RUN_DURATION_S. The phase is continuous, so the reference and its derivatives are smooth. */
#define POS_REF_CHIRP_F1_HOLD_S    20.0  /**< Time at f1 with full amplitude, after the sweep and before the fade-out, in seconds */


/* ---- 2. Controller --------------------------------------------------------------------------- */
#define POS_CONTROLLER_PI   0    /**< PI with conditional-integration anti-windup (controller_pi.c) */
#define POS_CONTROLLER_SMC  1    /**< Sliding-mode controller on the Method C model (controller_smc.c); needs POS_KF_ENABLE */
#define POS_CONTROLLER      POS_CONTROLLER_SMC
#define OUTPUT_LIMIT_A      28.0  /**< Controller output saturation, +/- Amps, for every controller; must be below the drive peak current */


/** POS_CONTROLLER_PI */
#define PID_KP_A_PER_MM     1.0  /**< Proportional gain: Amps per mm of position error */
#define PID_KI_A_PER_MM_S   0.12   /**< Integral gain: Amps per mm of error per second */
/* No D term yet: the laser signal carries noise and 50 Hz pickup. A later D term should act on the
 * measurement (not the error, to avoid a kick on reference steps) through a first-order low-pass. */

/** POS_CONTROLLER_SMC: sliding-mode controller of vca_greybox_fit.ipynb ("Sliding-mode control on the
 *  Method C plant"). With z1 = x - r, z2 = v - r', surface s = a z1 + z2 and model a = f(x, v) + g(x) i:
 *     i = (r'' - a z2 - f(x, v) - eta sat(s / phi)) / g(x)
 *  x and v are the EKF prediction at the cycle the output acts, x(k+1+D|k), v(k+1+D|k) with
 *  D = POS_KF_DELAY_CYCLES, and r is taken at that time, t + (1 + D) cycles. The notches are bypassed. Inside the layer |z1| <= SMC_LAYER_MM; there
 *  the law acts as a PD with stiffness (m / Gamma) eta / SMC_LAYER_MM (about 40 A/mm with the values below),
 *  so SMC_LAYER_MM is the knob to soften it. Sampling limit, checked at startup: (eta / phi) Ts =
 *  eta Ts / (a SMC_LAYER_MM) must stay below 2 (below 1 for no overshoot), so a slow reference (small a)
 *  needs a wider layer: with eta = 15.7 m/s^2, a 5 Hz top frequency needs SMC_LAYER_MM > 0.042 mm. */
#define SMC_A_FREQ_FACTOR   1.0   /**< Surface slope a = factor * 2 pi * highest reference frequency, in 1/s (notebook: 3) */
#define SMC_STEPS_FREQ_HZ   5.0   /**< "Highest frequency" used for a with POS_REF_SHAPE_STEPS, in Hz */
#define SMC_LAYER_MM        0.4  /**< Boundary layer as a position bound phi / a, in mm: phi = a * SMC_LAYER_MM */
#define SMC_ETA_SIGMAS      2.0   /**< Reaching gain eta = SMC_ETA_SIGMAS * POS_KF_SIG_A_M_S2 (model error), in m/s^2 */


/* ---- 3. Position filters --------------------------------------------------------------------- */
/** Order: raw position -> EKF (if enabled) -> notches (if enabled) -> controller. The position trip
 *  always uses the raw, unfiltered position. */

/** Extended Kalman filter: the Method C model of vca_greybox_fit.ipynb (ekf(), state [x, v], measurement
 *  x; parameters in vca_model.h; with POS_KF_MAINS_ENABLE also the 50 Hz pickup, see below), run on the raw
 *  position at the cycle rate. Model input for the step
 *  k-1 -> k is the current sent in cycle k-1-POS_KF_DELAY_CYCLES; the PI uses the filtered estimate x(k|k),
 *  the SMC the prediction at the cycle its output acts (see closed_loop_step()). */
#define POS_KF_ENABLE       1     /**< 1 = controller uses the EKF position x(k|k), 0 = off */
#define POS_KF_SIG_A_M_S2   7.86  /**< Process noise: white acceleration std in m/s^2 (Method C residual on its fit data) */
/** Measurement noise: the part of the laser signal that no state of the filter explains, so it sets how much
 *  the filter trusts each sample. The idle-window laser (0 A) has std 0.078-0.083 mm, of which the 50 Hz
 *  mains line (0.090-0.097 mm amplitude, 0.066 mm rms) is about 70 % of the variance. With
 *  POS_KF_MAINS_ENABLE that line is a filter state, so it must not be counted again as noise: without it
 *  the idle laser has std 0.040-0.046 mm (median 0.0448 over the last 12 position logs). That still holds
 *  the 150 Hz line (about 0.05 mm amplitude, not modelled) and about 0.024 mm white noise. The old value,
 *  0.0766 mm (whole idle laser, notebook), would make the filter trust the laser about 3x too little and
 *  lean on the model, which is 20-40 % off around 50 Hz. Without POS_KF_MAINS_ENABLE the filter adds
 *  POS_KF_MAINS_PICKUP_MM back in (sqrt(SIG_Y^2 + PICKUP^2 / 2), about 0.08 mm). */
#define POS_KF_SIG_Y_MM     0.1 /**< Measurement noise std in mm, without the 50 Hz pickup (idle window) */
#define POS_KF_DELAY_CYCLES 4 /**< Delay from the current sent to the EKF input, in cycles (the EKF sees the current that was sent) */

/** Mains pickup in the EKF: the laser reads 0.09-0.1 mm of 50 Hz pickup even at 0 A. Without a model of it
 *  the EKF passes it into x, v and the SMC (which bypasses the notches) drives about 2 A at 50 Hz, which
 *  really moves the shaft and beats with a reference near 50 Hz (1 Hz amplitude beat at 51 Hz). With this on,
 *  the state is [x, v, c, s]: y = x + c + noise, and [c, s] rotate at POS_KF_MAINS_HZ with a small random walk,
 *  so the filter keeps the pickup out of x, v (a notch of width about POS_KF_MAINS_BW_HZ on the innovation
 *  only). Closed-loop simulation (scripts/ekf_mains_replay.py --sim): 50 Hz current 2.1 -> 0.02 A. */
#define POS_KF_MAINS_ENABLE 1     /**< 1 = model the mains pickup on the laser as a filter state, 0 = off */
#define POS_KF_MAINS_HZ     50.0  /**< Mains frequency in Hz */
#define POS_KF_MAINS_BW_HZ  0.5   /**< Pickup tracking bandwidth in Hz: must cover the mains drift (0.02-0.05 Hz)
                                       and stay well below |reference frequency - POS_KF_MAINS_HZ| */
#define POS_KF_MAINS_PICKUP_MM 0.094 /**< Typical pickup amplitude in mm (idle-window median): initial uncertainty, and noise when off */

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
