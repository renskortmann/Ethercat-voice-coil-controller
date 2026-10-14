/** \file closed_loop_position.h
 * \brief Closed-loop position control (EXPERIMENT_POSITION_PID): the loop around the controller
 *
 * Per cycle: raw position -> trip check -> EKF -> notches -> controller (controller_*.c) -> output limit.
 * The SMC takes the EKF prediction at the cycle its output acts instead and skips the notches.
 * Settings in closed_loop_settings.h.
 */

#ifndef CLOSED_LOOP_POSITION_H
#define CLOSED_LOOP_POSITION_H

#include "main.h"
#include "closed_loop_settings.h"

/** \brief Notches in the controller path: POS_NOTCH_ENABLE, except for the SMC, which bypasses them */
#define POS_NOTCH_ACTIVE    (POS_NOTCH_ENABLE && POS_CONTROLLER != POS_CONTROLLER_SMC)

/** \brief The chirp shapes, which share the POS_REF_CHIRP_* settings */
#define POS_REF_IS_CHIRP    (POS_REF_SHAPE == POS_REF_SHAPE_CHIRP || POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD)

/** \brief Highest frequency in the reference, in Hz (sets the SMC surface slope a) */
#if POS_REF_IS_CHIRP
#define POS_REF_MAX_FREQ_HZ fmax(POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ)
#elif POS_REF_SHAPE == POS_REF_SHAPE_SINE
#define POS_REF_MAX_FREQ_HZ POS_REF_SINE_FREQ_HZ
#else
#define POS_REF_MAX_FREQ_HZ SMC_STEPS_FREQ_HZ
#endif

boolean closed_loop_setup(double kp_amps);
boolean closed_loop_trip_update(double position_mm);
pid_log_t closed_loop_step(double t_s, double position_mm, double sent_current_A);

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
/** \brief File-name tag, built from the settings: "pos" + controller name + "_" + reference + controller
 *  gains + filters, e.g. posPI_chirp_0.0mm_3.0mm_1.0to5.0Hz_60.0s_hold2.0s_taper1.0s_kp1.0_ki0.12_notch50.0Hz_Q10.0_kfd4
 *  (_kf with POS_KF_DELAY_CYCLES 0, _kfd<D> otherwise; m appended with POS_KF_MAINS_ENABLE, e.g. _kfd4m) */
#if POS_CONTROLLER == POS_CONTROLLER_PI
#define POS_CTRL_NAME       "PI"
#define POS_CTRL_TAG        "_kp" STRINGIFY(PID_KP_A_PER_MM) "_ki" STRINGIFY(PID_KI_A_PER_MM_S)
#elif POS_CONTROLLER == POS_CONTROLLER_SMC
#define POS_CTRL_NAME       "SMC"
#define POS_CTRL_TAG        "_a" STRINGIFY(SMC_A_FREQ_FACTOR) "x_phi" STRINGIFY(SMC_LAYER_MM) "mm_eta" \
                            STRINGIFY(SMC_ETA_SIGMAS) "sig"
#else
#error "Unknown POS_CONTROLLER"
#endif

/** POS_REF_STEPS as <time>s<position>mm per breakpoint, e.g. "0.0s0.0mm-8.0s1.0mm-" */
#define POS_REF_STEPS_LABEL_(t, p)  STRINGIFY(t) "s" STRINGIFY(p) "mm-"
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
#define POS_REF_TAG         "steps_" POS_REF_STEPS(POS_REF_STEPS_LABEL_) "r" STRINGIFY(POS_REF_RAMP_S) "s"
#elif POS_REF_SHAPE == POS_REF_SHAPE_CHIRP
#define POS_REF_TAG         "chirp_" STRINGIFY(POS_REF_CHIRP_OFFSET_MM) "mm_" \
                            STRINGIFY(POS_REF_CHIRP_AMPLITUDE_MM) "mm_" STRINGIFY(POS_REF_CHIRP_F0_HZ) "to" \
                            STRINGIFY(POS_REF_CHIRP_F1_HZ) "Hz_" STRINGIFY(POS_REF_CHIRP_DURATION_S) "s_hold" \
                            STRINGIFY(POS_REF_CHIRP_START_S) "s_taper" STRINGIFY(POS_REF_CHIRP_TAPER_S) "s"
#elif POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD
#define POS_REF_TAG         "chirphold_" STRINGIFY(POS_REF_CHIRP_OFFSET_MM) "mm_" \
                            STRINGIFY(POS_REF_CHIRP_AMPLITUDE_MM) "mm_" STRINGIFY(POS_REF_CHIRP_F0_HZ) "to" \
                            STRINGIFY(POS_REF_CHIRP_F1_HZ) "Hz_" STRINGIFY(POS_REF_CHIRP_DURATION_S) "s_hold" \
                            STRINGIFY(POS_REF_CHIRP_START_S) "s_taper" STRINGIFY(POS_REF_CHIRP_TAPER_S) "s_f1hold" \
                            STRINGIFY(POS_REF_CHIRP_F1_HOLD_S) "s"
#elif POS_REF_SHAPE == POS_REF_SHAPE_SINE
#define POS_REF_TAG         "sine_" STRINGIFY(POS_REF_SINE_OFFSET_MM) "mm_" \
                            STRINGIFY(POS_REF_SINE_AMPLITUDE_MM) "mm_" STRINGIFY(POS_REF_SINE_FREQ_HZ) "Hz"
#else
#error "Unknown POS_REF_SHAPE"
#endif

#if POS_NOTCH_ACTIVE
#define POS_NOTCHES_LABEL_(f, q)    "_notch" STRINGIFY(f) "Hz_Q" STRINGIFY(q)
#define POS_NOTCH_TAG       POS_NOTCHES(POS_NOTCHES_LABEL_)
#else
#define POS_NOTCH_TAG       ""
#endif
#if POS_KF_ENABLE && POS_KF_MAINS_ENABLE
#define POS_KF_MAINS_TAG    "m"
#else
#define POS_KF_MAINS_TAG    ""
#endif
#if POS_KF_ENABLE && POS_KF_DELAY_CYCLES > 0
#define POS_KF_TAG          "_kfd" STRINGIFY(POS_KF_DELAY_CYCLES) POS_KF_MAINS_TAG
#elif POS_KF_ENABLE
#define POS_KF_TAG          "_kf" POS_KF_MAINS_TAG
#else
#define POS_KF_TAG          ""
#endif

#define EXPERIMENT_TAG      "pos" POS_CTRL_NAME "_" POS_REF_TAG POS_CTRL_TAG POS_NOTCH_TAG POS_KF_TAG
#endif /* EXPERIMENT_MODE == EXPERIMENT_POSITION_PID */

#endif /* CLOSED_LOOP_POSITION_H */
