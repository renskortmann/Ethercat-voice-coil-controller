/** \file closed_loop_position.h
 * \brief Closed-loop position control (EXPERIMENT_POSITION_PID): the loop around the controller
 *
 * Per cycle: raw position -> trip check -> EKF -> notches -> controller (controller_*.c) -> output limit.
 * Settings in closed_loop_settings.h.
 */

#ifndef CLOSED_LOOP_POSITION_H
#define CLOSED_LOOP_POSITION_H

#include "main.h"
#include "closed_loop_settings.h"

boolean closed_loop_setup(double kp_amps);
boolean closed_loop_trip_update(double position_mm);
pid_log_t closed_loop_step(double t_s, double position_mm, double sent_current_A);

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID
/** \brief File-name tag, built from the settings: "pos" + controller name + "_" + reference + controller
 *  gains + filters, e.g. posPI_chirp_0.0mm_3.0mm_1.0to5.0Hz_60.0s_hold2.0s_kp1.0_ki0.12_notch50.0Hz_Q10.0_kf */
#if POS_CONTROLLER == POS_CONTROLLER_PI
#define POS_CTRL_NAME       "PI"
#define POS_CTRL_TAG        "_kp" STRINGIFY(PID_KP_A_PER_MM) "_ki" STRINGIFY(PID_KI_A_PER_MM_S)
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
                            STRINGIFY(POS_REF_CHIRP_START_S) "s"
#elif POS_REF_SHAPE == POS_REF_SHAPE_SINE
#define POS_REF_TAG         "sine_" STRINGIFY(POS_REF_SINE_OFFSET_MM) "mm_" \
                            STRINGIFY(POS_REF_SINE_AMPLITUDE_MM) "mm_" STRINGIFY(POS_REF_SINE_FREQ_HZ) "Hz"
#else
#error "Unknown POS_REF_SHAPE"
#endif

#if POS_NOTCH_ENABLE
#define POS_NOTCHES_LABEL_(f, q)    "_notch" STRINGIFY(f) "Hz_Q" STRINGIFY(q)
#define POS_NOTCH_TAG       POS_NOTCHES(POS_NOTCHES_LABEL_)
#else
#define POS_NOTCH_TAG       ""
#endif
#if POS_KF_ENABLE
#define POS_KF_TAG          "_kf"
#else
#define POS_KF_TAG          ""
#endif

#define EXPERIMENT_TAG      "pos" POS_CTRL_NAME "_" POS_REF_TAG POS_CTRL_TAG POS_NOTCH_TAG POS_KF_TAG
#endif /* EXPERIMENT_MODE == EXPERIMENT_POSITION_PID */

#endif /* CLOSED_LOOP_POSITION_H */
