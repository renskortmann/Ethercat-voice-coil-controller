/** \file controller.h
 * \brief Interface every position controller implements (one controller_<name>.c per controller)
 *
 * Each controller_<name>.c wraps its body in #if POS_CONTROLLER == POS_CONTROLLER_<NAME>, so exactly
 * one of them is compiled in. closed_loop_position.c calls these functions without knowing which
 * controller it is, and applies the shared safety (output limit, non-finite guard, position trip) to
 * whatever it returns.
 *
 * Adding a controller: add POS_CONTROLLER_<NAME> and a settings section to closed_loop_settings.h, copy
 * controller_pi.c to controller_<name>.c and change the maths, add it to CMakeLists.txt, and add its
 * file-name tag to closed_loop_position.h. Each controller reads the fields of pos_ctrl_in_t it needs.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "main.h"
#include "closed_loop_settings.h"

/** \brief Controller inputs for one cycle, built by closed_loop_step() */
typedef struct
{
   double r_mm;                 /**< Reference at t, in mm (logged) */
   double y_mm;                 /**< Filtered position in mm: EKF x(k|k) and/or notches, as POS_KF/NOTCH_ENABLE */
   double r_next_mm;            /**< Reference when the output acts, t + (1 + POS_KF_DELAY_CYCLES) cycles, in mm */
   double rd_next_mm_s;         /**< Its first derivative, in mm/s */
   double rdd_next_mm_s2;       /**< Its second derivative, in mm/s^2 */
   double x_pred_mm;            /**< EKF prediction x(k+1+D|k) at that time, in mm (NaN without POS_KF_ENABLE) */
   double v_pred_mm_s;          /**< EKF prediction v(k+1+D|k), in mm/s (NaN without POS_KF_ENABLE) */
} pos_ctrl_in_t;

/** \brief Print the controller and its gains, check them, reset the controller state; called once
 *  before the cyclic loop
 *  \param kp_amps Drive peak current
 *  \return TRUE if usable, FALSE (with a message printed) otherwise
 */
boolean controller_setup(double kp_amps);

/** \brief One controller update, called every cycle from t = 0 (not during the idle window)
 *  \param in References and position estimates from this cycle's PDO exchange
 *  \return position_ref_mm, p_A, i_A (the controller's own terms, or NaN if it has none) and
 *          output_A, which is sent to the drive in the next cycle
 */
pid_log_t controller_update(const pos_ctrl_in_t *in);

/** \brief Clamp x to [-limit, limit] (NaN passes through; callers check isfinite) */
static inline double
clamp_abs(double x, double limit)
{
   if (x > limit) return limit;
   if (x < -limit) return -limit;
   return x;
}

#endif /* CONTROLLER_H */
