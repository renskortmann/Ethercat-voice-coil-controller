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
 * file-name tag to closed_loop_position.h.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "main.h"
#include "closed_loop_settings.h"

/** \brief Print the controller and its gains, check them, reset the controller state; called once
 *  before the cyclic loop
 *  \param kp_amps Drive peak current
 *  \return TRUE if usable, FALSE (with a message printed) otherwise
 */
boolean controller_setup(double kp_amps);

/** \brief One controller update, called every cycle from t = 0 (not during the idle window)
 *  \param r_mm Reference position in mm
 *  \param y_mm Filtered position in mm (from this cycle's PDO exchange)
 *  \return position_ref_mm, p_A, i_A (the controller's own terms, or NaN if it has none) and
 *          output_A, which is sent to the drive in the next cycle
 */
pid_log_t controller_update(double r_mm, double y_mm);

/** \brief Clamp x to [-limit, limit] (NaN passes through; callers check isfinite) */
static inline double
clamp_abs(double x, double limit)
{
   if (x > limit) return limit;
   if (x < -limit) return -limit;
   return x;
}

#endif /* CONTROLLER_H */
