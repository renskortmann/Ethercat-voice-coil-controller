/** \file controller_pi.c
 * \brief PI position controller (POS_CONTROLLER_PI)
 */

#include "controller.h"

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID && POS_CONTROLLER == POS_CONTROLLER_PI

_Static_assert((int)(PID_KP_A_PER_MM * 1e6) >= 0 && (int)(PID_KI_A_PER_MM_S * 1e6) >= 0,
               "PID gains must be >= 0 (a negative gain is positive feedback)");

/** \brief PI integrator in Amps. Reset by controller_setup() and only updated from t = 0, so the idle
 *  window leaves it at 0. */
static double pi_integrator_A = 0.0;

boolean
controller_setup(double kp_amps)
{
   printf("\nExperiment: position PI, Kp = %.4f A/mm, Ki = %.4f A/(mm s), output limit +/-%.2f A (drive KP %.1f A)\n",
          PID_KP_A_PER_MM, PID_KI_A_PER_MM_S, OUTPUT_LIMIT_A, kp_amps);
   pi_integrator_A = 0.0;
   return TRUE;
}

/** \brief One PI update: e = r - y, P = Kp e, I += Ki e dt, u = sat(P + I)
 *  Anti-windup by conditional integration: while P + I is beyond the output limit, integrator steps
 *  that would push it further out are dropped. The integrator is also clamped to the output limit.
 *  A non-finite input or result (which a valid AI1 reading cannot produce) gives 0 A.
 */
pid_log_t
controller_update(const pos_ctrl_in_t *in)
{
   const double dt_s = CYCLE_TIME_MS / 1000.0;
   const double r_mm = in->r_mm, y_mm = in->y_mm;
   if (!isfinite(r_mm) || !isfinite(y_mm))
   {
      /* Cannot happen with a valid int16 AI1 reading; command 0 A and leave the integrator alone. */
      return (pid_log_t){ .position_ref_mm = r_mm, .p_A = 0.0, .i_A = pi_integrator_A, .output_A = 0.0 };
   }
   double e_mm = r_mm - y_mm;
   double p_A = PID_KP_A_PER_MM * e_mm;
   double i_A = pi_integrator_A + PID_KI_A_PER_MM_S * e_mm * dt_s;
   double unsat_A = p_A + i_A;
   if ((unsat_A > OUTPUT_LIMIT_A && e_mm > 0.0) || (unsat_A < -OUTPUT_LIMIT_A && e_mm < 0.0))
   {
      i_A = pi_integrator_A;                                 /* saturated: hold the integrator */
   }
   i_A = clamp_abs(i_A, OUTPUT_LIMIT_A);
   if (!isfinite(i_A))
   {
      i_A = 0.0;
   }
   pi_integrator_A = i_A;

   double u_A = clamp_abs(p_A + i_A, OUTPUT_LIMIT_A);
   if (!isfinite(u_A))
   {
      u_A = 0.0;
   }
   return (pid_log_t){ .position_ref_mm = r_mm, .p_A = p_A, .i_A = i_A, .output_A = u_A };
}

#endif
