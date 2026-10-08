/** \file controller_smc.c
 * \brief Sliding-mode position controller (POS_CONTROLLER_SMC): port of smc() in vca_greybox_fit.ipynb
 *  ("Sliding-mode control on the Method C plant"). Runs in SI units (m, m/s); mm at the interface.
 */

#include "controller.h"
#include "closed_loop_position.h"
#include "vca_model.h"

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID && POS_CONTROLLER == POS_CONTROLLER_SMC

_Static_assert((int)(SMC_A_FREQ_FACTOR * 1e6) > 0 && (int)(SMC_STEPS_FREQ_HZ * 1e6) > 0 &&
               (int)(SMC_LAYER_MM * 1e6) > 0 && (int)(SMC_ETA_SIGMAS * 1e6) >= 0,
               "SMC_A_FREQ_FACTOR, SMC_STEPS_FREQ_HZ and SMC_LAYER_MM must be > 0, SMC_ETA_SIGMAS >= 0");

/** \brief Gains, set by controller_setup() */
static double smc_a;            /**< Surface slope a, 1/s */
static double smc_phi;          /**< Boundary layer phi, m/s */
static double smc_eta;          /**< Reaching gain eta, m/s^2 */

boolean
controller_setup(double kp_amps)
{
   const double ts = CYCLE_TIME_MS / 1000.0;
   smc_a = SMC_A_FREQ_FACTOR * 2.0 * M_PI * POS_REF_MAX_FREQ_HZ;
   smc_phi = smc_a * SMC_LAYER_MM * 1e-3;
   smc_eta = SMC_ETA_SIGMAS * POS_KF_SIG_A_M_S2;
   double lambda = smc_eta / smc_phi;                        /* boundary-layer pole of s, rad/s */
   double m_per_gamma = VCA_MASS_KG / VCA_GAMMA_N_PER_A;     /* A per m/s^2 at the centre */
   printf("\nExperiment: position SMC, a = %.1f 1/s (%.1f x 2 pi %.2f Hz), phi = %.4g m/s (|x - r| <= %.3f mm "
          "in the layer), eta = %.2f m/s^2 (%.1f x SIG_A), output limit +/-%.2f A (drive KP %.1f A)\n",
          smc_a, SMC_A_FREQ_FACTOR, POS_REF_MAX_FREQ_HZ, smc_phi, SMC_LAYER_MM, smc_eta, SMC_ETA_SIGMAS,
          OUTPUT_LIMIT_A, kp_amps);
   printf("  in the layer, at the centre: PD with Kp = %.1f A/mm, Kd = %.3f A/(mm/s); s pole eta/phi = %.0f rad/s\n",
          m_per_gamma * smc_a * lambda * 1e-3, m_per_gamma * (smc_a + lambda) * 1e-3, lambda);
   /* The continuous law is sampled at CYCLE_TIME_MS: inside the layer s(k+1) = (1 - lambda Ts) s(k), which
    * overshoots for lambda Ts > 1 and is unstable (chatters between +/- eta) for lambda Ts >= 2. */
   if (smc_a * ts >= 1.0 || lambda * ts >= 2.0)
   {
      printf("POSITION SMC: a Ts = %.3f must be below 1 and (eta / phi) Ts = %.3f below 2; raise SMC_LAYER_MM "
             "or lower SMC_A_FREQ_FACTOR / SMC_ETA_SIGMAS\n", smc_a * ts, lambda * ts);
      return FALSE;
   }
   if (lambda * ts >= 1.0)
   {
      printf("  WARNING: (eta / phi) Ts = %.2f >= 1: the sampled boundary layer overshoots; "
             "raise SMC_LAYER_MM for monotone behaviour\n", lambda * ts);
   }
   return TRUE;
}

/** \brief One SMC update on the predicted state at the time the output acts:
 *  z1 = x - r, z2 = v - r', s = a z1 + z2, i = (r'' - a z2 - f(x, v) - eta sat(s / phi)) / g(x),
 *  with f = vca_acc(x, v, 0) and g = vca_gain(x). Logged: p_A = equivalent control (r'' - a z2 - f) / g,
 *  i_A = switching term -eta sat(s / phi) / g. A non-finite input or result gives 0 A.
 */
pid_log_t
controller_update(const pos_ctrl_in_t *in)
{
   double x = in->x_pred_mm * 1e-3, v = in->v_pred_mm_s * 1e-3;
   double r = in->r_next_mm * 1e-3, rd = in->rd_next_mm_s * 1e-3, rdd = in->rdd_next_mm_s2 * 1e-3;
   double g = vca_gain(x);
   if (!isfinite(x) || !isfinite(v) || !isfinite(r) || !isfinite(rd) || !isfinite(rdd) || !(g > 0.0))
   {
      /* Cannot happen inside the trip window (g > 0 for |x| < Gamma / Gamma1, about 56 mm). */
      return (pid_log_t){ .position_ref_mm = in->r_mm, .p_A = NAN, .i_A = NAN, .output_A = 0.0 };
   }
   double z1 = x - r, z2 = v - rd;
   double s = smc_a * z1 + z2;
   double eq_A = (rdd - smc_a * z2 - vca_acc(x, v, 0.0)) / g;
   double sw_A = -smc_eta * clamp_abs(s / smc_phi, 1.0) / g;
   double u_A = clamp_abs(eq_A + sw_A, OUTPUT_LIMIT_A);
   if (!isfinite(u_A))
   {
      u_A = 0.0;
   }
   return (pid_log_t){ .position_ref_mm = in->r_mm, .p_A = eq_A, .i_A = sw_A, .output_A = u_A };
}

#endif
