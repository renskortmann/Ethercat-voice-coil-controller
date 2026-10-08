/** \file vca_ekf.c
 * \brief Position EKF: port of ekf() in vca_greybox_fit.ipynb (Method C model, state [x, v],
 *  measurement x, white acceleration process noise). Runs in SI units (m, m/s); mm at the interface.
 */

#include "vca_ekf.h"
#include "vca_model.h"

static struct
{
   double x, v;                 /**< State estimate x(k|k) in m, v(k|k) in m/s */
   double p11, p12, p22;        /**< Covariance P (symmetric) */
   double q11, q12, q22, R;     /**< Discretised process noise and measurement variance */
   boolean seeded;              /**< FALSE until the first sample has set the state */
} kf;

/** \brief Compute the noise matrices; call once before the cyclic loop
 *  \param sig_a_m_s2 Process noise: white acceleration std in m/s^2
 *  \param sig_y_mm Measurement noise std in mm
 */
void
vca_ekf_init(double sig_a_m_s2, double sig_y_mm)
{
   const double ts = CYCLE_TIME_MS / 1000.0;
   const double sa2 = sig_a_m_s2 * sig_a_m_s2;
   kf.q11 = sa2 * ts * ts * ts * ts / 4.0;
   kf.q12 = sa2 * ts * ts * ts / 2.0;
   kf.q22 = sa2 * ts * ts;
   kf.R = (sig_y_mm * 1e-3) * (sig_y_mm * 1e-3);
   kf.seeded = FALSE;
}

/** \brief Method C acceleration in m/s^2: ((Gamma + Gamma1 x) i - c v - (C_R / (g0 - x)^3 - C_L / (g0 + x)^3)) / m */
static double
kf_acc(double x, double v, double u)
{
   double gr = VCA_GAP_M - x, gl = VCA_GAP_M + x;
   double spring = VCA_C_R_NM3 / (gr * gr * gr) - VCA_C_L_NM3 / (gl * gl * gl);
   return ((VCA_GAMMA_N_PER_A + VCA_GAMMA1_N_PER_AM * x) * u - VCA_DAMPING_NS_PER_M * v - spring) / VCA_MASS_KG;
}

/** \brief Start at measurement y (m), at rest, velocity std 1 cm/s (as the notebook) */
static void
kf_seed(double y)
{
   kf.x = y;
   kf.v = 0.0;
   kf.p11 = kf.R;
   kf.p12 = 0.0;
   kf.p22 = 1e-4;
   kf.seeded = TRUE;
}

/** \brief Filter one position sample
 *  \param y_mm Raw position in mm, received this cycle
 *  \param u_A Current sent in the previous cycle, held over the step k-1 -> k
 *  \return x(k|k), v(k|k), innovation and its predicted std
 */
vca_ekf_out_t
vca_ekf_step(double y_mm, double u_A)
{
   const double ts = CYCLE_TIME_MS / 1000.0;
   double y = y_mm * 1e-3;
   if (!kf.seeded)
   {
      kf_seed(y);                /* first sample: no predict step */
   }
   else
   {
      /* Predict: one RK4 step of the model with u held over the step */
      double x = kf.x, v = kf.v;
      double k1x = v, k1v = kf_acc(x, v, u_A);
      double k2x = v + ts / 2 * k1v, k2v = kf_acc(x + ts / 2 * k1x, v + ts / 2 * k1v, u_A);
      double k3x = v + ts / 2 * k2v, k3v = kf_acc(x + ts / 2 * k2x, v + ts / 2 * k2v, u_A);
      double k4x = v + ts * k3v, k4v = kf_acc(x + ts * k3x, v + ts * k3v, u_A);
      kf.x = x + ts / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
      kf.v = v + ts / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);

      /* Covariance F P F' + Q with F = I + A Ts + (A Ts)^2 / 2, A = [[0, 1], [a21, a22]] linearised at
       * the predicted x; a21 = -dFs/dx / m with Fs = spring - Gamma1 x i */
      double gr = VCA_GAP_M - kf.x, gl = VCA_GAP_M + kf.x;
      double a21 = -(3.0 * VCA_C_R_NM3 / (gr * gr * gr * gr) + 3.0 * VCA_C_L_NM3 / (gl * gl * gl * gl)
                     - VCA_GAMMA1_N_PER_AM * u_A) / VCA_MASS_KG;
      double a22 = -VCA_DAMPING_NS_PER_M / VCA_MASS_KG;
      double f11 = 1 + a21 * ts * ts / 2, f12 = ts + a22 * ts * ts / 2;
      double f21 = a21 * ts + a21 * a22 * ts * ts / 2, f22 = 1 + a22 * ts + (a21 + a22 * a22) * ts * ts / 2;
      double g11 = f11 * kf.p11 + f12 * kf.p12, g12 = f11 * kf.p12 + f12 * kf.p22;
      double g21 = f21 * kf.p11 + f22 * kf.p12, g22 = f21 * kf.p12 + f22 * kf.p22;
      kf.p11 = g11 * f11 + g12 * f12 + kf.q11;
      kf.p12 = g11 * f21 + g12 * f22 + kf.q12;
      kf.p22 = g21 * f21 + g22 * f22 + kf.q22;
   }

   /* Update with the measured position */
   double S = kf.p11 + kf.R;
   double k1 = kf.p11 / S, k2 = kf.p12 / S, e = y - kf.x;
   kf.x += k1 * e;
   kf.v += k2 * e;
   double p11 = kf.p11, p12 = kf.p12;
   kf.p11 = p11 - k1 * p11;
   kf.p12 = p12 - k1 * p12;
   kf.p22 = kf.p22 - k2 * p12;

   vca_ekf_out_t out = { .x_mm = kf.x * 1e3, .v_mm_s = kf.v * 1e3, .innov_mm = e * 1e3, .innov_std_mm = sqrt(S) * 1e3 };
   if (!isfinite(kf.x) || !isfinite(kf.v) || !isfinite(kf.p11) || !isfinite(kf.p12) || !isfinite(kf.p22)
       || !isfinite(out.innov_std_mm))
   {
      /* Cannot happen inside the trip window; restart from the raw sample. */
      kf_seed(y);
      out = (vca_ekf_out_t){ .x_mm = y_mm, .v_mm_s = 0.0, .innov_mm = NAN, .innov_std_mm = NAN };
   }
   return out;
}
