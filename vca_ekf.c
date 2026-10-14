/** \file vca_ekf.c
 * \brief Position EKF: port of ekf() in vca_greybox_fit.ipynb (Method C model, state [x, v],
 *  measurement x, white acceleration process noise). Runs in SI units (m, m/s); mm at the interface.
 *
 * With POS_KF_MAINS_ENABLE the state is [x, v, c, s]: c and s are the in-phase and quadrature parts of the
 * mains pickup on the laser, y = x + c + e. Each step [c, s] is rotated by 2 pi POS_KF_MAINS_HZ Ts plus a
 * small random walk, so it follows a slowly drifting sinusoid; the current does not drive it. The filter
 * then puts the pickup in c, s and keeps it out of x, v. See closed_loop_settings.h section 3.
 */

#include "vca_ekf.h"
#include "vca_model.h"
#include "closed_loop_settings.h"

#define KF_N    (POS_KF_MAINS_ENABLE ? 4 : 2)   /**< State size: x, v (+ pickup c, s) */

static struct
{
   double z[KF_N];              /**< State estimate z(k|k): x in m, v in m/s (, c, s in m) */
   double P[KF_N][KF_N];        /**< Covariance (symmetric) */
   double Q[KF_N][KF_N];        /**< Discretised process noise (block diagonal) */
   double R;                    /**< Measurement variance */
   double rot_c, rot_s;         /**< Pickup rotation per step: cos, sin of 2 pi POS_KF_MAINS_HZ Ts */
   double pickup_var;           /**< Initial variance of c and s, m^2 */
   boolean seeded;              /**< FALSE until the first sample has set the state */
} kf;

/** \brief Compute the noise matrices; call once before the cyclic loop
 *  \param sig_a_m_s2 Process noise: white acceleration std in m/s^2
 *  \param sig_y_mm Measurement noise std in mm, without the mains pickup
 *  \param mains_hz Mains frequency in Hz
 *  \param mains_bw_hz Tracking bandwidth of the pickup estimate in Hz (sets its random walk)
 *  \param mains_pickup_mm Typical pickup amplitude in mm: initial uncertainty of c, s; without the pickup
 *                         model it is lumped into the measurement noise instead
 *  \return Measurement noise std in mm the filter uses
 */
double
vca_ekf_init(double sig_a_m_s2, double sig_y_mm, double mains_hz, double mains_bw_hz, double mains_pickup_mm)
{
   const double ts = CYCLE_TIME_MS / 1000.0;
   const double sa2 = sig_a_m_s2 * sig_a_m_s2;
   for (int i = 0; i < KF_N; i++)
   {
      for (int j = 0; j < KF_N; j++)
      {
         kf.Q[i][j] = 0.0;
      }
   }
   kf.Q[0][0] = sa2 * ts * ts * ts * ts / 4.0;
   kf.Q[0][1] = kf.Q[1][0] = sa2 * ts * ts * ts / 2.0;
   kf.Q[1][1] = sa2 * ts * ts;
   double sig_y = sig_y_mm * 1e-3, pickup = mains_pickup_mm * 1e-3;
#if POS_KF_MAINS_ENABLE
   /* Random walk of c, s per step: a phasor seen through noise sig_y gets a steady-state gain
    * k ~ sqrt(q) / sig_y per step, i.e. a tracking bandwidth k / (2 pi Ts); solve for q at mains_bw_hz. */
   double q_std = 2.0 * M_PI * mains_bw_hz * ts * sig_y;
   kf.Q[2][2] = kf.Q[3][3] = q_std * q_std;
   kf.rot_c = cos(2.0 * M_PI * mains_hz * ts);
   kf.rot_s = sin(2.0 * M_PI * mains_hz * ts);
   kf.pickup_var = pickup * pickup;
   kf.R = sig_y * sig_y;
#else
   (void)mains_hz;
   (void)mains_bw_hz;
   kf.R = sig_y * sig_y + pickup * pickup / 2.0;   /* pickup not modelled: count its power as noise */
#endif
   kf.seeded = FALSE;
   return sqrt(kf.R) * 1e3;
}

/** \brief One RK4 step of the Method C model over CYCLE_TIME_MS with u held over the step
 *  \param x Position in m, updated in place
 *  \param v Velocity in m/s, updated in place
 *  \param u Current in A
 */
static void
kf_rk4(double *x, double *v, double u)
{
   const double ts = CYCLE_TIME_MS / 1000.0;
   double x0 = *x, v0 = *v;
   double k1x = v0, k1v = vca_acc(x0, v0, u);
   double k2x = v0 + ts / 2 * k1v, k2v = vca_acc(x0 + ts / 2 * k1x, v0 + ts / 2 * k1v, u);
   double k3x = v0 + ts / 2 * k2v, k3v = vca_acc(x0 + ts / 2 * k2x, v0 + ts / 2 * k2v, u);
   double k4x = v0 + ts * k3v, k4v = vca_acc(x0 + ts * k3x, v0 + ts * k3v, u);
   *x = x0 + ts / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
   *v = v0 + ts / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
}

/** \brief Start at measurement y (m), at rest, velocity std 1 cm/s (as the notebook), no pickup known yet */
static void
kf_seed(double y)
{
   for (int i = 0; i < KF_N; i++)
   {
      kf.z[i] = 0.0;
      for (int j = 0; j < KF_N; j++)
      {
         kf.P[i][j] = 0.0;
      }
   }
   kf.z[0] = y;
   kf.P[0][0] = kf.R;
   kf.P[1][1] = 1e-4;
#if POS_KF_MAINS_ENABLE
   kf.P[2][2] = kf.P[3][3] = kf.pickup_var;
#endif
   kf.seeded = TRUE;
}

/** \brief Filter one position sample
 *  \param y_mm Raw position in mm, received this cycle
 *  \param u_A Current acting over the step k-1 -> k (sent in cycle k-1-POS_KF_DELAY_CYCLES), held over it
 *  \return x(k|k), v(k|k), pickup c(k|k), innovation and its predicted std
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
      /* Predict: one RK4 step of the model with u held over the step; the pickup rotates */
      kf_rk4(&kf.z[0], &kf.z[1], u_A);
      double F[KF_N][KF_N] = { { 0.0 } };
#if POS_KF_MAINS_ENABLE
      double c = kf.z[2], s = kf.z[3];
      kf.z[2] = kf.rot_c * c - kf.rot_s * s;
      kf.z[3] = kf.rot_s * c + kf.rot_c * s;
      F[2][2] = kf.rot_c;
      F[2][3] = -kf.rot_s;
      F[3][2] = kf.rot_s;
      F[3][3] = kf.rot_c;
#endif

      /* Covariance F P F' + Q. Plant block F = I + A Ts + (A Ts)^2 / 2, A = [[0, 1], [a21, a22]] linearised
       * at the predicted x; a21 = -dFs/dx / m with Fs = spring - Gamma1 x i */
      double gr = VCA_GAP_M - kf.z[0], gl = VCA_GAP_M + kf.z[0];
      double a21 = -(3.0 * VCA_C_R_NM3 / (gr * gr * gr * gr) + 3.0 * VCA_C_L_NM3 / (gl * gl * gl * gl)
                     - VCA_GAMMA1_N_PER_AM * u_A) / VCA_MASS_KG;
      double a22 = -VCA_DAMPING_NS_PER_M / VCA_MASS_KG;
      F[0][0] = 1 + a21 * ts * ts / 2;
      F[0][1] = ts + a22 * ts * ts / 2;
      F[1][0] = a21 * ts + a21 * a22 * ts * ts / 2;
      F[1][1] = 1 + a22 * ts + (a21 + a22 * a22) * ts * ts / 2;
      double G[KF_N][KF_N];      /* F P */
      for (int i = 0; i < KF_N; i++)
      {
         for (int j = 0; j < KF_N; j++)
         {
            double g = 0.0;
            for (int m = 0; m < KF_N; m++)
            {
               g += F[i][m] * kf.P[m][j];
            }
            G[i][j] = g;
         }
      }
      for (int i = 0; i < KF_N; i++)
      {
         for (int j = i; j < KF_N; j++)   /* upper triangle, mirrored: P stays exactly symmetric */
         {
            double p = 0.0;
            for (int m = 0; m < KF_N; m++)
            {
               p += G[i][m] * F[j][m];
            }
            kf.P[i][j] = kf.P[j][i] = p + kf.Q[i][j];
         }
      }
   }

   /* Update with the measured position, H = [1, 0 (, 1, 0)]: PH = P H', S = H P H' + R */
   double PH[KF_N], K[KF_N];
   for (int i = 0; i < KF_N; i++)
   {
      PH[i] = kf.P[i][0];
#if POS_KF_MAINS_ENABLE
      PH[i] += kf.P[i][2];
#endif
   }
   double S = PH[0] + kf.R;
   double e = y - kf.z[0];
#if POS_KF_MAINS_ENABLE
   S += PH[2];
   e -= kf.z[2];
#endif
   for (int i = 0; i < KF_N; i++)
   {
      K[i] = PH[i] / S;
      kf.z[i] += K[i] * e;
   }
   for (int i = 0; i < KF_N; i++)
   {
      for (int j = i; j < KF_N; j++)
      {
         kf.P[i][j] = kf.P[j][i] = kf.P[i][j] - K[i] * PH[j];
      }
   }

   vca_ekf_out_t out = { .x_mm = kf.z[0] * 1e3, .v_mm_s = kf.z[1] * 1e3, .pickup_mm = NAN,
                         .innov_mm = e * 1e3, .innov_std_mm = sqrt(S) * 1e3 };
#if POS_KF_MAINS_ENABLE
   out.pickup_mm = kf.z[2] * 1e3;
#endif
   boolean finite = isfinite(out.innov_std_mm);
   for (int i = 0; i < KF_N; i++)
   {
      finite = finite && isfinite(kf.z[i]) && isfinite(kf.P[i][i]);
   }
   if (!finite)
   {
      /* Cannot happen inside the trip window; restart from the raw sample. */
      kf_seed(y);
      out = (vca_ekf_out_t){ .x_mm = y_mm, .v_mm_s = 0.0, .pickup_mm = NAN, .innov_mm = NAN, .innov_std_mm = NAN };
   }
   return out;
}

/** \brief n-step prediction of the plant from the current estimate, without changing the filter state:
 *  x(k+n|k), v(k+n|k) (the pickup is not part of the motion)
 *  \param u_A Model input for each step, u_A[i] held over the step k+i -> k+i+1
 *  \param n Number of steps
 *  \return Predicted position in mm (x_mm) and velocity in mm/s (v_mm_s); other fields are NaN
 */
vca_ekf_out_t
vca_ekf_predict_n(const double *u_A, int n)
{
   double x = kf.z[0], v = kf.z[1];
   for (int i = 0; i < n; i++)
   {
      kf_rk4(&x, &v, u_A[i]);
   }
   return (vca_ekf_out_t){ .x_mm = x * 1e3, .v_mm_s = v * 1e3, .pickup_mm = NAN, .innov_mm = NAN, .innov_std_mm = NAN };
}
