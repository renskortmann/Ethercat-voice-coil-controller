/** \file vca_ekf.h
 * \brief Position EKF on the Method C voice-coil model (vca_model.h)
 */

#ifndef VCA_EKF_H
#define VCA_EKF_H

#include "main.h"

/** \brief One cycle's EKF output, in mm */
typedef struct
{
   double x_mm;                 /**< Filtered position x(k|k) */
   double v_mm_s;               /**< Filtered velocity v(k|k) */
   double pickup_mm;            /**< Mains pickup estimate c(k|k) (NaN without POS_KF_MAINS_ENABLE) */
   double innov_mm;             /**< Innovation y(k) - x(k|k-1) (- c(k|k-1)) */
   double innov_std_mm;         /**< Predicted innovation std sqrt(S) */
} vca_ekf_out_t;

double vca_ekf_init(double sig_a_m_s2, double sig_y_mm, double mains_hz, double mains_bw_hz, double mains_pickup_mm);
vca_ekf_out_t vca_ekf_step(double y_mm, double u_A);
vca_ekf_out_t vca_ekf_predict_n(const double *u_A, int n);

#endif /* VCA_EKF_H */
