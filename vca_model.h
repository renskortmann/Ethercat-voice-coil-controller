/** \file vca_model.h
 * \brief Fitted voice-coil plant parameters (Method C of vca_greybox_fit.ipynb)
 *
 * m a = (Gamma + Gamma1 x) i - c v - (C_R / (g0 - x)^3 - C_L / (g0 + x)^3), SI units.
 * Not a per-run setting: replace the values only after a new fit (printed by the notebook's
 * Method C cell). Used by the position EKF (vca_ekf.c).
 */

#ifndef VCA_MODEL_H
#define VCA_MODEL_H

#define VCA_MASS_KG          51.5     /**< Moving mass m, kg */
#define VCA_GAMMA_N_PER_A    200.9    /**< Motor constant at the centre Gamma, N/A */
#define VCA_GAMMA1_N_PER_AM  3573.0   /**< Motor constant slope Gamma1, N/(A m) */
#define VCA_DAMPING_NS_PER_M 1586.3   /**< Viscous friction c, N s/m */
#define VCA_C_R_NM3          0.001604 /**< Right magnet strength C_R, N m^3 */
#define VCA_C_L_NM3          0.001574 /**< Left magnet strength C_L, N m^3 */
#define VCA_GAP_M            22.3e-3  /**< Magnet gap g0, m */

#endif /* VCA_MODEL_H */
