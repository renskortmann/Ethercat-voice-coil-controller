/** \file notch.h
 * \brief Biquad notch filters in cascade (RBJ cookbook, bilinear transform), Direct Form II transposed
 */

#ifndef NOTCH_H
#define NOTCH_H

#include "main.h"

/** \brief One notch. DC gain is exactly 1, so a constant input passes unchanged. Set freq_Hz and q,
 *  then call notch_init(). */
typedef struct
{
   double freq_Hz, q;           /**< Centre frequency and quality factor */
   double b0, b1, b2, a1, a2;   /**< Coefficients normalised to a0 = 1 */
   double z1, z2;               /**< Filter state */
   boolean seeded;              /**< FALSE until the first sample has set the state */
} notch_t;

void notch_init(notch_t *notches, int count, double fs_Hz);
double notch_cascade_step(notch_t *notches, int count, double x);

#endif /* NOTCH_H */
