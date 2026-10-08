/** \file notch.c
 * \brief Biquad notch filters in cascade
 */

#include "notch.h"

/** \brief Compute the coefficients of every notch from its freq_Hz and q; call once before the cyclic loop
 *  \param notches Notches with freq_Hz and q set
 *  \param count Number of notches
 *  \param fs_Hz Sample rate in Hz
 */
void
notch_init(notch_t *notches, int count, double fs_Hz)
{
   for (int i = 0; i < count; i++)
   {
      notch_t *n = &notches[i];
      double w0 = 2.0 * M_PI * n->freq_Hz / fs_Hz;
      double alpha = sin(w0) / (2.0 * n->q);
      double a0 = 1.0 + alpha;
      n->b0 = 1.0 / a0;
      n->b1 = -2.0 * cos(w0) / a0;
      n->b2 = 1.0 / a0;
      n->a1 = -2.0 * cos(w0) / a0;
      n->a2 = (1.0 - alpha) / a0;
      n->seeded = FALSE;
   }
}

/** \brief Set the state as if the input had been constant at x forever, so the output starts at x
 *  without a transient */
static void
notch_seed(notch_t *n, double x)
{
   n->z2 = (n->b2 - n->a2) * x;
   n->z1 = (n->b1 - n->a1) * x + n->z2;
   n->seeded = TRUE;
}

/** \brief Filter one sample through every notch in turn
 *  \param notches Notches initialised by notch_init()
 *  \param count Number of notches
 *  \param x Input sample
 *  \return Filtered sample
 */
double
notch_cascade_step(notch_t *notches, int count, double x)
{
   for (int i = 0; i < count; i++)
   {
      notch_t *n = &notches[i];
      if (!n->seeded)
      {
         notch_seed(n, x);
      }
      double y = n->b0 * x + n->z1;
      n->z1 = n->b1 * x - n->a1 * y + n->z2;
      n->z2 = n->b2 * x - n->a2 * y;
      if (!isfinite(y))
      {
         /* Cannot happen with a valid int16 AI1 reading; restart from this notch's input. */
         notch_seed(n, x);
         y = x;
      }
      x = y;
   }
   return x;
}
