/** \file waveforms.h
 * \brief Small waveform helpers shared by the open-loop current experiments and the position reference
 */

#ifndef WAVEFORMS_H
#define WAVEFORMS_H

#include <math.h>

/** \brief One breakpoint of a ramped step table (CHIRP_SCHED, POS_REF_STEPS) */
typedef struct
{
   double t_s;      /**< Time the ramp to value starts, in seconds */
   double value;    /**< Value after the ramp (Amps or mm, depending on the table) */
} breakpoint_t;

/** \brief Value of a breakpoint table at time t: the last breakpoint at or before t, ramped in linearly
 *  from the previous breakpoint's value over ramp_s; the last value holds after the last breakpoint
 *  \param bp Breakpoints, first at t = 0, times increasing
 *  \param n Number of breakpoints (>= 1)
 *  \param ramp_s Ramp time at each breakpoint in seconds; 0 for a hard step
 *  \param t Time in seconds (t >= 0)
 */
static inline double
breakpoint_value(const breakpoint_t *bp, int n, double ramp_s, double t)
{
   int i = 0;
   while (i + 1 < n && bp[i + 1].t_s <= t)
   {
      i++;
   }
   double into_ramp_s = t - bp[i].t_s;
   if (i > 0 && ramp_s > 0.0 && into_ramp_s < ramp_s)
   {
      double prev = bp[i - 1].value;
      return prev + (bp[i].value - prev) * (into_ramp_s / ramp_s);
   }
   return bp[i].value;
}

/** \brief Phase of an exponential sweep f(t) = f0 * (f1 / f0)^(t / T), i.e. equal time per octave.
 *  Shared by the current-mode chirps and the position chirp reference.
 *  \param t Time since the start of the sweep in seconds
 *  \return Phase in radians: the integral of 2 pi f(t), which is 2 pi f0 L (exp(t / L) - 1), L = T / ln(f1 / f0)
 */
static inline double
exp_chirp_phase(double t, double f0, double f1, double T)
{
   const double L = T / log(f1 / f0);
   return 2.0 * M_PI * f0 * L * (exp(t / L) - 1.0);
}

#endif /* WAVEFORMS_H */
