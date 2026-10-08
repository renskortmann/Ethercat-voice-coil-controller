/** \file closed_loop_position.c
 * \brief Closed-loop position control: reference generator, position filters, trip, and the shared
 *  safety around the controller selected by POS_CONTROLLER (controller_*.c)
 */

#include "closed_loop_position.h"
#include "controller.h"
#include "notch.h"
#include "vca_ekf.h"
#include "vca_model.h"
#include "waveforms.h"

#if EXPERIMENT_MODE == EXPERIMENT_POSITION_PID

#define MS_(x)  ((int)((x) * 1000))   /* seconds/mm/Hz -> integer thousandths, for _Static_assert */

_Static_assert(MS_(POS_TRIP_MIN_MM) <= MS_(POS_REF_MIN_MM) && MS_(POS_REF_MAX_MM) <= MS_(POS_TRIP_MAX_MM),
               "POS_REF_MIN/MAX_MM must lie inside the POS_TRIP_MIN/MAX_MM window");
_Static_assert(MS_(OUTPUT_LIMIT_A) > 0, "OUTPUT_LIMIT_A must be > 0");
_Static_assert(POS_TRIP_CYCLES >= 1, "POS_TRIP_CYCLES must be >= 1");
_Static_assert(POS_NOTCH_ENABLE == 0 || POS_NOTCH_ENABLE == 1, "POS_NOTCH_ENABLE must be 0 or 1");
#define POS_NOTCH_VALID_(f, q)      && MS_(f) > 0 && (int)((f) * CYCLE_TIME_MS) < 500 && MS_(q) > 0
_Static_assert(1 POS_NOTCHES(POS_NOTCH_VALID_),
               "POS_NOTCHES: each frequency must be > 0 and below the Nyquist frequency (500 / CYCLE_TIME_MS Hz), each Q > 0");
_Static_assert(POS_KF_ENABLE == 0 || POS_KF_ENABLE == 1, "POS_KF_ENABLE must be 0 or 1");
_Static_assert(POS_CONTROLLER != POS_CONTROLLER_SMC || POS_KF_ENABLE,
               "POS_CONTROLLER_SMC needs POS_KF_ENABLE 1 (it uses the EKF position and velocity)");
_Static_assert((int)(POS_KF_SIG_A_M_S2 * 1e6) > 0 && (int)(POS_KF_SIG_Y_MM * 1e6) > 0 && MS_(VCA_MASS_KG) > 0,
               "POS_KF_SIG_A_M_S2, POS_KF_SIG_Y_MM and VCA_MASS_KG must be > 0");
_Static_assert(POS_KF_DELAY_CYCLES >= 0 && POS_KF_DELAY_CYCLES <= 16, "POS_KF_DELAY_CYCLES must be 0 .. 16");
_Static_assert(POS_KF_MAINS_ENABLE == 0 || POS_KF_MAINS_ENABLE == 1, "POS_KF_MAINS_ENABLE must be 0 or 1");
_Static_assert(MS_(POS_KF_MAINS_HZ) > 0 && (int)(POS_KF_MAINS_HZ * CYCLE_TIME_MS) < 500 && MS_(POS_KF_MAINS_BW_HZ) > 0 &&
               MS_(POS_KF_MAINS_PICKUP_MM) >= 0,
               "POS_KF_MAINS_HZ must be > 0 and below Nyquist, POS_KF_MAINS_BW_HZ > 0, POS_KF_MAINS_PICKUP_MM >= 0");
_Static_assert(MS_(POS_TRIP_MAX_MM) < (int)(VCA_GAP_M * 1e6) && MS_(-POS_TRIP_MIN_MM) < (int)(VCA_GAP_M * 1e6),
               "POS_TRIP_MIN/MAX_MM must stay inside the magnet gap VCA_GAP_M (the EKF model has a pole there)");
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
#define POS_REF_IN_WINDOW_(t, p)    && MS_(p) >= MS_(POS_REF_MIN_MM) && MS_(p) <= MS_(POS_REF_MAX_MM)
_Static_assert(1 POS_REF_STEPS(POS_REF_IN_WINDOW_),
               "POS_REF_STEPS position outside POS_REF_MIN_MM .. POS_REF_MAX_MM");
#elif POS_REF_SHAPE == POS_REF_SHAPE_SINE
_Static_assert(MS_(POS_REF_SINE_OFFSET_MM - POS_REF_SINE_AMPLITUDE_MM) >= MS_(POS_REF_MIN_MM) &&
               MS_(POS_REF_SINE_OFFSET_MM + POS_REF_SINE_AMPLITUDE_MM) <= MS_(POS_REF_MAX_MM),
               "POS_REF_SINE offset +/- amplitude outside POS_REF_MIN_MM .. POS_REF_MAX_MM");
_Static_assert(MS_(POS_REF_SINE_FREQ_HZ) > 0 && (int)(POS_REF_SINE_FREQ_HZ * CYCLE_TIME_MS) <= 100,
               "POS_REF_SINE_FREQ_HZ must be > 0 and give at least 10 samples per period");
#elif POS_REF_IS_CHIRP
_Static_assert(MS_(POS_REF_CHIRP_OFFSET_MM - POS_REF_CHIRP_AMPLITUDE_MM) >= MS_(POS_REF_MIN_MM) &&
               MS_(POS_REF_CHIRP_OFFSET_MM + POS_REF_CHIRP_AMPLITUDE_MM) <= MS_(POS_REF_MAX_MM),
               "POS_REF_CHIRP offset +/- amplitude outside POS_REF_MIN_MM .. POS_REF_MAX_MM");
_Static_assert(MS_(POS_REF_CHIRP_F0_HZ) > 0 && MS_(POS_REF_CHIRP_F0_HZ) != MS_(POS_REF_CHIRP_F1_HZ),
               "POS_REF_CHIRP_F0_HZ must be > 0 and differ from POS_REF_CHIRP_F1_HZ");
_Static_assert((int)(POS_REF_CHIRP_F0_HZ * CYCLE_TIME_MS) <= 100 && (int)(POS_REF_CHIRP_F1_HZ * CYCLE_TIME_MS) <= 100,
               "POS_REF_CHIRP frequencies must give at least 10 samples per period");
_Static_assert(MS_(POS_REF_CHIRP_DURATION_S) > 0 && MS_(POS_REF_CHIRP_START_S) >= 0,
               "POS_REF_CHIRP_DURATION_S must be > 0 and POS_REF_CHIRP_START_S >= 0");
_Static_assert(MS_(POS_REF_CHIRP_TAPER_S) >= 0 && MS_(2.0 * POS_REF_CHIRP_TAPER_S) <= MS_(POS_REF_CHIRP_DURATION_S),
               "POS_REF_CHIRP_TAPER_S must be >= 0 and the fade-in plus fade-out must fit in POS_REF_CHIRP_DURATION_S");
#if POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD
_Static_assert(MS_(POS_REF_CHIRP_F1_HOLD_S) >= 0, "POS_REF_CHIRP_F1_HOLD_S must be >= 0");
/** \brief Time after the sweep: the hold at f1 and the fade-out */
#define POS_REF_CHIRP_AFTER_S   (POS_REF_CHIRP_F1_HOLD_S + POS_REF_CHIRP_TAPER_S)
#else
#define POS_REF_CHIRP_AFTER_S   0.0
#endif
/* The sweep ends at the next zero crossing after T (+ hold and fade-out), at most half a period of f1 later;
 * 0.1 s covers f1 >= 5 Hz. */
_Static_assert(MS_(POS_REF_CHIRP_START_S + POS_REF_CHIRP_DURATION_S + POS_REF_CHIRP_AFTER_S + 0.1) <= MS_(RUN_DURATION_S),
               "POS_REF_CHIRP_START_S + POS_REF_CHIRP_DURATION_S (+ F1_HOLD + TAPER for CHIRP_HOLD, +0.1 s) must fit in RUN_DURATION_S");
#endif

/* ---- Reference ------------------------------------------------------------------------------- */

#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
/** \brief Reference breakpoints (value in mm) from POS_REF_STEPS, validated by pos_ref_check() */
#define POS_REF_STEPS_ENTRY_(t, p)  {t, p},
static const breakpoint_t pos_ref_steps[] = { POS_REF_STEPS(POS_REF_STEPS_ENTRY_) };
#define POS_REF_STEPS_N ((int)(sizeof(pos_ref_steps) / sizeof(pos_ref_steps[0])))
#endif

#if POS_REF_IS_CHIRP
/** \brief Chirp phase and its derivatives at sweep time tau: the exponential sweep, and for
 *  POS_REF_SHAPE_CHIRP_HOLD after T a constant f1, continuing from the sweep's phase at T
 *  \param tau Sweep time in seconds (>= 0)
 *  \param ph Phase in rad; \param dph Its derivative 2 pi f in rad/s; \param ddph Its second derivative in rad/s^2
 */
static void
pos_ref_chirp_phase(double tau, double *ph, double *dph, double *ddph)
{
   const double T = POS_REF_CHIRP_DURATION_S;
   const double L = T / log(POS_REF_CHIRP_F1_HZ / POS_REF_CHIRP_F0_HZ);
#if POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD
   if (tau >= T)
   {
      *dph = 2.0 * M_PI * POS_REF_CHIRP_F1_HZ;
      *ph = exp_chirp_phase(T, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ, T) + *dph * (tau - T);
      *ddph = 0.0;
      return;
   }
#endif
   /* phase' = 2 pi f(tau) = 2 pi f0 exp(tau / L), phase'' = phase' / L */
   *ph = exp_chirp_phase(tau, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ, T);
   *dph = 2.0 * M_PI * POS_REF_CHIRP_F0_HZ * exp(tau / L);
   *ddph = *dph / L;
}

/** \brief Sweep time at which the position chirp ends: the first zero crossing of the sine at or after
 *  POS_REF_CHIRP_DURATION_S (CHIRP_HOLD: after T + POS_REF_CHIRP_F1_HOLD_S + POS_REF_CHIRP_TAPER_S), so the
 *  reference returns to the offset without a step. Computed by inverting the phase at
 *  phase = pi * ceil(phase(end) / pi): tau = L ln(1 + phase / (2 pi f0 L)) on the sweep, linear at f1 after it.
 *  \return Sweep time in seconds since POS_REF_CHIRP_START_S (at most half a period of f1 after the nominal end)
 */
static double
pos_ref_chirp_end_s(void)
{
   static double end_s = -1.0;   /* computed once; pos_ref() calls this every cycle */
   if (end_s < 0.0)
   {
      const double T = POS_REF_CHIRP_DURATION_S;
      double ph, dph, ddph;
      pos_ref_chirp_phase(T + POS_REF_CHIRP_AFTER_S, &ph, &dph, &ddph);
      double phase_end = M_PI * ceil(ph / M_PI);
#if POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD
      double phase_T = exp_chirp_phase(T, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ, T);
      end_s = T + (phase_end - phase_T) / (2.0 * M_PI * POS_REF_CHIRP_F1_HZ);
#else
      const double L = T / log(POS_REF_CHIRP_F1_HZ / POS_REF_CHIRP_F0_HZ);
      end_s = L * log(1.0 + phase_end / (2.0 * M_PI * POS_REF_CHIRP_F0_HZ * L));
#endif
   }
   return end_s;
}

/** \brief Raised-cosine amplitude envelope of the chirp: rises from 0 to 1 over the first POS_REF_CHIRP_TAPER_S
 *  of the sweep and falls back to 0 over the last, so the reference velocity and acceleration start and end at 0
 *  \param tau Sweep time in seconds (0 < tau < end_s)
 *  \param end_s Sweep end from pos_ref_chirp_end_s()
 *  \param w Envelope (0 .. 1); \param wd Its derivative in 1/s; \param wdd Its second derivative in 1/s^2
 */
static void
pos_ref_chirp_taper(double tau, double end_s, double *w, double *wd, double *wdd)
{
   const double F = POS_REF_CHIRP_TAPER_S;
   *w = 1.0;
   *wd = 0.0;
   *wdd = 0.0;
   if (F <= 0.0)
   {
      return;
   }
   double u = tau < F ? tau : end_s - tau;                   /* time from the nearer end */
   if (u < F)
   {
      double sign = tau < F ? 1.0 : -1.0;                    /* rising at the start, falling at the end */
      *w = 0.5 * (1.0 - cos(M_PI * u / F));
      *wd = sign * M_PI / (2.0 * F) * sin(M_PI * u / F);
      *wdd = M_PI * M_PI / (2.0 * F * F) * cos(M_PI * u / F);
   }
}
#endif

/** \brief Position reference and its first two derivatives at time t (analytic, no differencing)
 *  \param t Time since the controller started in seconds (t >= 0)
 *  \param r_mm Reference position in mm (absolute position_mm frame)
 *  \param rd_mm_s First derivative in mm/s
 *  \param rdd_mm_s2 Second derivative in mm/s^2 (0 at ramp corners and steps, which are not differentiable)
 */
static void
pos_ref(double t, double *r_mm, double *rd_mm_s, double *rdd_mm_s2)
{
   *rd_mm_s = 0.0;
   *rdd_mm_s2 = 0.0;
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
   *r_mm = breakpoint_value(pos_ref_steps, POS_REF_STEPS_N, POS_REF_RAMP_S, t);
   int i = 0;
   while (i + 1 < POS_REF_STEPS_N && pos_ref_steps[i + 1].t_s <= t)
   {
      i++;
   }
   if (i > 0 && POS_REF_RAMP_S > 0.0 && t - pos_ref_steps[i].t_s < POS_REF_RAMP_S)
   {
      *rd_mm_s = (pos_ref_steps[i].value - pos_ref_steps[i - 1].value) / POS_REF_RAMP_S;   /* inside a ramp */
   }
#elif POS_REF_IS_CHIRP
   double tau = t - POS_REF_CHIRP_START_S;
   *r_mm = POS_REF_CHIRP_OFFSET_MM;
   if (tau > 0.0 && tau < pos_ref_chirp_end_s())             /* hold before and after the sweep */
   {
      /* r = A w sin(phase) */
      double ph, dph, ddph;
      pos_ref_chirp_phase(tau, &ph, &dph, &ddph);
      double w, wd, wdd;
      pos_ref_chirp_taper(tau, pos_ref_chirp_end_s(), &w, &wd, &wdd);
      double s = sin(ph), c = cos(ph);
      *r_mm += POS_REF_CHIRP_AMPLITUDE_MM * w * s;
      *rd_mm_s = POS_REF_CHIRP_AMPLITUDE_MM * (wd * s + w * dph * c);
      *rdd_mm_s2 = POS_REF_CHIRP_AMPLITUDE_MM * (wdd * s + 2.0 * wd * dph * c + w * (ddph * c - dph * dph * s));
   }
#else
   const double w = 2.0 * M_PI * POS_REF_SINE_FREQ_HZ;
   *r_mm = POS_REF_SINE_OFFSET_MM + POS_REF_SINE_AMPLITUDE_MM * sin(w * t);
   *rd_mm_s = POS_REF_SINE_AMPLITUDE_MM * w * cos(w * t);
   *rdd_mm_s2 = -POS_REF_SINE_AMPLITUDE_MM * w * w * sin(w * t);
#endif
}

/** \brief Check the shared settings before the run (doubles rule out some _Static_asserts)
 *  \param kp_amps Drive peak current; the output limit must stay below it
 *  \return TRUE if usable, FALSE (with a message printed) otherwise
 */
static boolean
pos_ref_check(double kp_amps)
{
   if (!(kp_amps > 0.0))
   {
      printf("POSITION PID: drive peak current KP not read (%.1f A); refusing to run\n", kp_amps);
      return FALSE;
   }
   if (OUTPUT_LIMIT_A >= kp_amps)
   {
      printf("POSITION PID: OUTPUT_LIMIT_A %.2f A is not below the drive peak current %.1f A\n",
             OUTPUT_LIMIT_A, kp_amps);
      return FALSE;
   }
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
   if (pos_ref_steps[0].t_s != 0.0)
   {
      printf("POS_REF_STEPS: first breakpoint must be at t = 0 s (is %.3f s)\n", pos_ref_steps[0].t_s);
      return FALSE;
   }
   for (int i = 0; i < POS_REF_STEPS_N; i++)
   {
      if (pos_ref_steps[i].value < POS_REF_MIN_MM || pos_ref_steps[i].value > POS_REF_MAX_MM)
      {
         printf("POS_REF_STEPS: %.3f mm at t = %.3f s is outside %.1f .. %.1f mm\n",
                pos_ref_steps[i].value, pos_ref_steps[i].t_s, POS_REF_MIN_MM, POS_REF_MAX_MM);
         return FALSE;
      }
      if (pos_ref_steps[i].t_s >= RUN_DURATION_S)
      {
         printf("POS_REF_STEPS: breakpoint at t = %.3f s is not before RUN_DURATION_S (%.1f s)\n",
                pos_ref_steps[i].t_s, RUN_DURATION_S);
         return FALSE;
      }
      if (i > 0 && pos_ref_steps[i].t_s < pos_ref_steps[i - 1].t_s + POS_REF_RAMP_S)
      {
         printf("POS_REF_STEPS: breakpoint at t = %.3f s starts before the ramp at t = %.3f s ends "
                "(times must increase by at least POS_REF_RAMP_S = %.3f s)\n",
                pos_ref_steps[i].t_s, pos_ref_steps[i - 1].t_s, POS_REF_RAMP_S);
         return FALSE;
      }
   }
#endif
   return TRUE;
}

/* ---- Position filters ------------------------------------------------------------------------ */

#if POS_NOTCH_ACTIVE
#define NOTCH_ENTRY_(f_, q_) { .freq_Hz = f_, .q = q_ },
static notch_t notches[] = { POS_NOTCHES(NOTCH_ENTRY_) };
#define NOTCH_COUNT ((int)(sizeof(notches) / sizeof(notches[0])))
#endif

#if POS_KF_ENABLE
/* Currents sent in the last POS_KF_DELAY_CYCLES + 1 cycles, oldest at kf_u_oldest. At the start of cycle k
 * they are those of cycles k-1-D .. k-1, so the oldest is the EKF input for the step k-1 -> k. */
#define KF_U_N  (POS_KF_DELAY_CYCLES + 1)
static double kf_u_sent_A[KF_U_N];
static int kf_u_oldest = 0;
#endif

/* ---- Entry points ---------------------------------------------------------------------------- */

/** \brief Print the controller, filters and reference, check the settings and initialise the filters;
 *  call once before the cyclic loop
 *  \param kp_amps Drive peak current
 *  \return TRUE if usable, FALSE (with a message printed) otherwise
 */
boolean
closed_loop_setup(double kp_amps)
{
   if (!controller_setup(kp_amps))
   {
      return FALSE;
   }
   printf("  reference window %.1f .. %.1f mm, trip outside %.1f .. %.1f mm (raw position) for %d cycles\n",
          POS_REF_MIN_MM, POS_REF_MAX_MM, POS_TRIP_MIN_MM, POS_TRIP_MAX_MM, POS_TRIP_CYCLES);
#if POS_KF_ENABLE
   double sig_y_used_mm = vca_ekf_init(POS_KF_SIG_A_M_S2, POS_KF_SIG_Y_MM, POS_KF_MAINS_HZ, POS_KF_MAINS_BW_HZ,
                                       POS_KF_MAINS_PICKUP_MM);
   printf("  position filter: EKF on the Method C model, SIG_A = %.3g m/s^2, SIG_Y = %.4f mm%s\n",
          POS_KF_SIG_A_M_S2, sig_y_used_mm,
          POS_KF_MAINS_ENABLE ? "" : " (POS_KF_SIG_Y_MM with the mains pickup counted as noise)");
#if POS_KF_MAINS_ENABLE
   printf("    mains pickup on the laser modelled as a %.2f Hz state, tracking bandwidth %.2f Hz, typical %.3f mm\n",
          POS_KF_MAINS_HZ, POS_KF_MAINS_BW_HZ, POS_KF_MAINS_PICKUP_MM);
#if POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD || POS_REF_SHAPE == POS_REF_SHAPE_SINE
   const double f_hold_hz = POS_REF_SHAPE == POS_REF_SHAPE_SINE ? POS_REF_SINE_FREQ_HZ : POS_REF_CHIRP_F1_HZ;
   if (fabs(f_hold_hz - POS_KF_MAINS_HZ) < 2.0 * POS_KF_MAINS_BW_HZ)
   {
      printf("    WARNING: the reference frequency %.2f Hz is within 2 x POS_KF_MAINS_BW_HZ of %.2f Hz; the filter "
             "will take part of the real motion for pickup. Lower POS_KF_MAINS_BW_HZ or move the frequency\n",
             f_hold_hz, POS_KF_MAINS_HZ);
   }
#endif
#endif
   printf("    m = %.2f kg, Gamma = %.1f N/A, Gamma1 = %.4g N/(A m), c = %.1f N s/m, C_R = %.4g N m^3, "
          "C_L = %.4g N m^3, g0 = %.1f mm\n", VCA_MASS_KG, VCA_GAMMA_N_PER_A, VCA_GAMMA1_N_PER_AM,
          VCA_DAMPING_NS_PER_M, VCA_C_R_NM3, VCA_C_L_NM3, VCA_GAP_M * 1e3);
   printf("    input delay: the current sent in cycle k acts over the step k + %d -> k + %d (%.1f ms after sending)\n",
          POS_KF_DELAY_CYCLES, POS_KF_DELAY_CYCLES + 1, POS_KF_DELAY_CYCLES * CYCLE_TIME_MS);
   for (int i = 0; i < KF_U_N; i++)
   {
      kf_u_sent_A[i] = 0.0;     /* the run starts with the 0 A idle window */
   }
   kf_u_oldest = 0;
#endif
#if POS_CONTROLLER == POS_CONTROLLER_SMC
   printf("  SMC uses the EKF %d-step prediction x(k+%d|k), v(k+%d|k) and the reference at t + %.1f ms%s\n",
          POS_KF_DELAY_CYCLES + 1, POS_KF_DELAY_CYCLES + 1, POS_KF_DELAY_CYCLES + 1,
          (POS_KF_DELAY_CYCLES + 1) * CYCLE_TIME_MS, POS_NOTCH_ENABLE ? "; POS_NOTCHES bypassed" : "");
#endif
#if POS_NOTCH_ACTIVE
   printf("  position filter: %d notch(es) in cascade (PI uses the notched %s)\n",
          NOTCH_COUNT, POS_KF_ENABLE ? "EKF x(k|k)" : "raw position");
   for (int i = 0; i < NOTCH_COUNT; i++)
   {
      printf("    notch at %.2f Hz, Q = %.2f\n", notches[i].freq_Hz, notches[i].q);
   }
   notch_init(notches, NOTCH_COUNT, 1000.0 / CYCLE_TIME_MS);
#endif
#if !POS_KF_ENABLE && !POS_NOTCH_ACTIVE
   printf("  position filter: off (PI uses the raw position)\n");
#endif
#if POS_REF_SHAPE == POS_REF_SHAPE_STEPS
   printf("  reference: absolute steps, last value held until t = %.2f s\n", RUN_DURATION_S);
   for (int i = 0; i < POS_REF_STEPS_N; i++)
   {
      printf("  t = %7.2f s -> %+.3f mm%s\n", pos_ref_steps[i].t_s, pos_ref_steps[i].value,
             i > 0 && POS_REF_RAMP_S > 0.0 ? " (linear ramp)" : "");
   }
   if (POS_REF_RAMP_S > 0.0)
   {
      printf("  ramp time %.2f s\n", POS_REF_RAMP_S);
   }
#elif POS_REF_SHAPE == POS_REF_SHAPE_CHIRP_HOLD
   const double sweep_end_s = POS_REF_CHIRP_START_S + POS_REF_CHIRP_DURATION_S;
   printf("  reference: absolute chirp and hold, %+.3f mm + %.3f mm * sin(phase), exponential sweep %.2f -> %.2f Hz, "
          "then %.2f Hz\n", POS_REF_CHIRP_OFFSET_MM, POS_REF_CHIRP_AMPLITUDE_MM, POS_REF_CHIRP_F0_HZ,
          POS_REF_CHIRP_F1_HZ, POS_REF_CHIRP_F1_HZ);
   printf("  hold %+.3f mm until t = %.2f s, sweep until t = %.2f s, %.2f Hz at full amplitude until t = %.2f s, "
          "fade-out until t = %.3f s (zero crossing), then hold until t = %.2f s\n", POS_REF_CHIRP_OFFSET_MM,
          POS_REF_CHIRP_START_S, sweep_end_s, POS_REF_CHIRP_F1_HZ, sweep_end_s + POS_REF_CHIRP_F1_HOLD_S,
          POS_REF_CHIRP_START_S + pos_ref_chirp_end_s(), RUN_DURATION_S);
   if (POS_REF_CHIRP_TAPER_S > 0.0)
   {
      printf("  amplitude fades in over the first %.2f s of the sweep and out over the last %.2f s (raised cosine)\n",
             POS_REF_CHIRP_TAPER_S, POS_REF_CHIRP_TAPER_S);
   }
#elif POS_REF_SHAPE == POS_REF_SHAPE_CHIRP
   printf("  reference: absolute chirp, %+.3f mm + %.3f mm * sin(phase), exponential sweep %.2f -> %.2f Hz\n",
          POS_REF_CHIRP_OFFSET_MM, POS_REF_CHIRP_AMPLITUDE_MM, POS_REF_CHIRP_F0_HZ, POS_REF_CHIRP_F1_HZ);
   printf("  hold %+.3f mm until t = %.2f s, sweep until t = %.3f s (next zero crossing after %.2f s), "
          "then hold until t = %.2f s\n", POS_REF_CHIRP_OFFSET_MM, POS_REF_CHIRP_START_S,
          POS_REF_CHIRP_START_S + pos_ref_chirp_end_s(), POS_REF_CHIRP_START_S + POS_REF_CHIRP_DURATION_S,
          RUN_DURATION_S);
   if (POS_REF_CHIRP_TAPER_S > 0.0)
   {
      printf("  amplitude fades in over the first %.2f s and out over the last %.2f s of the sweep (raised cosine)\n",
             POS_REF_CHIRP_TAPER_S, POS_REF_CHIRP_TAPER_S);
   }
#else
   printf("  reference: absolute sine, %+.3f mm + %.3f mm * sin(2 pi %.3f Hz t)\n",
          POS_REF_SINE_OFFSET_MM, POS_REF_SINE_AMPLITUDE_MM, POS_REF_SINE_FREQ_HZ);
#endif
   return pos_ref_check(kp_amps);
}

/** \brief Position trip on the raw position, over the whole run (idle window included): a laser fault
 *  (signal lost reads about +29 mm) or a shaft outside the window stops the run
 *  \param position_mm Raw position from this cycle's exchange
 *  \return TRUE once the position has been outside POS_TRIP_MIN/MAX_MM for POS_TRIP_CYCLES consecutive cycles
 */
boolean
closed_loop_trip_update(double position_mm)
{
   static int pos_trip_count = 0;   /* consecutive cycles outside the trip window */
   if (position_mm < POS_TRIP_MIN_MM || position_mm > POS_TRIP_MAX_MM)
   {
      pos_trip_count++;
      return pos_trip_count >= POS_TRIP_CYCLES;
   }
   pos_trip_count = 0;
   return FALSE;
}

/** \brief One cycle of the position loop: EKF -> notches -> controller -> output limit
 *  The EKF and notch run from the first cycle, so they have settled long before the controller starts at
 *  t = 0. The EKF gets the raw position (its model has no notch in it); the notch then acts on x(k|k).
 *  The SMC skips the notches and uses the EKF prediction at the cycle its output acts, 1 + POS_KF_DELAY_CYCLES
 *  steps ahead (see closed_loop_settings.h).
 *  \param t_s Elapsed time in seconds; negative during the bias idle window (controller off, 0 A)
 *  \param position_mm Raw position from this cycle's exchange
 *  \param sent_current_A Current sent this cycle: acts over the step to the next measurement
 *  \return This cycle's values for the log; output_A is the current to send in the NEXT cycle
 */
pid_log_t
closed_loop_step(double t_s, double position_mm, double sent_current_A)
{
   double position_filt_mm;
   pid_log_t pid;
   pos_ctrl_in_t in = { .x_pred_mm = NAN, .v_pred_mm_s = NAN };
#if POS_KF_ENABLE
   /* Step k-1 -> k with the current sent in cycle k-1-D, then replace it by this cycle's */
   vca_ekf_out_t kf_out = vca_ekf_step(position_mm, kf_u_sent_A[kf_u_oldest]);
   position_filt_mm = kf_out.x_mm;
   kf_u_sent_A[kf_u_oldest] = sent_current_A;
   kf_u_oldest = (kf_u_oldest + 1) % KF_U_N;
   /* State when this cycle's output reaches the plant: it is sent in cycle k+1 and acts from step k+1+D, so
    * D+1 model steps on with the currents still on their way, those sent in cycles k-D .. k */
   double u_pending_A[KF_U_N];
   for (int i = 0; i < KF_U_N; i++)
   {
      u_pending_A[i] = kf_u_sent_A[(kf_u_oldest + i) % KF_U_N];
   }
   vca_ekf_out_t kf_pred = vca_ekf_predict_n(u_pending_A, KF_U_N);
   in.x_pred_mm = kf_pred.x_mm;
   in.v_pred_mm_s = kf_pred.v_mm_s;
#else
   (void)sent_current_A;
   position_filt_mm = position_mm;
#endif
#if POS_NOTCH_ACTIVE
   position_filt_mm = notch_cascade_step(notches, NOTCH_COUNT, position_filt_mm);
#endif

   if (t_s < 0.0)
   {
      /* Bias idle window: controller off, 0 A, controller state untouched. */
      pid = (pid_log_t){ .position_ref_mm = NAN, .p_A = 0.0, .i_A = 0.0, .output_A = 0.0 };
   }
   else
   {
      double rd_mm_s, rdd_mm_s2;
      pos_ref(t_s, &in.r_mm, &rd_mm_s, &rdd_mm_s2);
      /* Reference at the time the output acts, matching x_pred (one cycle without the EKF) */
      pos_ref(t_s + (1 + POS_KF_ENABLE * POS_KF_DELAY_CYCLES) * CYCLE_TIME_MS / 1000.0,
              &in.r_next_mm, &in.rd_next_mm_s, &in.rdd_next_mm_s2);
      in.y_mm = position_filt_mm;
      pid = controller_update(&in);
      /* Shared safety for every controller: limit the output, never send a non-finite value. */
      pid.output_A = clamp_abs(pid.output_A, OUTPUT_LIMIT_A);
      if (!isfinite(pid.output_A))
      {
         pid.output_A = 0.0;
      }
   }
   pid.position_filt_mm = position_filt_mm;
#if POS_KF_ENABLE
   pid.kf_velocity_mm_s = kf_out.v_mm_s;
   pid.kf_innovation_mm = kf_out.innov_mm;
   pid.kf_innov_std_mm = kf_out.innov_std_mm;
   pid.kf_pickup_mm = kf_out.pickup_mm;
#else
   pid.kf_velocity_mm_s = NAN;
   pid.kf_innovation_mm = NAN;
   pid.kf_innov_std_mm = NAN;
   pid.kf_pickup_mm = NAN;
#endif
   return pid;
}

#endif /* EXPERIMENT_MODE == EXPERIMENT_POSITION_PID */
