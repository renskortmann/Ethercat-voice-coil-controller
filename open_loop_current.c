/** \file open_loop_current.c
 * \brief Open-loop current experiments: target current per cycle for the selected EXPERIMENT_MODE,
 *  plus the startup banner and table checks
 */

#include "open_loop_current.h"
#include "waveforms.h"

#if EXPERIMENT_MODE != EXPERIMENT_POSITION_PID

#define MS_(x)  ((int)((x) * 1000))   /* seconds/Hz/A -> integer thousandths, for _Static_assert */

#if EXPERIMENT_MODE == EXPERIMENT_STEP_RELEASE
_Static_assert(MS_(HOLD_DURATION_S) < MS_(RUN_DURATION_S),
               "HOLD_DURATION_S must be shorter than RUN_DURATION_S, otherwise the release never happens");
#endif

#if EXPERIMENT_MODE == EXPERIMENT_CHIRP || EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
_Static_assert(MS_(CHIRP_DURATION_S) <= MS_(RUN_DURATION_S),
               "CHIRP_DURATION_S must not exceed RUN_DURATION_S");
_Static_assert(MS_(CHIRP_F0_HZ) > 0 && MS_(CHIRP_F0_HZ) != MS_(CHIRP_F1_HZ),
               "CHIRP_F0_HZ must be > 0 and differ from CHIRP_F1_HZ (the sweep rate divides by ln(f1/f0))");
/* 10 samples per period at the highest frequency: CHIRP_F1_HZ <= 200 Hz at a 0.5 ms cycle. */
_Static_assert((int)(CHIRP_F1_HZ * CYCLE_TIME_MS) <= 100,
               "CHIRP_F1_HZ too high for CYCLE_TIME_MS: fewer than 10 samples per period");
#endif

#if EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
#define SINE_BLOCKS_ONE_(f, a)    + 1
#define SINE_BLOCKS_COUNT         (0 SINE_BLOCKS(SINE_BLOCKS_ONE_))
#define SINE_BLOCKS_FREQ_OK_(f, a) && MS_(f) > 0 && (int)((f) * CYCLE_TIME_MS) <= 100
_Static_assert(MS_(SINE_BLOCKS_COUNT * SINE_BLOCK_PERIOD_S) <= MS_(RUN_DURATION_S),
               "SINE_BLOCKS do not fit in RUN_DURATION_S (count * SINE_BLOCK_PERIOD_S)");
_Static_assert(MS_(SINE_BLOCK_RAMP_S) > 0, "SINE_BLOCK_RAMP_S must be > 0");
/* 10 samples per period at each block frequency: <= 200 Hz at a 0.5 ms cycle. */
_Static_assert(1 SINE_BLOCKS(SINE_BLOCKS_FREQ_OK_),
               "SINE_BLOCKS frequency must be > 0 and give at least 10 samples per period at CYCLE_TIME_MS");
#endif

#if EXPERIMENT_MODE == EXPERIMENT_PRBS
_Static_assert(MS_(PRBS_DURATION_S) <= MS_(RUN_DURATION_S),
               "PRBS_DURATION_S must not exceed RUN_DURATION_S");
_Static_assert(MS_(PRBS_BANDWIDTH_HZ) <= 60000,
               "PRBS_BANDWIDTH_HZ must not exceed 60 Hz");
#endif

/* ---- Chirps ---------------------------------------------------------------------------------- */

#if EXPERIMENT_MODE == EXPERIMENT_CHIRP || EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
/** \brief Unit-amplitude exponential chirp CHIRP_F0_HZ -> CHIRP_F1_HZ over CHIRP_DURATION_S
 *  \param t Time since the start of the sweep in seconds (0 <= t < CHIRP_DURATION_S)
 *  \return sin(phase(t)), in [-1, 1]
 */
static double
chirp_unit(double t)
{
   return sin(exp_chirp_phase(t, CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S));
}
#endif

#if EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
/** \brief Amplitude breakpoints (value in Amps) from CHIRP_SCHED, validated by chirp_sched_check() */
#define CHIRP_SCHED_ENTRY_(t, a)  {t, a},
static const breakpoint_t chirp_sched[] = { CHIRP_SCHED(CHIRP_SCHED_ENTRY_) };
#define CHIRP_SCHED_N ((int)(sizeof(chirp_sched) / sizeof(chirp_sched[0])))

/** \brief Check the CHIRP_SCHED table before the run (doubles rule out a _Static_assert)
 *  \param kp_amps Drive peak current; larger amplitudes would be clipped by the saturation
 *  \return TRUE if the table is usable, FALSE (with a message printed) otherwise
 */
static boolean
chirp_sched_check(double kp_amps)
{
   if (chirp_sched[0].t_s != 0.0)
   {
      printf("CHIRP_SCHED: first breakpoint must be at t = 0 s (is %.3f s)\n", chirp_sched[0].t_s);
      return FALSE;
   }
   for (int i = 0; i < CHIRP_SCHED_N; i++)
   {
      if (fabs(chirp_sched[i].value) > kp_amps)
      {
         printf("CHIRP_SCHED: %.2f A at t = %.3f s exceeds the drive peak current %.1f A\n",
                chirp_sched[i].value, chirp_sched[i].t_s, kp_amps);
         return FALSE;
      }
      if (chirp_sched[i].t_s >= CHIRP_DURATION_S)
      {
         printf("CHIRP_SCHED: breakpoint at t = %.3f s is not before CHIRP_DURATION_S (%.1f s)\n",
                chirp_sched[i].t_s, CHIRP_DURATION_S);
         return FALSE;
      }
      if (i > 0 && chirp_sched[i].t_s < chirp_sched[i - 1].t_s + CHIRP_SCHED_RAMP_S)
      {
         printf("CHIRP_SCHED: breakpoint at t = %.3f s starts before the ramp at t = %.3f s ends "
                "(times must increase by at least CHIRP_SCHED_RAMP_S = %.3f s)\n",
                chirp_sched[i].t_s, chirp_sched[i - 1].t_s, CHIRP_SCHED_RAMP_S);
         return FALSE;
      }
   }
   return TRUE;
}
#endif

/* ---- Sine blocks ----------------------------------------------------------------------------- */

#if EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
/** \brief Sine blocks from SINE_BLOCKS, validated by sine_blocks_check() */
#define SINE_BLOCKS_ENTRY_(f, a)  {f, a},
static const struct
{
   double f_Hz;     /**< Sine frequency in Hz */
   double amp_A;    /**< Amplitude during the hold, in Amps */
} sine_blocks[] = { SINE_BLOCKS(SINE_BLOCKS_ENTRY_) };
#define SINE_BLOCKS_N ((int)(sizeof(sine_blocks) / sizeof(sine_blocks[0])))

/** \brief Sine-block current at time t: ramp-in, hold, ramp-out, pause per block, then 0 A
 *  \param t Time since the start of the first block in seconds (t >= 0)
 *  \return Target current in Amps
 */
static double
sine_blocks_current_A(double t)
{
   int k = (int)(t / SINE_BLOCK_PERIOD_S);
   if (k >= SINE_BLOCKS_N)
   {
      return 0.0;                                            /* all blocks done: ring-down */
   }
   double tb = t - k * SINE_BLOCK_PERIOD_S;                  /* time into block k */
   double envelope;
   if (tb < SINE_BLOCK_RAMP_S)
   {
      envelope = tb / SINE_BLOCK_RAMP_S;                     /* ramp in */
   }
   else if (tb < SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S)
   {
      envelope = 1.0;                                        /* hold */
   }
   else if (tb < 2.0 * SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S)
   {
      envelope = (2.0 * SINE_BLOCK_RAMP_S + SINE_BLOCK_HOLD_S - tb) / SINE_BLOCK_RAMP_S;  /* ramp out */
   }
   else
   {
      return 0.0;                                            /* pause */
   }
   return sine_blocks[k].amp_A * envelope * sin(2.0 * M_PI * sine_blocks[k].f_Hz * tb);
}

/** \brief Check the SINE_BLOCKS table before the run (doubles rule out a _Static_assert)
 *  \param kp_amps Drive peak current; larger amplitudes would be clipped by the saturation
 *  \return TRUE if the table is usable, FALSE (with a message printed) otherwise
 */
static boolean
sine_blocks_check(double kp_amps)
{
   for (int i = 0; i < SINE_BLOCKS_N; i++)
   {
      if (fabs(sine_blocks[i].amp_A) > kp_amps)
      {
         printf("SINE_BLOCKS: %.2f A in block %d (%.2f Hz) exceeds the drive peak current %.1f A\n",
                sine_blocks[i].amp_A, i, sine_blocks[i].f_Hz, kp_amps);
         return FALSE;
      }
   }
   return TRUE;
}
#endif

/* ---- Entry points ---------------------------------------------------------------------------- */

/** \brief Print the experiment and check its tables; call once before the cyclic loop
 *  \param kp_amps Drive peak current
 *  \return TRUE if usable, FALSE (with a message printed) otherwise
 */
boolean
open_loop_setup(double kp_amps)
{
   (void)kp_amps;
#if EXPERIMENT_MODE == EXPERIMENT_SINE
   printf("\nExperiment: sine, %.1f Hz, %.2f A amplitude\n", SINE_FREQ_HZ, SINE_AMPLITUDE_A);
#elif EXPERIMENT_MODE == EXPERIMENT_STEP_RELEASE
   printf("\nExperiment: step-release, hold %.2f A (ramp %.2f s) until t = %.2f s, then release to 0 A\n",
          HOLD_CURRENT_A, HOLD_RAMP_S, HOLD_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP
   printf("\nExperiment: exponential chirp, %.2f A, %.2f -> %.2f Hz over %.2f s, then 0 A\n",
          CHIRP_AMPLITUDE_A, CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
   printf("\nExperiment: exponential chirp, scheduled amplitude, %.2f -> %.2f Hz over %.2f s, then 0 A\n",
          CHIRP_F0_HZ, CHIRP_F1_HZ, CHIRP_DURATION_S);
   for (int i = 0; i < CHIRP_SCHED_N; i++)
   {
      printf("  t = %7.2f s -> %.2f A%s\n", chirp_sched[i].t_s, chirp_sched[i].value,
             i > 0 && CHIRP_SCHED_RAMP_S > 0.0 ? " (linear ramp)" : "");
   }
   if (CHIRP_SCHED_RAMP_S > 0.0)
   {
      printf("  ramp time %.2f s\n", CHIRP_SCHED_RAMP_S);
   }
   if (!chirp_sched_check(kp_amps))
   {
      return FALSE;
   }
#elif EXPERIMENT_MODE == EXPERIMENT_PRBS
   printf("\nExperiment: PRBS, +/-%.2f A, bandwidth %.1f Hz (%d cycles per bit) for %.2f s, then 0 A\n",
          PRBS_AMPLITUDE_A, PRBS_BANDWIDTH_HZ, PRBS_HOLD_CYCLES, PRBS_DURATION_S);
#elif EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
   printf("\nExperiment: %d sine blocks, each %.1f s ramp-in, %.1f s hold, %.1f s ramp-out, %.1f s at 0 A\n",
          SINE_BLOCKS_N, SINE_BLOCK_RAMP_S, SINE_BLOCK_HOLD_S, SINE_BLOCK_RAMP_S, SINE_BLOCK_PAUSE_S);
   for (int i = 0; i < SINE_BLOCKS_N; i++)
   {
      double hold_start_s = i * SINE_BLOCK_PERIOD_S + SINE_BLOCK_RAMP_S;
      printf("  block %d: %6.2f Hz, %5.2f A, hold t = %7.2f .. %7.2f s\n", i, sine_blocks[i].f_Hz,
             sine_blocks[i].amp_A, hold_start_s, hold_start_s + SINE_BLOCK_HOLD_S);
   }
   if (!sine_blocks_check(kp_amps))
   {
      return FALSE;
   }
#elif EXPERIMENT_MODE == EXPERIMENT_NOISE
   printf("\nExperiment: noise, 0 A with the drive enabled for %.2f s\n", RUN_DURATION_S);
#endif
   return TRUE;
}

/** \brief Target current for this cycle, in Amps, for the experiment selected by EXPERIMENT_MODE
 *  This is the only experiment-specific code in the cyclic loop. It must stay cheap and
 *  allocation-free: it runs once per cycle inside the real-time loop.
 *  \param elapsed_s Elapsed time since the experiment start in seconds (>= 0; the loop sends 0 A
 *         during the bias idle window itself)
 *  \return Target current in Amps (converted to raw drive units by the caller)
 */
double
open_loop_current_A(double elapsed_s)
{
#if EXPERIMENT_MODE == EXPERIMENT_SINE
   return SINE_AMPLITUDE_A * sin(2.0 * M_PI * SINE_FREQ_HZ * elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_STEP_RELEASE
   if (elapsed_s >= HOLD_DURATION_S)
   {
      return 0.0;                                            /* released: free response */
   }
   if (HOLD_RAMP_S > 0.0 && elapsed_s < HOLD_RAMP_S)
   {
      return HOLD_CURRENT_A * (elapsed_s / HOLD_RAMP_S);     /* ramp in */
   }
   return HOLD_CURRENT_A;                                    /* hold */
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP
   if (elapsed_s >= CHIRP_DURATION_S)
   {
      return 0.0;                                            /* sweep done: ring-down */
   }
   return CHIRP_AMPLITUDE_A * chirp_unit(elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_CHIRP_SCHEDULED
   if (elapsed_s >= CHIRP_DURATION_S)
   {
      return 0.0;                                            /* sweep done: ring-down */
   }
   return breakpoint_value(chirp_sched, CHIRP_SCHED_N, CHIRP_SCHED_RAMP_S, elapsed_s) * chirp_unit(elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_PRBS
   static uint16_t lfsr = 1;      /* 15-bit LFSR state; nonzero seed, fixed so every run repeats */
   static long bit_index = 0;     /* index of the PRBS bit held in lfsr */
   if (elapsed_s >= PRBS_DURATION_S)
   {
      return 0.0;                                            /* sequence done: ring-down */
   }
   /* Cycle index from elapsed time; rounding absorbs floating-point error in elapsed_s. */
   long target_bit = lround(elapsed_s * 1000.0 / CYCLE_TIME_MS) / PRBS_HOLD_CYCLES;
   while (bit_index < target_bit)
   {
      /* Fibonacci LFSR, polynomial x^15 + x^14 + 1 (maximal length, period 32767) */
      uint16_t bit = ((lfsr >> 14) ^ (lfsr >> 13)) & 1u;
      lfsr = (uint16_t)(((lfsr << 1) | bit) & 0x7FFFu);
      bit_index++;
   }
   return (lfsr & 1u) ? PRBS_AMPLITUDE_A : -PRBS_AMPLITUDE_A;
#elif EXPERIMENT_MODE == EXPERIMENT_SINE_BLOCKS
   return sine_blocks_current_A(elapsed_s);
#elif EXPERIMENT_MODE == EXPERIMENT_NOISE
   (void)elapsed_s;
   return 0.0;                                               /* noise floor: drive enabled, no excitation */
#endif
}

#endif /* EXPERIMENT_MODE != EXPERIMENT_POSITION_PID */
