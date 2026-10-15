/** \file main.c
 * \brief CST current-mode cyclic control + logging for AMC servo drive
 *
 * Usage: ethercat-voice-coil-controller IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * Runs a RUN_DURATION_S-second hardware-synchronized control loop commanding
 * a sine-wave current to a voice-coil motor, logging samples and faults to CSV.
 */

#include "main.h"
#include "live_view.h"

/** \brief Main function for the voice-coil application
 * \param argc Number of command-line arguments
 * \param argv Array of command-line arguments
 * \return 0 on success, 1 on failure
 */
int main(int argc, char *argv[])
{
   Fieldbus fieldbus;
   ec_adaptert *adapter;
   struct sched_param param;

   if (argc != 2)
   {
      printf("Usage: ethercat-voice-coil-controller IFNAME\n");
      printf("IFNAME is the NIC interface name, e.g. 'eth0'\n\n");
      printf("Available adapters:\n");
      adapter = ec_find_adapters();
      while (adapter)
      {
         printf("  - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      return 1;
   }

   /* Lock memory to prevent page-fault latency. Kept early: locking pages doesn't affect
    * other processes, and surfacing the warning up front is useful either way. */
   if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
   {
      printf("WARNING: mlockall() failed (requires CAP_IPC_LOCK or root): %s\n", strerror(errno));
   }

   fieldbus_initialize(&fieldbus, argv[1]);

   /* Allocate buffers and prefault every page by touching them now, so the first write to a
    * given sample/fault slot during the real-time loop can't trigger a page fault. */
   fieldbus.samples = (sample_log_entry_t *)malloc(MAX_SAMPLES * sizeof(sample_log_entry_t));
   fieldbus.faults = (fault_log_entry_t *)malloc(MAX_FAULTS * sizeof(fault_log_entry_t));

   if (!fieldbus.samples || !fieldbus.faults)
   {
      printf("ERROR: Memory allocation failed\n");
      return 1;
   }
   memset(fieldbus.samples, 0, MAX_SAMPLES * sizeof(sample_log_entry_t));
   memset(fieldbus.faults, 0, MAX_FAULTS * sizeof(fault_log_entry_t));
   live_view_open();

   /* fieldbus_start() and cia402_bring_up() are the unbounded-duration, printf-heavy SDO
    * configuration and state-machine bring-up phases. They run at the default scheduling
    * class: they aren't the deadline-critical part, and running them at max real-time
    * priority would needlessly risk starving the rest of the system if anything stalls. */
   if (fieldbus_start(&fieldbus))
   {
      if (cia402_bring_up(&fieldbus))
      {
         /* Enter real-time mode: pin to the reserved core, raise to SCHED_FIFO at max
          * priority, and minimize timer slack, immediately before the deadline-critical
          * cyclic loop. Restored right after the loop returns. */
         long online_cpus = sysconf(_SC_NPROCESSORS_ONLN);
         cpu_set_t rt_cpuset, default_cpuset;
         CPU_ZERO(&rt_cpuset);
         CPU_ZERO(&default_cpuset);
         if (sched_getaffinity(0, sizeof(default_cpuset), &default_cpuset) != 0)
         {
            printf("WARNING: sched_getaffinity() failed: %s\n", strerror(errno));
         }
         if (online_cpus > RT_CPU_CORE)
         {
            CPU_SET(RT_CPU_CORE, &rt_cpuset);
            if (sched_setaffinity(0, sizeof(rt_cpuset), &rt_cpuset) != 0)
            {
               printf("WARNING: sched_setaffinity(core %d) failed: %s\n", RT_CPU_CORE, strerror(errno));
            }
         }
         else
         {
            printf("WARNING: RT_CPU_CORE %d not available (%ld online CPUs); skipping affinity\n",
                   RT_CPU_CORE, online_cpus);
         }

         param.sched_priority = sched_get_priority_max(SCHED_FIFO);
         if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
         {
            printf("WARNING: sched_setscheduler(SCHED_FIFO) failed (requires root): %s\n", strerror(errno));
         }

         if (prctl(PR_SET_TIMERSLACK, 1) != 0)
         {
            printf("WARNING: prctl(PR_SET_TIMERSLACK) failed: %s\n", strerror(errno));
         }

         fieldbus_run_cyclic(&fieldbus);

         /* Exit real-time mode before shutdown/CSV export (file I/O, printf). */
         param.sched_priority = 0;
         if (sched_setscheduler(0, SCHED_OTHER, &param) != 0)
         {
            printf("WARNING: sched_setscheduler(SCHED_OTHER) restore failed: %s\n", strerror(errno));
         }
         if (sched_setaffinity(0, sizeof(default_cpuset), &default_cpuset) != 0)
         {
            printf("WARNING: sched_setaffinity() restore failed: %s\n", strerror(errno));
         }
      }
      else
      {
         printf("ERROR: Failed to bring up CiA402 state machine\n");
         read_drive_status_sdo(&fieldbus, 0.0);
      }

      /* Disable voltage and shutdown */
      rx_pdo_t *rx = (rx_pdo_t *)fieldbus.context.grouplist[fieldbus.group].outputs;
      rx->controlword = CTRL_DISABLE_VOLT;
      fieldbus_roundtrip(&fieldbus);

      fieldbus_stop(&fieldbus);
   }

   live_view_close();
   export_csv(&fieldbus);

   free(fieldbus.samples);
   free(fieldbus.faults);

   return 0;
}
