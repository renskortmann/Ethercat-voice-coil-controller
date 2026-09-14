/** \file cia402.c
 * \brief CiA402 state machine bring-up: progress drive through enable sequence
 */

#include "main.h"

#define BRING_UP_TIMEOUT_CYCLES 3000
#define BRING_UP_ATTEMPTS       2

/** \brief Clear a latched fault with a rising edge on ControlWord bit 7 (transition 15)
 *  Shutdown cannot clear a fault; only the bit 7 edge does. Bit 7 is dropped again afterwards so
 *  a later fault can still produce a fresh edge.
 *  \param fieldbus Fieldbus context
 *  \return TRUE when the Fault bit clears, FALSE on timeout
 */
static boolean
cia402_fault_reset(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   rx_pdo_t *rx = (rx_pdo_t *)grp->outputs;
   tx_pdo_t *tx = (tx_pdo_t *)grp->inputs;
   int cycles;
   boolean cleared = FALSE;

   printf("  [0/3] Fault latched (StatusWord=0x%04X), resetting...", tx->statusword);
   rx->controlword = CTRL_FAULT_RESET;
   for (cycles = 0; cycles < BRING_UP_TIMEOUT_CYCLES; cycles++)
   {
      ecx_send_processdata(context);
      ecx_receive_processdata(context, EC_TIMEOUTRET);
      if (!(tx->statusword & STATUS_FAULT_BIT))
      {
         printf(" OK (StatusWord=0x%04X)\n", tx->statusword);
         cleared = TRUE;
         break;
      }
      osal_usleep(1000);
   }
   if (!cleared)
   {
      printf(" TIMEOUT (StatusWord=0x%04X)\n", tx->statusword);
   }
   rx->controlword = CTRL_DISABLE_VOLT;
   return cleared;
}

/** \brief Command one CiA402 transition and wait for the drive to reach the target state
 *  Reports every StatusWord change: when the drive trips during bring-up, the cycle number pins
 *  the trip to the command that provoked it.
 *  \param fieldbus Fieldbus context
 *  \param controlword ControlWord command to issue
 *  \param target_state Masked StatusWord value that marks the transition complete
 *  \param label Progress label printed for this step
 *  \return TRUE on reaching target_state, FALSE on timeout or if the drive trips to Fault
 */
static boolean
cia402_transition(Fieldbus *fieldbus, uint16_t controlword, uint16_t target_state, const char *label)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   rx_pdo_t *rx = (rx_pdo_t *)grp->outputs;
   tx_pdo_t *tx = (tx_pdo_t *)grp->inputs;
   uint16_t last_statusword = tx->statusword;
   int cycles;
   int wkc;

   printf("  %s...", label);
   rx->controlword = controlword;
   for (cycles = 0; cycles < BRING_UP_TIMEOUT_CYCLES; cycles++)
   {
      ecx_send_processdata(context);
      wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
      if (tx->statusword != last_statusword)
      {
         printf("\n    cycle %d: wkc=%d sw=0x%04X", cycles, wkc, tx->statusword);
         last_statusword = tx->statusword;
      }
      if ((tx->statusword & STATUS_WORD_MASK) == target_state)
      {
         printf(" OK (StatusWord=0x%04X)\n", tx->statusword);
         return TRUE;
      }
      if (tx->statusword & STATUS_FAULT_BIT)
      {
         printf(" FAULT (StatusWord=0x%04X)\n", tx->statusword);
         return FALSE;
      }
      osal_usleep(1000);
   }
   printf(" TIMEOUT (StatusWord=0x%04X)\n", tx->statusword);
   return FALSE;
}

/** \brief Progress drive through CiA402 state machine to Operation Enabled
 *  Executes state transitions: Shutdown→Ready to Switch On→Switched On→Operation Enabled.
 *  A fault latched before or during bring-up is reset and the sequence retried, bounded by
 *  BRING_UP_ATTEMPTS so a drive that keeps tripping reports rather than loops.
 *  \param fieldbus Fieldbus context
 *  \return TRUE when Operation Enabled reached, FALSE on timeout or persistent fault
 */
boolean
cia402_bring_up(Fieldbus *fieldbus)
{
   ecx_contextt *context = &fieldbus->context;
   ec_groupt *grp = context->grouplist + fieldbus->group;
   tx_pdo_t *tx = (tx_pdo_t *)grp->inputs;
   int attempt;

   printf("\nBringing drive through CiA402 state machine:\n");

   for (attempt = 1; attempt <= BRING_UP_ATTEMPTS; attempt++)
   {
      ecx_send_processdata(context);
      ecx_receive_processdata(context, EC_TIMEOUTRET);

      if ((tx->statusword & STATUS_FAULT_BIT) && !cia402_fault_reset(fieldbus))
      {
         return FALSE;
      }

      if (cia402_transition(fieldbus, CTRL_SHUTDOWN, STATE_READY_TO_SWITCH_ON,
                            "[1/3] Shutdown → Ready to Switch On") &&
          cia402_transition(fieldbus, CTRL_SWITCH_ON, STATE_SWITCHED_ON,
                            "[2/3] Switch On → Switched On") &&
          cia402_transition(fieldbus, CTRL_ENABLE_OP, STATE_OPERATION_ENABLED,
                            "[3/3] Enable Operation"))
      {
         return TRUE;
      }

      /* A timeout without a fault means the drive is ignoring the command rather than
       * rejecting it, and a reset would change nothing. */
      if (!(tx->statusword & STATUS_FAULT_BIT))
      {
         return FALSE;
      }
      printf("  Attempt %d tripped the drive to Fault.\n", attempt);
   }

   return FALSE;
}
