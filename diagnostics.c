/** \file diagnostics.c
 * \brief Fault diagnosis: decode status flags and read post-fault SDO diagnostics
 */

#include "main.h"

static void
lookup_fault_name(uint16 drive_status_flag, uint8 subindex)
{
   /* This function can be expanded to decode specific drive status flags and subindices into human-readable fault names */
   printf("  Subindex: 0x%02X, Drive Status Flag: 0x%04X\n", subindex, drive_status_flag);

   // convert drive status to bitfield and print which bits are set
   for (int bit = 0; bit < 16; bit++)
   {
      if (drive_status_flag & (1 << bit))
      {
         switch (subindex)
         {
            case 0x01:  /* Drive Bridge Status */
               printf("    Drive Bridge Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Bridge Enabled\n"); break;
                  case 1: printf("Dynamix Brake Enabled\n"); break;
                  case 2: printf("Stop Enabled\n"); break;
                  case 3: printf("Positive Stop Enabled\n"); break;
                  case 4: printf("Negative Stop Enabled\n"); break;
                  case 5: printf("Positive Torque Inhibit Active\n"); break;
                  case 6: printf("Negative Torque Inhibit Active\n"); break;
                  case 7: printf("External Brake Active\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x02:  /* Drive Protection Status */
               printf("    Drive Protection Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Drive Reset\n"); break;
                  case 1: printf("Drive Internal Error\n"); break;
                  case 2: printf("Short Circuit\n"); break;
                  case 3: printf("Over Current\n"); break;
                  case 4: printf("Under Voltage\n"); break;
                  case 5: printf("Over Voltage\n"); break;
                  case 6: printf("Drive Over Temperature\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x03:  /* System Protection Status */
               printf("    System Protection Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Parameter Restore Error\n"); break;
                  case 1: printf("Parameter Store Error\n"); break;
                  case 2: printf("Invalid Hall State\n"); break;
                  case 3: printf("Phase sync Error\n"); break;
                  case 4: printf("Motor over temperature\n"); break;
                  case 5: printf("Phase Detection Fault\n"); break;
                  case 6: printf("Feedback Sensor Error\n"); break;
                  case 7: printf("Motor over Speed\n"); break;
                  case 8: printf("Max measured position\n"); break;
                  case 9: printf("Min measured position\n"); break;
                  case 10: printf("Comm error (node guarding)\n"); break;
                  case 11: printf("PWM input broken wire\n"); break;
                  case 12: printf("Motion Engine Error\n"); break;
                  case 13: printf("Motion Engine Abort\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x04:  /* Drive system status 1 */
               printf("   Drive system status 1: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Log Entry missed\n"); break;
                  case 1: printf("Software Disable\n"); break;
                  case 2: printf("User Disable\n"); break;
                  case 3: printf("User Positive Inhibit\n"); break;
                  case 4: printf("User Negative Inhibit\n"); break;
                  case 5: printf("Current Limiting\n"); break;
                  case 6: printf("Continuous Current Foldback\n"); break;
                  case 7: printf("Current loop saturated\n"); break;
                  case 8: printf("User under voltage\n"); break;
                  case 9: printf("User over voltage\n"); break;
                  case 10: printf("Non sinusoidal commutation\n"); break;
                  case 11: printf("Phase detect Active\n"); break;
                  case 12: printf("Motion Engine Active\n"); break;
                  case 13: printf("User Auxiliary Disabled\n"); break;
                  case 14: printf("Shunt Regulator\n"); break;
                  case 15: printf("Phase detect done\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x05:  /* Drive system status 2 */
               printf("   Drive system status 2: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Zero Velocity\n"); break;
                  case 1: printf("At Command\n"); break;
                  case 2: printf("Velocity Following Error\n"); break;
                  case 3: printf("Positive target velocity limit\n"); break;
                  case 4: printf("Negative target velocity limit\n"); break;
                  case 5: printf("Command Limiter Active\n"); break;
                  case 6: printf("In Home Position\n"); break;
                  case 7: printf("Position Following Error\n"); break;
                  case 8: printf("Max Target Position Limit\n"); break;
                  case 9: printf("Min target position limit\n"); break;
                  case 10: printf("Set Position\n"); break;
                  case 11: printf("Reserved\n"); break;
                  case 12: printf("Homing Active\n"); break;
                  case 13: printf("Safe Torque Off Status\n"); break;
                  case 14: printf("Homing Complete\n"); break;
                  case 15: printf("Zero Position Error\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x06:  /* Drive system status 3 */
               printf("   Drive system status 3: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Reserved\n"); break;
                  case 1: printf("Reserved\n"); break;
                  case 2: printf("Reserved\n"); break;
                  case 3: printf("Reserved\n"); break;
                  case 4: printf("Reserved\n"); break;
                  case 5: printf("Reserved\n"); break;
                  case 6: printf("Commanded Stop\n"); break;
                  case 7: printf("User Stop\n"); break;
                  case 8: printf("Capture 1 active\n"); break;
                  case 9: printf("Capture 2 active\n"); break;
                  case 10: printf("Capture 3 active\n"); break;
                  case 11: printf("Commanded Positive Limit\n"); break;
                  case 12: printf("Commanded Negative Limit\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
            case 0x07:  /* Active Configuration Status */
               printf("   Active Configuration Status: Bit %d set: ", bit);
               switch (bit)
               {
                  case 0: printf("Absolute Position valid\n"); break;
                  case 1: printf("Positive Stop Active\n"); break;
                  case 2: printf("Negative Stop Active\n"); break;
                  default: printf("Reserved\n"); break;
               }
               break;
         }
      }
   }
}


/** \brief Read diagnostic objects (2002h, 2003h, 200Fh, 2021h) via SDO after cyclic loop exits
 *  Called only when fault_detected is true, after PDO exchange has stopped.
 *  Logs each non-zero diagnostic value as FAULT_DRIVE_STATUS_FLAG with encoded detail.
 *  \param fieldbus Fieldbus context
 *  \param timestamp_s Elapsed time when fault was detected
 */
void
read_drive_status_sdo(Fieldbus *fieldbus, double timestamp_s)
{
   ecx_contextt *context = &fieldbus->context;
   int psize;
   uint16_t value16;
   int16_t value_signed16;
   int32_t value32;
   uint8_t sub;
   int32_t temp_raw;
   double temp_c;

   printf("Reading drive diagnostic objects via SDO...\n");

   /* Read 603Fh (CiA402 Error Code): the drive's own name for the most recent fault, which
    * distinguishes causes that the 2002h bitfields alone leave ambiguous. */
   psize = sizeof(value16);
   if (ecx_SDOread(context, fieldbus->amc_slave_index, ERROR_CODE_INDEX, 0x00, FALSE,
                   &psize, &value16, EC_TIMEOUTSAFE) <= 0)
   {
      printf("  603Fh (Error Code): SDO read failed (object not supported by this drive?)\n");
   }
   else
   {
      printf("  603Fh (Error Code): 0x%04X ", value16);
      switch (value16)
      {
         case 0x0000: printf("[No error]\n"); break;
         case 0x2310: printf("[Continuous over current]\n"); break;
         case 0x2320: printf("[Short circuit / earth leakage]\n"); break;
         case 0x3210: printf("[DC link over voltage]\n"); break;
         case 0x3220: printf("[DC link under voltage]\n"); break;
         case 0x4310: printf("[Drive over temperature]\n"); break;
         case 0x5530: printf("[EEPROM fault]\n"); break;
         case 0x6320: printf("[Parameter error]\n"); break;
         case 0x7300: printf("[Sensor fault]\n"); break;
         case 0x7305: printf("[Incremental sensor 1 fault]\n"); break;
         case 0x7500: printf("[Communication fault]\n"); break;
         case 0x8110: printf("[CAN overrun]\n"); break;
         case 0x8130: printf("[Life guard / heartbeat error - node guarding]\n"); break;
         case 0x8140: printf("[Recovered from bus off]\n"); break;
         case 0x8210: printf("[PDO not processed due to length error]\n"); break;
         case 0x8220: printf("[PDO length exceeded]\n"); break;
         case 0xFF00: printf("[Manufacturer specific]\n"); break;
         default: printf("[Unlisted code - check AMC documentation]\n"); break;
      }
   }

   /* Read 2002h.01h–.07h (Drive Status: current active fault flags) */
   printf("  2002h (Drive Status):\n");
   for (sub = 0x01; sub <= 0x07; sub++)
   {
      psize = sizeof(value16);
      ecx_SDOread(context, fieldbus->amc_slave_index, 0x2002, sub, FALSE, &psize, &value16, EC_TIMEOUTSAFE);
      // printf("    .%02Xh = 0x%04X\n", sub, value16);
      if (value16 != 0)
         lookup_fault_name(value16, sub);
      // log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG,
      //           (0x2000 | (sub << 8) | value16), RECOVERY_NONE);
   }

   /* Read 2003h.01h–.07h (Drive Status History: events that ever occurred) */
   printf("  2003h (Status History):\n");
   for (sub = 0x01; sub <= 0x07; sub++)
   {
      psize = sizeof(value16);
      ecx_SDOread(context, fieldbus->amc_slave_index, 0x2003, sub, FALSE, &psize, &value16, EC_TIMEOUTSAFE);
      printf("    .%02Xh = 0x%04X (history)\n", sub, value16);
      lookup_fault_name(value16, sub);
      // log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG,
      //           (0x3000 | (sub << 8) | value16), RECOVERY_NONE);
   }

   /* Read 200Fh.01h (DC Bus Voltage, Integer16, DV1 units) */
   printf("  200Fh (DC Bus Voltage):\n");
   psize = sizeof(value_signed16);
   if (ecx_SDOread(context, fieldbus->amc_slave_index, 0x200F, 0x01, FALSE,
                   &psize, &value_signed16, EC_TIMEOUTSAFE) > 0)
   {
      printf("    .01h = %d (DV1 units)\n", value_signed16);
      if (value_signed16 < 200)  /* Abnormally low voltage indicator */
      {
         log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG,
                   (0xF001 | (value_signed16 & 0xFF)), RECOVERY_NONE);
      }
   }

   /* Read 2021h.01h (Motor Temperature, Integer32, formula: value/65536 = °C) */
   printf("  2021h (Motor Temperature):\n");
   psize = sizeof(value32);
   if (ecx_SDOread(context, fieldbus->amc_slave_index, 0x2021, 0x01, FALSE,
                   &psize, &value32, EC_TIMEOUTSAFE) > 0)
   {
      temp_c = value32 / 65536.0;
      printf("    .01h = %d raw → %.2f °C\n", value32, temp_c);
      if (temp_c > 80.0)  /* Abnormally high temperature indicator */
      {
         log_fault(fieldbus, timestamp_s, FAULT_DRIVE_STATUS_FLAG,
                   (0x2100 | ((int)(temp_c) & 0xFF)), RECOVERY_NONE);
      }
   }
}
