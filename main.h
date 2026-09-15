/** \file main.h
 * \brief Shared types, macros, and function prototypes for voice-coil application
 *
 * Contains all common configuration constants, PDO indices, data structures,
 * and function declarations used across the modularized source files.
 */

#ifndef MAIN_H
#define MAIN_H

/* Enable GNU extensions and POSIX.1b features */
#define _GNU_SOURCE
#define _POSIX_C_SOURCE 199309L

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>
#include <sched.h>      /* sched_setscheduler, sched_get_priority_max, SCHED_FIFO, CPU_SET */
#include <sys/mman.h>   /* mlockall, MCL_CURRENT, MCL_FUTURE */
#include <errno.h>      /* strerror(errno) for error messages */
#include <sys/prctl.h>  /* prctl, PR_SET_TIMERSLACK */
#include <unistd.h>     /* sysconf, _SC_NPROCESSORS_ONLN */

/** \brief Runtime configuration constants (modify via recompilation) */
#define CYCLE_TIME_MS       0.5  /**< EtherCAT cycle period in milliseconds */
#define SINE_FREQ_HZ        10.0 /**< Target current waveform frequency in Hz */
#define SINE_AMPLITUDE_A    2.0  /**< Target current waveform amplitude in Amps */
#define RUN_DURATION_S      10.0 /**< Total runtime in seconds */
#define CSV_DIR             "data" /**< Output directory for CSV logs */
#define MAX_SAMPLES         ((int)(RUN_DURATION_S / (CYCLE_TIME_MS / 1000.0)) + 100)
#define MAX_FAULTS          1000

/** CPU core reserved for the real-time cyclic loop. Adjust to match an isolated core
 *  (see docs/realtime-tuning.md for the matching isolcpus= kernel boot parameter). */
#define RT_CPU_CORE         1

#define AMC_VENDOR_ID       0xBD
#define CST_MODE            0x0A
#define STATUS_WORD_MASK    0x6F
/** StatusWord Fault bit. Tested on its own rather than via STATUS_WORD_MASK: CiA402 leaves bit 5
 *  (Quick stop) don't-care in the Fault state, and this drive sets it (Fault reads 0x_628). */
#define STATUS_FAULT_BIT    0x0008
#define STATE_READY_TO_SWITCH_ON  0x21
#define STATE_SWITCHED_ON         0x23
#define STATE_OPERATION_ENABLED   0x27
#define CTRL_SHUTDOWN       0x0006
#define CTRL_SWITCH_ON      0x0007
#define CTRL_ENABLE_OP      0x000F
#define CTRL_DISABLE_VOLT   0x0000
#define CTRL_FAULT_RESET    0x0080

#define SERVO_DRIVE_TYPE    0x0192

#define IO_MAP_SIZE        4096

#define DC1_SCALE          8192.0  // Scaling factor 2^13 for CiA402 DC1 current units (16-bit signed)
#define DC2_SCALE          32768.0 // Scaling factor 2^15 for CiA402 DC2 position/velocity units (32-bit signed)
#define DAI_SCALE          819.2   // Scaling factor 2^14/20 for AMC DAI analog-input-voltage units (201Ah);
                                   // volts = raw / DAI_SCALE  (AMC EtherCAT Comm Manual MNCMECRF-07, Appendix A Table A.1)
#define PBV_SCALE          10.0    // Power Board Voltage units (20D8h): volts = raw / PBV_SCALE
#define DV1_BASE           16384.0 // 2^14, numerator of DV1 (DC Bus Voltage) scaling factor 2^14/(1.05*K_OV)

// PDO object indices for CiA402 current control
#define ACTUAL_CURRENT_INDEX           0x6077    // Actual current (DC1) in 16-bit signed integer format
#define TARGET_CURRENT_INDEX           0x6071    // Target current (DC1) in 16-bit signed integer format
#define TARGET_POSITION_INDEX          0x607A    // Target position (DC2) in 32-bit signed integer format
#define TARGET_VELOCITY_INDEX          0x60FF    // Target velocity (DC2) in 32-bit signed integer format
#define CURRENT_VALUES_INDEX           0x2010    // Current values (DC1) in 16-bit signed integer format
#define CONTROL_WORD_INDEX             0x6040    // ControlWord (16-bit) for CiA402 state machine
#define STATUS_WORD_INDEX              0x6041    // StatusWord (16-bit) for CiA402 state machine
#define MODE_OF_OPERATION_INDEX        0x6060    // Mode (16-bit) for CiA402 mode of operation
#define INTERPOLATION_TIME_INDEX       0x60C2    // Interpolation Time Period (32-bit) for CST mode
#define ERROR_CODE_INDEX               0x603F    // CiA402 Error Code (16-bit) naming the most recent fault
#define AI_VALUE_INDEX                 0x201A    // Analog Input scaled value (16-bit signed)
#define POWER_BOARD_INFORMATION_INDEX  0x20D8 // Power Board Information (32-bit unsigned) for reading KP
#define POWER_BRIDGE_VALUES_INDEX      0x200F // Power Bridge Values (16-bit signed) for reading DC Bus Voltage
#define COMM_CHANNEL_ERROR_ACTION_INDEX 0x2065 // Event Action for Comm Channel Error (16-bit unsigned)
#define SYNC_MANAGER_COMM_TYPE_INDEX   0x1C00 // Sync Manager Communication Type (8-bit unsigned)
#define DEVICE_TYPE_INDEX              0x1000 // Device Type (32-bit unsigned) for identifying CiA402 drive
#define IDENTITY_OBJECT_INDEX          0x1018 // Identity Object (32-bit unsigned) for vendor/product info
#define WATCHDOG_TIMEOUT_INDEX         0x2065 // Watchdog Timeout (16-bit unsigned) for drive watchdog configuration
#define WATCHDOG_ACTION_INDEX          0x2065 // Watchdog Action (16-bit unsigned) for drive watchdog configuration
#define SYNC_MANAGER_CHANNELS_INDEX    0x1C00 // Sync Manager Channels (8-bit unsigned) for drive synchronization
#define TxPDO_INDEX                    0x1A00 // Transmit PDO mapping index for CiA402 drive
#define RxPDO_INDEX                    0x1600 // Receive PDO mapping index for CiA402 drive

#define MANTISSA_SUBINDEX                    0x01 // Sub-index for mantissa in interpolation time period object
#define EXPONENT_SUBINDEX                    0x02 // Sub-index for exponent in interpolation time period object
#define COMM_CHANNEL_ERROR_ACTION_SUBINDEX   0x21 // Sub-index for comm channel error action object
#define MAX_PEAK_CURRENT_SUBINDEX            0x0C // Sub-index for maximum peak current in power board information object
#define DC_BUS_OVER_VOLTAGE_SUBINDEX         0x09 // Sub-index for DC bus over-voltage limit in power board information object
#define VENDOR_ID_SUBINDEX                   0x01 // Sub-index for vendor ID in identity object
#define PRODUCT_CODE_SUBINDEX                0x02 // Sub-index for product code in identity object
#define REVISION_NUMBER_SUBINDEX             0x03 // Sub-index for revision number in identity object
#define SERIAL_NUMBER_SUBINDEX               0x04 // Sub-index for serial number in identity object
#define CURRENT_DEMAND_SUBINDEX              0x02 // Sub-index for Current Demand in the current values object
#define AI1_VALUE_SUBINDEX                   0x01 // Sub-index for Analog Input 1 scaled value
#define AI2_VALUE_SUBINDEX                   0x02 // Sub-index for Analog Input 2 scaled value
#define DC_BUS_VOLTAGE_SUBINDEX              0x01 // Sub-index for DC Bus Voltage in power bridge values object

/** \brief Master-to-slave process data: CiA402 ControlWord and CST target current command */
typedef struct OSAL_PACKED
{
   uint16_t controlword;     /**< CiA402 control bits (Shutdown/Switch On/Enable Operation) */
   int32_t target_position;  /**< Desired motor position (encoder units) */
   int32_t target_velocity;  /**< Desired motor velocity (encoder units/s) */
   int16_t target_current;   /**< Desired motor current, scaled by KP (DC1 units) */
} rx_pdo_t;

/** \brief Slave-to-master process data: CiA402 StatusWord, actual current, target current, and analog sensor inputs */
typedef struct OSAL_PACKED
{
   uint16_t statusword;      /**< CiA402 status bits (state machine state + fault bits) */
   int16_t actual_current;   /**< Measured motor current from drive, scaled by KP (DC1 units) */
   int16_t target_current;   /**< Desired motor current, scaled by KP (DC1 units) */
   int16_t ai1_value;        /**< Analog Input 1 scaled value (201Ah, DAI units: volts = value / DAI_SCALE) */
   int16_t ai2_value;        /**< Analog Input 2 scaled value (201Ah, DAI units: volts = value / DAI_SCALE) */
   int16_t demand_current;    /**< Current Demand from drive (DC1 units) */
   int16_t dc_bus_voltage_raw; /**< DC Bus Voltage (200Fh.01h, DV1 units: volts = raw * 1.05 * K_OV / DV1_BASE) */
} tx_pdo_t;

/** \brief Fault categories for diagnostic logging */
typedef enum
{
   FAULT_WKC_ERROR = 0,            /**< Working Counter validation failure */
   FAULT_ALstatuscode_CHANGE = 1,  /**< Drive reported non-zero ALstatuscode (reserved for expansion) */
   FAULT_STATE_DRIFT = 2,          /**< Unexpected CiA402 state transition detected */
   FAULT_DRIVE_STATUS_FLAG = 3     /**< Drive status flags (2002h) indicate fault condition */
} fault_type_t;

/** \brief Recovery actions taken in response to detected faults */
typedef enum
{
   RECOVERY_NONE = 0,              /**< Fault logged; operation continues */
   RECOVERY_AUTO_RECOVER = 1,      /**< Automatic recovery attempt made */
   RECOVERY_SHUTDOWN_INITIATED = 2 /**< Unrecoverable fault; controlled shutdown initiated */
} recovery_action_t;

/** \brief A single fault event logged during operation */
typedef struct
{
   double timestamp_s;                 /**< Absolute time when fault was detected */
   fault_type_t fault_type;            /**< Type of fault that occurred */
   uint32_t fault_detail;              /**< Additional detail (e.g., ALstatuscode value or WKC mismatch count) */
   recovery_action_t recovery_action;  /**< Action taken in response to the fault (if any) */
} fault_log_entry_t;

/** \brief Timestamped data sample from one cycle of the real-time loop */
typedef struct
{
   double timestamp_s;       /**< Absolute time when sample was acquired */
   double ai1_value_V;       /**< Analog input 1 scaled value in physical Volts (201Ah, DAI) at this timestamp */
   double ai2_value_V;       /**< Analog input 2 scaled value in physical Volts (201Ah, DAI) at this timestamp */
   double actual_current_A;  /**< Actual motor current in physical Amps at this timestamp */
   double target_current_A;  /**< Drive's reported target current in Amps at this timestamp (6071h, DC1) */
   double demand_current_A;  /**< Drive's reported current demand in Amps at this timestamp (2010h.02, DC1) */
   double dc_bus_voltage_V;  /**< DC Bus Voltage in physical Volts (200Fh.01h, DV1) at this timestamp */
   double power_W;           /**< Bus-referred electrical power: dc_bus_voltage_V * |actual_current_A| */
   double energy_J;          /**< Cumulative energy delivered to the motor up to and including this sample */
   double cycle_jitter_us;   /**< Signed offset between actual and scheduled cycle time (positive = late) */
   double pdo_exchange_us;   /**< Time spent in ecx_send_processdata + ecx_receive_processdata (frame round-trip) */
} sample_log_entry_t;

/** \brief Master state container: EtherCAT protocol context, drive parameters, and sample/fault buffers */
typedef struct
{
   ecx_contextt context;           /**< SOEM EtherCAT context with slave list and I/O mapping */
   char *iface;                    /**< Network interface name (e.g., "eth0") */
   uint8 group;                    /**< I/O group index (0 for single-group setup) */
   int roundtrip_time;             /**< Last measured PDO roundtrip time in microseconds */
   uint8 map[IO_MAP_SIZE];                /**< I/O mapping buffer for ecx_config_map_group() */
   double kp_amps;                 /**< Drive peak current rating (read from object 20D8.0Ch); used for current scaling */
   double kov_volts;               /**< DC bus over-voltage limit in volts (read from object 20D8.09h); used for DC Bus Voltage scaling */
   double cumulative_energy_J;     /**< Running trapezoidal integral of power_W over the run, in Joules */
   uint16_t amc_slave_index;       /**< Slave index of detected AMC drive (1-based) */
   sample_log_entry_t *samples;    /**< Preallocated buffer for cyclic samples */
   int sample_count;               /**< Number of samples logged so far */
   fault_log_entry_t *faults;      /**< Preallocated buffer for fault events */
   int fault_count;                /**< Number of fault events logged so far */
} Fieldbus;

/* Function prototypes */
void fieldbus_initialize(Fieldbus *fieldbus, char *iface);
int fieldbus_roundtrip(Fieldbus *fieldbus);
boolean fieldbus_start(Fieldbus *fieldbus);
void fieldbus_stop(Fieldbus *fieldbus);
int amc_slave_config(ecx_contextt *context, uint16 slave);
boolean cia402_bring_up(Fieldbus *fieldbus);
void add_timespec(struct timespec *ts, int64_t addus);
boolean fieldbus_run_cyclic(Fieldbus *fieldbus);
void log_sample(Fieldbus *fieldbus, double timestamp_s, const tx_pdo_t *tx,
                double cycle_jitter_us, double pdo_exchange_us);
void log_fault(Fieldbus *fieldbus, double timestamp_s, fault_type_t fault_type,
               uint32_t fault_detail, recovery_action_t recovery_action);
void read_drive_status_sdo(Fieldbus *fieldbus, double timestamp_s);
void export_csv(Fieldbus *fieldbus);

#endif /* MAIN_H */
