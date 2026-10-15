/** \file live_view.h
 * \brief Live stream of position, EKF position and current to scripts/live_view.py
 *
 * A ring buffer in POSIX shared memory (/dev/shm/voice_coil_live). The cyclic loop only writes
 * memory (no syscalls), so the viewer cannot add jitter; the viewer reads it whenever it likes.
 * Layout (little-endian, keep in sync with scripts/live_view.py):
 *   header, 32 bytes: uint32 magic, uint32 version, uint32 capacity, uint32 reserved,
 *                     double run_start_unix_s, uint64 write_count
 *   then capacity samples of 4 doubles: time_s, position_mm, position_filt_mm, actual_current_A
 * Sample n is in slot n % capacity; write_count is stored (release) after the slot is written.
 */

#ifndef LIVE_VIEW_H
#define LIVE_VIEW_H

#define LIVE_VIEW_SHM_NAME  "/voice_coil_live"
#define LIVE_VIEW_MAGIC     0x45564C56u   /* "VLVE" */
#define LIVE_VIEW_VERSION   1u
#define LIVE_VIEW_CAPACITY  16384u        /* samples, 8.2 s at 2 kHz */

void live_view_open(void);
void live_view_push(double t_s, double position_mm, double position_filt_mm, double current_A);
void live_view_close(void);

#endif /* LIVE_VIEW_H */
