/** \file live_view.c
 * \brief Live stream of position, EKF position and current to scripts/live_view.py (see live_view.h)
 */

#include "main.h"
#include "live_view.h"
#include <fcntl.h>
#include <sys/stat.h>

typedef struct
{
   uint32_t magic;
   uint32_t version;
   uint32_t capacity;
   uint32_t reserved;
   double run_start_unix_s;
   uint64_t write_count;
} live_view_header_t;

typedef struct
{
   double t_s;
   double position_mm;
   double position_filt_mm;
   double current_A;
} live_view_sample_t;

_Static_assert(sizeof(live_view_header_t) == 32, "live_view_header_t layout must match scripts/live_view.py");
_Static_assert(sizeof(live_view_sample_t) == 32, "live_view_sample_t layout must match scripts/live_view.py");

#define LIVE_VIEW_SIZE (sizeof(live_view_header_t) + LIVE_VIEW_CAPACITY * sizeof(live_view_sample_t))

static live_view_header_t *lv_header = NULL;
static live_view_sample_t *lv_samples = NULL;
static uint64_t lv_count = 0;

/** \brief Create (or reset) the shared-memory ring buffer. Call before the cyclic loop: it does
 *  syscalls and touches every page so the loop never page-faults on it. On failure the stream is
 *  off and live_view_push() does nothing; the run itself is not affected. */
void
live_view_open(void)
{
   int fd = shm_open(LIVE_VIEW_SHM_NAME, O_CREAT | O_RDWR, 0644);
   if (fd < 0)
   {
      printf("WARNING: live view off, shm_open(%s) failed: %s\n", LIVE_VIEW_SHM_NAME, strerror(errno));
      return;
   }
   fchmod(fd, 0644);   /* readable by a viewer running as a normal user, whatever the umask */
   if (ftruncate(fd, LIVE_VIEW_SIZE) != 0)
   {
      printf("WARNING: live view off, ftruncate failed: %s\n", strerror(errno));
      close(fd);
      return;
   }
   void *p = mmap(NULL, LIVE_VIEW_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
   close(fd);
   if (p == MAP_FAILED)
   {
      printf("WARNING: live view off, mmap failed: %s\n", strerror(errno));
      return;
   }
   memset(p, 0, LIVE_VIEW_SIZE);   /* prefault (mlockall MCL_FUTURE keeps it resident) */
   lv_header = (live_view_header_t *)p;
   lv_samples = (live_view_sample_t *)((char *)p + sizeof(live_view_header_t));
   lv_count = 0;

   struct timespec now;
   clock_gettime(CLOCK_REALTIME, &now);
   lv_header->capacity = LIVE_VIEW_CAPACITY;
   lv_header->version = LIVE_VIEW_VERSION;
   lv_header->run_start_unix_s = now.tv_sec + now.tv_nsec * 1e-9;
   __atomic_store_n(&lv_header->write_count, 0, __ATOMIC_RELEASE);
   __atomic_store_n(&lv_header->magic, LIVE_VIEW_MAGIC, __ATOMIC_RELEASE);   /* last: header is valid */
   printf("Live view: /dev/shm%s (run scripts/live_view.py to watch)\n", LIVE_VIEW_SHM_NAME);
}

/** \brief Append one sample; memory writes only, safe in the cyclic loop */
void
live_view_push(double t_s, double position_mm, double position_filt_mm, double current_A)
{
   if (!lv_header)
   {
      return;
   }
   lv_samples[lv_count % LIVE_VIEW_CAPACITY] =
      (live_view_sample_t){ t_s, position_mm, position_filt_mm, current_A };
   lv_count++;
   __atomic_store_n(&lv_header->write_count, lv_count, __ATOMIC_RELEASE);
}

/** \brief Unmap the buffer. The shared memory stays, so the viewer keeps showing the last samples. */
void
live_view_close(void)
{
   if (lv_header)
   {
      munmap(lv_header, LIVE_VIEW_SIZE);
      lv_header = NULL;
      lv_samples = NULL;
   }
}
