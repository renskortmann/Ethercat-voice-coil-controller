#!/usr/bin/env python3
"""Live view of a running experiment: position x (raw laser), x_hat (EKF x(k|k)) and the actual current.

Reads the shared-memory ring buffer the controller writes every cycle (/dev/shm/voice_coil_live, layout in
live_view.h). Start it before or during a run, as a normal user; it waits for a run, clears itself when a new
run starts and keeps showing the last samples after a run ends. x_hat is only there in closed-loop runs with
POS_KF_ENABLE; the controller never waits for this script, so it cannot disturb the loop.

    .venv/bin/python scripts/live_view.py              # last 5 s
    .venv/bin/python scripts/live_view.py --window 2
"""
from __future__ import annotations

import argparse
import mmap
import os
import struct

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

SHM_PATH = "/dev/shm/voice_coil_live"
MAGIC, VERSION = 0x45564C56, 1
HEADER = struct.Struct("<IIIIdQ")        # magic, version, capacity, reserved, run_start_unix_s, write_count
WRITE_COUNT_OFFSET = 24
SAMPLE_DOUBLES = 4                       # time_s, position_mm, position_filt_mm, actual_current_A


class Stream:
    """Reader for the ring buffer; returns the samples written since the last call."""

    def __init__(self) -> None:
        self.mm: mmap.mmap | None = None
        self.run_start = None
        self.read = 0

    def _attach(self) -> bool:
        if self.mm is not None:
            return True
        try:
            fd = os.open(SHM_PATH, os.O_RDONLY)
        except FileNotFoundError:
            return False
        try:
            if os.fstat(fd).st_size < HEADER.size:
                return False
            self.mm = mmap.mmap(fd, 0, prot=mmap.PROT_READ)
        finally:
            os.close(fd)
        return True

    def poll(self) -> tuple[bool, np.ndarray]:
        """(new_run, samples as an (n, 4) array)."""
        empty = np.empty((0, SAMPLE_DOUBLES))
        if not self._attach():
            return False, empty
        if self.mm.size() < HEADER.size:        # the controller is recreating it
            self.mm.close()
            self.mm = None
            return False, empty
        magic, version, cap, _, run_start, _ = HEADER.unpack_from(self.mm, 0)
        if magic != MAGIC or version != VERSION or cap == 0:
            return False, empty
        new_run = run_start != self.run_start
        if new_run:
            self.run_start, self.read = run_start, 0
        count = struct.unpack_from("<Q", self.mm, WRITE_COUNT_OFFSET)[0]
        if count < self.read:                    # buffer was reset under us
            self.read = 0
        start = max(self.read, count - cap)
        if count == start:
            return new_run, empty
        ring = np.frombuffer(self.mm, dtype="<f8", count=cap * SAMPLE_DOUBLES,
                             offset=HEADER.size).reshape(cap, SAMPLE_DOUBLES)
        idx = np.arange(start, count) % cap
        data = ring[idx].copy()
        # Slots the writer may have overwritten while we copied are dropped
        after = struct.unpack_from("<Q", self.mm, WRITE_COUNT_OFFSET)[0]
        keep = np.arange(start, count) >= after - cap
        self.read = count
        return new_run, data[keep]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--window", type=float, default=5.0, help="seconds shown (default 5; the buffer holds 8 s)")
    ap.add_argument("--fps", type=float, default=20.0, help="redraw rate (default 20)")
    args = ap.parse_args()

    stream = Stream()
    buf = np.empty((0, SAMPLE_DOUBLES))

    fig, (ax_x, ax_i) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
    (l_x,) = ax_x.plot([], [], color="0.6", lw=0.8, label="x  (raw laser)")
    (l_xh,) = ax_x.plot([], [], color="k", lw=1.2, label="x̂  (EKF x(k|k))")
    (l_i,) = ax_i.plot([], [], color="tab:blue", lw=1.0, label="actual current")
    ax_x.set_ylabel("position (mm)")
    ax_i.set_ylabel("current (A)")
    ax_i.set_xlabel("time_s (s)")
    for ax in (ax_x, ax_i):
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")
    title = fig.suptitle(f"waiting for a run ({SHM_PATH})")

    def update(_frame):
        nonlocal buf
        new_run, data = stream.poll()
        if new_run:
            buf = np.empty((0, SAMPLE_DOUBLES))
        if len(data):
            buf = np.vstack((buf, data))
        if not len(buf):
            return l_x, l_xh, l_i
        t_end = buf[-1, 0]
        buf = buf[buf[:, 0] >= t_end - args.window]
        t, x, xh, i = buf.T
        l_x.set_data(t, x)
        l_xh.set_data(t, xh)
        l_i.set_data(t, i)
        ax_x.set_xlim(t_end - args.window, t_end)
        for ax, ys in ((ax_x, (x, xh)), (ax_i, (i,))):
            y = np.concatenate(ys)
            y = y[np.isfinite(y)]
            if len(y):
                lo, hi = y.min(), y.max()
                pad = max(0.05 * (hi - lo), 0.05)
                ax.set_ylim(lo - pad, hi + pad)
        title.set_text(f"t = {t_end:7.3f} s   x = {x[-1]:+.3f} mm   x̂ = {xh[-1]:+.3f} mm   i = {i[-1]:+.2f} A")
        return l_x, l_xh, l_i

    _anim = FuncAnimation(fig, update, interval=1000.0 / args.fps, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
