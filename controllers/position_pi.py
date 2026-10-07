"""Position PI, a copy of pi_update() on the origin/position_control branch (control_loop.c, main.h).

Per cycle, as the firmware does:
    y_f = notch(y)                     (optional 50 Hz biquad, runs in the idle window too)
    e   = r - y_f
    P   = Kp e
    I   = I + Ki e dt, unless P + I is beyond the limit and e pushes further out (conditional integration)
    I   = clamp(I, +-limit)
    u   = clamp(P + I, +-limit)
In the idle window the integrator is held at 0 and the output is 0. A non-finite reference or position
gives 0 A and leaves the integrator alone.
"""

import math

from vca_sim.controller import Controller, Param


class PositionPI(Controller):
    NAME = "Position PI (firmware)"
    DESCRIPTION = "pi_update() from origin/position_control: PI on the laser position, optional 50 Hz notch."
    PARAMS = (
        Param("Kp", 1.0, 0.0, 100.0, "A/mm", help="main.h PID_KP_A_PER_MM"),
        Param("Ki", 0.12, 0.0, 1000.0, "A/(mm s)", help="main.h PID_KI_A_PER_MM_S"),
        Param("limit", 10.0, 0.0, 60.0, "A", help="main.h PID_OUTPUT_LIMIT_A"),
        Param("notch", False, kind="bool", help="main.h POS_NOTCH_ENABLE"),
        Param("notch_hz", 50.0, 1.0, 999.0, "Hz", help="main.h POS_NOTCH_FREQ_HZ"),
        Param("notch_q", 10.0, 0.1, 1000.0, "", help="main.h POS_NOTCH_Q"),
    )
    SIGNALS = ("p_A", "i_A", "position_filt_mm")
    REF_UNIT = "mm"

    def reset(self):
        self.integ = 0.0
        # RBJ cookbook notch, bilinear transform at the cycle rate (firmware notch_init)
        w0 = 2 * math.pi * self.p["notch_hz"] * self.dt
        alpha = math.sin(w0) / (2 * self.p["notch_q"])
        a0 = 1 + alpha
        self.b0, self.b1, self.b2 = 1 / a0, -2 * math.cos(w0) / a0, 1 / a0
        self.a1, self.a2 = -2 * math.cos(w0) / a0, (1 - alpha) / a0
        self.z1 = self.z2 = 0.0
        self.seeded = False

    def _notch(self, x):
        """Direct form II transposed; seeded with the first sample so a constant passes without a transient."""
        if not self.seeded:
            self.z2 = (self.b2 - self.a2) * x
            self.z1 = (self.b1 - self.a1) * x + self.z2
            self.seeded = True
        y = self.b0 * x + self.z1
        self.z1 = self.b1 * x - self.a1 * y + self.z2
        self.z2 = self.b2 * x - self.a2 * y
        if not math.isfinite(y):          # firmware: re-seed on a non-finite result
            self.seeded = False
            y = x
        return y

    def step(self, m):
        y = self._notch(m.position_mm) if self.p["notch"] else m.position_mm
        if not m.active:
            self.integ = 0.0
            return 0.0, (0.0, 0.0, y)
        r, lim = m.ref, self.p["limit"]
        if not (math.isfinite(r) and math.isfinite(y)):
            return 0.0, (0.0, self.integ, y)
        e = r - y
        p = self.p["Kp"] * e
        i = self.integ + self.p["Ki"] * e * self.dt
        unsat = p + i
        if (unsat > lim and e > 0.0) or (unsat < -lim and e < 0.0):
            i = self.integ                 # saturated: hold the integrator
        i = max(-lim, min(lim, i))
        if not math.isfinite(i):
            i = 0.0
        self.integ = i
        u = max(-lim, min(lim, p + i))
        if not math.isfinite(u):
            u = 0.0
        return u, (p, i, y)
