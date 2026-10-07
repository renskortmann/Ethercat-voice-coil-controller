"""Open-loop current playback: sends the reference (in A) as the target current, like the firmware's
current-mode experiments (sine, chirp, scheduled chirp, sine blocks).

Timing matches the firmware: in current mode the PC sends experiment_target_current_A(t_k) in cycle k
itself. The engine always sends a controller's output one cycle later, so this controller returns the
reference of the NEXT cycle (one sample of preview).
"""

from vca_sim.controller import Controller, Param


class OpenLoopCurrent(Controller):
    NAME = "Open-loop current"
    DESCRIPTION = "Plays the reference back as target current (firmware current-mode experiments)."
    PARAMS = (Param("scale", 1.0, -10.0, 10.0, "", help="multiplies the reference"),)
    REF_UNIT = "A"
    PREVIEW_TICKS = 1

    def step(self, m):
        nxt = m.ref_preview[0] if m.ref_preview else m.ref
        return self.p["scale"] * nxt
