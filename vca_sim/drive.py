"""The AMC drive in current mode (CST): target current in, coil current out.

What the PC sends goes through the same conversion as the firmware (amps_to_target_current_raw):
clamped to +-KP and rounded to KP / 32768. The drive then makes the coil current with

    lag_s * dy/dt = c_del - y                       (current loop)
    shelf_tau_s * dl/dt = y - l                     (shelf)
    i = gain_hf * y + (gain_dc - gain_hf) * l

where c_del is the command delayed by `delay_ticks` cycles and held for one cycle. The shelf gives
gain_dc at low frequency and gain_hf above about 1 / (2 pi shelf_tau_s). The shape and the defaults
come from calibration.fit_drive() on the 2 Oct plain chirps (command recomputed from the file name,
compared with actual_current_A); see the comments on DriveConfig.

Over one cycle the command is constant, so the coil current has the closed form
i(s) = level + a1 exp(-s / lag_s) + a2 exp(-s / shelf_tau_s), which the plant integration evaluates.

The reported actual current (6077h) is the coil current just before the new command takes effect,
rounded to KP / 8192, with the noise seen in the 0 A idle windows.

The plant presets were fitted against the reported current, so their motor constant already includes
whatever difference there is between reported and true current.
"""

import math
from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class DriveConfig:
    kp_A: float = 60.0              # drive peak current KP: command step 1.831 mA = KP / 32768 in every log
    # calibration.fit_drive() on the 2 Oct plain chirps (train / test): 1.009 / 1.010, 0.851 / 0.847,
    # 25.2 / 26.1 ms, lag 0.01 / 0.03 ms, 2 cycles. Residual 0.23-0.26 A rms (about 5 % of the signal),
    # not explained by velocity (back-EMF) or position: an open question, not modelled.
    gain_dc: float = 1.01
    gain_hf: float = 0.849
    shelf_tau_s: float = 0.0256
    lag_s: float = 0.0              # current-loop lag; the fit puts it at about 0
    delay_ticks: int = 2            # whole cycles from command to coil current; +1 more to the report
    quantise_command: bool = True   # round to KP / 32768, as the RxPDO does
    quantise_report: bool = True    # round the reported current to KP / 8192 (DC1 units)
    report_noise_A: float = 0.023   # std of actual_current_A in the 0 A idle windows (2 Oct logs)

    def describe(self):
        return asdict(self)


IDEAL_DRIVE = DriveConfig(gain_dc=1.0, gain_hf=1.0, shelf_tau_s=0.0, lag_s=0.0, delay_ticks=0,
                          quantise_command=False, quantise_report=False, report_noise_A=0.0)


class Drive:
    """Run-time state of the drive. One instance per simulation run."""

    def __init__(self, cfg: DriveConfig, report_noise):
        self.cfg = cfg
        self.cmd_step = cfg.kp_A / 32768.0
        self.rep_step = cfg.kp_A / 8192.0
        self.queue = [0.0] * cfg.delay_ticks         # commands waiting for the delay, oldest first
        self.c_del = 0.0
        self.y = 0.0                                 # current-loop lag output
        self.l = 0.0                                 # shelf low-pass state
        self.g_hf, self.g_dc = cfg.gain_hf, cfg.gain_dc
        self.tc = max(cfg.lag_s, 0.0)
        self.ts = cfg.shelf_tau_s if (cfg.shelf_tau_s > 0.0 and cfg.gain_dc != cfg.gain_hf) else 0.0
        if self.tc > 0.0 and self.ts > 0.0 and abs(self.tc - self.ts) < 1e-9:
            self.tc *= 1.000001                      # keep the two exponentials distinct (closed form below)
        self.report_noise = report_noise             # pre-generated list, one value per tick

    def to_wire(self, amps):
        """The firmware's amps -> raw -> amps round trip: 0 if not finite, clamped to +-KP, quantised."""
        if not math.isfinite(amps):
            return 0.0
        raw = max(-32767.0, min(32767.0, amps / self.cmd_step))
        if self.cfg.quantise_command:
            raw = float(round(raw))
        return raw * self.cmd_step

    def send(self, amps):
        """Command sent this cycle. Returns what reaches the drive, and sets the level for this cycle."""
        wire = self.to_wire(amps)
        self.queue.append(wire)
        self.c_del = self.queue.pop(0)
        if self.tc == 0.0:
            self.y = self.c_del                      # no lag: the loop output jumps to the command
        return wire

    def _coeffs(self):
        """Closed form over this cycle, with c the held command:
        y(s) = c + dy e^(-s/tc);  l(s) = c + B e^(-s/tc) + dl e^(-s/ts)."""
        c = self.c_del
        dy = self.y - c
        B = dy * self.tc / (self.tc - self.ts) if (self.tc > 0.0 and self.ts > 0.0) else 0.0
        dl = self.l - c - B
        return c, dy, B, dl

    def segment(self):
        """Coil current over this cycle: i(s) = level + a1 exp(-s / t1) + a2 exp(-s / t2), s = time into
        the cycle. A time constant of 0 means that term is absent."""
        c, dy, B, dl = self._coeffs()
        a1 = (self.g_hf * dy + (self.g_dc - self.g_hf) * B) if self.tc > 0.0 else 0.0
        a2 = (self.g_dc - self.g_hf) * dl if self.ts > 0.0 else 0.0
        return self.g_dc * c, a1, self.tc, a2, self.ts

    def advance(self, dt):
        """End of cycle: move the lag and shelf states on by dt."""
        c, dy, B, dl = self._coeffs()
        e1 = math.exp(-dt / self.tc) if self.tc > 0.0 else 0.0
        self.l = c + B * e1 + dl * math.exp(-dt / self.ts) if self.ts > 0.0 else c + dy * e1
        self.y = c + dy * e1

    def current_now(self):
        """Coil current at the end of the cycle that just finished (before the next command)."""
        return self.g_hf * self.y + (self.g_dc - self.g_hf) * (self.l if self.ts > 0.0 else self.y)

    def report(self, i_true, k):
        """Reported actual current for cycle k, from the true coil current."""
        i = i_true + self.report_noise[k]
        if self.cfg.quantise_report:
            i = round(i / self.rep_step) * self.rep_step
        return i
