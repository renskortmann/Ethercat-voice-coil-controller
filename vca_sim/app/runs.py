"""Server-side store of the current run and the pinned runs.

Results hold full 2 kHz arrays, too big to send to the browser on every change, so they stay here and
the browser only holds run ids. The app is meant for one person on their own machine: one process,
one store. Per run, the slower analyses (tracking per frequency, loop gain) are cached by their settings.
"""

import itertools
import threading
from dataclasses import dataclass, field

# Blue and orange belong to the current run's signals (see plots.py). Pinned runs get magenta, aqua and
# brown: with blue and orange these five pass the dataviz validator on ALL pairs (normal-vision and
# colour-blind separation), so any two lines on screen stay distinguishable. No sixth colour passes,
# hence at most three pinned runs. A pinned run keeps its colour while it exists.
CURRENT_COLOR = "#2a78d6"
SERIES = ("#c0307b", "#1baf7a", "#8a5a00")
MAX_PINNED = len(SERIES)


@dataclass
class Run:
    id: str
    label: str
    scenario: object
    result: object
    color: str = CURRENT_COLOR
    tracking: dict = field(default_factory=dict)    # settings tuple -> rows of analysis.tracking_response
    loopgain: dict = field(default_factory=dict)    # settings tuple -> analysis.LoopGain


class RunStore:
    def __init__(self):
        self.lock = threading.RLock()
        self.current = None
        self.pinned = {}                              # id -> Run, in pinning order
        self._ids = itertools.count(1)

    def set_current(self, scenario, result):
        with self.lock:
            self.current = Run(f"run{next(self._ids)}", "current", scenario, result)
            return self.current

    def pin(self, label=""):
        """Copy the current run into the pinned set with the first free colour slot."""
        with self.lock:
            if self.current is None:
                raise ValueError("nothing to pin yet: run a scenario first")
            if len(self.pinned) >= MAX_PINNED:
                raise ValueError(f"at most {MAX_PINNED} pinned runs (one colour each); remove one first")
            used = {r.color for r in self.pinned.values()}
            color = next(c for c in SERIES if c not in used)
            sc = self.current.scenario
            n = next(self._ids)
            label = label.strip() or f"#{n} {sc.controller.NAME}, {sc.plant.key}"
            run = Run(f"pin{n}", label, sc, self.current.result, color,
                      dict(self.current.tracking), dict(self.current.loopgain))
            self.pinned[run.id] = run
            return run

    def remove(self, ids):
        with self.lock:
            for i in ids:
                self.pinned.pop(i, None)

    def get(self, run_id):
        with self.lock:
            if self.current is not None and run_id == self.current.id:
                return self.current
            return self.pinned.get(run_id)

    def shown(self, visible_ids):
        """The current run first, then the visible pinned runs in pinning order."""
        with self.lock:
            out = [self.current] if self.current is not None else []
            return out + [r for i, r in self.pinned.items() if i in set(visible_ids or ())]
