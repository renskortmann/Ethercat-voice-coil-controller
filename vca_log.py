"""Load voice-coil CSV logs, in both the old and the new (bias-window) format.

New logs (firmware with BIAS_IDLE_S) start with a 0 A idle window at negative time_s and
carry an ``ai2_corrected_V`` column: the accelerometer voltage minus its mean over that
window. Old logs have neither. ``load_log`` hides the difference: every caller gets an
experiment-only table (t >= 0) that always has a bias-free ``ai2_corrected_V`` column, plus
the idle rows on the side. See docs/accelerometer-bias.md for the whole process.
"""

import warnings
from dataclasses import dataclass

import numpy as np
import pandas as pd


BIAS_FROM_IDLE = "idle window"
BIAS_FROM_RUN_MEAN = "whole-run mean"


@dataclass
class VcaLog:
    run: pd.DataFrame   # experiment rows (t >= 0; all rows for old files), always has ai2_corrected_V
    idle: pd.DataFrame  # 0 A bias-window rows (t < 0); empty for old files
    bias_V: float       # accelerometer offset that was removed, in volts
    bias_source: str    # BIAS_FROM_IDLE or BIAS_FROM_RUN_MEAN
    idle_std_V: float   # std of the accelerometer over the idle window; NaN without one
    path: str


def load_log(path) -> VcaLog:
    """Read one log, remove the accelerometer bias and split off the idle window."""
    raw = pd.read_csv(path)
    raw.columns = [c.strip() for c in raw.columns]  # some headers carry a leading space

    # The firmware stamps its first two rows with the same time. Keeping the later one is
    # exactly the old ``iloc[1:]`` for old logs, and stays correct now that the duplicate
    # sits at the start of the idle window instead of at t = 0.
    raw = raw[~raw["time_s"].duplicated(keep="last")].reset_index(drop=True)

    is_idle = raw["time_s"].to_numpy() < 0.0
    has_firmware_bias = (
        "ai2_corrected_V" in raw.columns
        and is_idle.any()
        and raw.loc[is_idle, "ai2_corrected_V"].notna().any()
    )

    if has_firmware_bias:
        # The firmware subtracted one constant, so this difference is that constant on every
        # row; the median just guards against print rounding.
        bias_V = float(np.median(raw.loc[is_idle, "ai2_value_V"] - raw.loc[is_idle, "ai2_corrected_V"]))
        bias_source = BIAS_FROM_IDLE
        idle_std_V = float(raw.loc[is_idle, "ai2_value_V"].std())
    else:
        if "ai2_corrected_V" in raw.columns:
            warnings.warn(f"{path}: ai2_corrected_V is empty (no usable idle window); "
                          "falling back to the whole-run mean as accelerometer bias")
        # Old behaviour: the mover starts and ends at rest, so the whole-run mean is taken as
        # the sensor offset. Only approximately true, which is why new logs have an idle window.
        bias_V = float(raw["ai2_value_V"].mean())
        bias_source = BIAS_FROM_RUN_MEAN
        idle_std_V = float("nan")
        raw["ai2_corrected_V"] = raw["ai2_value_V"] - bias_V

    return VcaLog(
        run=raw[~is_idle].reset_index(drop=True),
        idle=raw[is_idle].reset_index(drop=True),
        bias_V=bias_V,
        bias_source=bias_source,
        idle_std_V=idle_std_V,
        path=str(path),
    )


def bias_report(logs: dict, acc_scale_V_per_g: float) -> pd.DataFrame:
    """One row per run: removed accelerometer bias, where it came from, and idle-window noise."""
    rows = []
    for name, log in logs.items():
        dt = np.median(np.diff(log.run["time_s"]))
        rows.append({
            "run": name,
            "bias source": log.bias_source,
            "bias (V)": log.bias_V,
            "bias (g)": log.bias_V / acc_scale_V_per_g,
            "idle (s)": len(log.idle) * dt,
            "idle std (mV)": log.idle_std_V * 1e3,
        })
    return pd.DataFrame(rows).set_index("run")
