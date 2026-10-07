"""Offline simulator of the VCA rig for trying control strategies. See docs/simulation-tool.md."""

from .controller import Controller, Measurement, Param, load_controller, discover
from .drive import DriveConfig, IDEAL_DRIVE
from .loop import HostConfig, Scenario, SimResult, simulate, variant
from .plant import PRESETS, DEFAULT_PRESET, PlantModel
from .sensors import LaserConfig, AccelConfig, NOISE_FREE_LASER, NOISE_FREE_ACCEL
from . import references
