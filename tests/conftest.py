import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from vca_sim import load_controller  # noqa: E402


@pytest.fixture(scope="session")
def PI():
    return load_controller(ROOT / "controllers" / "position_pi.py")


@pytest.fixture(scope="session")
def OL():
    return load_controller(ROOT / "controllers" / "open_loop_current.py")


@pytest.fixture(scope="session")
def root():
    return ROOT
