"""Shared pytest configuration: Hypothesis profiles and contract-level fixtures.

Agent-specific fixtures live next to their tests, not here.
"""

from __future__ import annotations

import dataclasses
import os
from pathlib import Path

import pytest
from hypothesis import HealthCheck, settings

from aqua_bridge.config import load_config
from aqua_bridge.model import MpcConfig, SolverKind

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLE_CONFIG = REPO_ROOT / "config.example.yaml"
HWMON_ROOT = Path("/sys/class/hwmon")
AQUAERO_HWMON_NAME = "aquaero"

# --- Hypothesis profiles ----------------------------------------------------
# Select with HYPOTHESIS_PROFILE=dev|ci|nightly|pi (default: dev).
settings.register_profile("dev", max_examples=50, deadline=None, print_blob=True)
settings.register_profile(
    "ci",
    # PR and main runs: deterministic, so a red check is reproducible and
    # never caused by a lucky seed. Randomized search runs nightly.
    max_examples=200,
    derandomize=True,
    print_blob=True,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow],
)
settings.register_profile(
    "nightly",
    # Scheduled CI run: randomized, wider search. Failures print a
    # @reproduce_failure blob in the log.
    max_examples=1000,
    derandomize=False,
    print_blob=True,
    deadline=None,
    suppress_health_check=[HealthCheck.too_slow],
)
settings.register_profile(
    "pi",
    max_examples=20,
    deadline=None,
    print_blob=True,
    suppress_health_check=[HealthCheck.too_slow],
)
settings.load_profile(os.environ.get("HYPOTHESIS_PROFILE", "dev"))


# --- hardware discovery -----------------------------------------------------


def find_aquaero_hwmon(root: Path = HWMON_ROOT) -> Path | None:
    """Directory of the hwmon device whose ``name`` is ``aquaero``, or ``None``."""
    if not root.is_dir():
        return None
    for dev in sorted(root.iterdir()):
        try:
            if (dev / "name").read_text().strip() == AQUAERO_HWMON_NAME:
                return dev
        except OSError:
            continue
    return None


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip ``hardware`` tests when no aquaero hwmon device is present."""
    if find_aquaero_hwmon() is not None:
        return
    skip = pytest.mark.skip(reason="no aquaero hwmon device under /sys/class/hwmon")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)


# --- fixtures ---------------------------------------------------------------


@pytest.fixture(scope="session")
def example_config_path() -> Path:
    return EXAMPLE_CONFIG


@pytest.fixture
def cfg() -> MpcConfig:
    """Valid MpcConfig built from config.example.yaml's ``mpc`` section."""
    return load_config(EXAMPLE_CONFIG).mpc


@pytest.fixture(params=[SolverKind.PI, SolverKind.MPC], ids=["pi", "mpc"])
def solver_kind(request) -> SolverKind:
    """Both section 3 solvers (section 8: "replace PI with a small linear MPC; keep the
    same tests"). The core suites override ``cfg`` with it so every section 4 scenario
    runs once per solver; the shared ``cfg`` fixture itself stays the example config.
    """
    return request.param


@pytest.fixture
def aquaero_hwmon() -> Path:
    """Path of the live aquaero hwmon device; skips when absent."""
    dev = find_aquaero_hwmon()
    if dev is None:
        pytest.skip("no aquaero hwmon device under /sys/class/hwmon")
    return dev


@pytest.fixture
def fast_cfg(cfg: MpcConfig) -> MpcConfig:
    """Example config with tick quantities shrunk so multi-tick stories stay short.

    ``dt=1``, ``confirm_ticks=2``, ``fallback_hold_s=4``, ``stuck_ticks=4``;
    PWM limits unchanged (``pwm_min=0.15``, ``pwm_max=1.0``, ``d_pwm_max=0.1``,
    ``fallback_pwm=0.8``).
    """
    return dataclasses.replace(
        cfg, dt=1.0, confirm_s=2.0, fallback_hold_s=4.0, stuck_s=4.0, median3=False
    )
