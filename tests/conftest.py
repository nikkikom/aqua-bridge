"""Shared pytest configuration: Hypothesis profiles and contract-level fixtures.

Agent-specific fixtures live next to their tests, not here.
"""

from __future__ import annotations

import dataclasses
import os
from enum import StrEnum
from pathlib import Path

import pytest
from hypothesis import HealthCheck, settings

from aqua_bridge.config import load_config
from aqua_bridge.model import MpcConfig, SolverKind

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLE_CONFIG = REPO_ROOT / "config.example.yaml"
EXAMPLE_DAS_CONFIG = REPO_ROOT / "config.example-das.yaml"
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


class SolverCase(StrEnum):
    """What the ``solver_kind`` argument of the core suites stands for.

    * ``pi`` / ``mpc`` -- the legacy config (``config.example.yaml``) with that solver
    * ``pi_das``       -- the zoned DAS config (``config.example-das.yaml``) with the
      ``pi`` solver in its margin-deficit form (plan section 4)

    A ``SolverCase`` is a ``str``, so ``dataclasses.replace(cfg, solver=case)`` works
    for the legacy cases; :attr:`kind` is the ``SolverKind`` of every case.
    """

    PI = "pi"
    MPC = "mpc"
    PI_DAS = "pi_das"

    @property
    def kind(self) -> SolverKind:
        return SolverKind.MPC if self is SolverCase.MPC else SolverKind.PI

    @property
    def das(self) -> bool:
        return self is SolverCase.PI_DAS


LEGACY_SOLVER_CASES: tuple[SolverCase, ...] = (SolverCase.PI, SolverCase.MPC)
DAS_SOLVER_CASES: tuple[SolverCase, ...] = (SolverCase.PI_DAS,)


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    """Parametrise ``solver_kind`` over every :class:`SolverCase` (section 8: "keep the
    same tests" for every solver).

    A test or module marked ``@pytest.mark.solver_cases("pi", "mpc")`` runs only those
    cases: the legacy core suites are written against the coolant example config and
    run the legacy cases; ``tests/test_das_core.py`` runs the DAS ones.
    """
    if "solver_kind" not in metafunc.fixturenames:
        return
    marker = metafunc.definition.get_closest_marker("solver_cases")
    cases = tuple(SolverCase(c) for c in marker.args) if marker else tuple(SolverCase)
    metafunc.parametrize("solver_kind", cases, ids=[c.value for c in cases])


@pytest.fixture
def aquaero_hwmon() -> Path:
    """Path of the live aquaero hwmon device; skips when absent."""
    dev = find_aquaero_hwmon()
    if dev is None:
        pytest.skip("no aquaero hwmon device under /sys/class/hwmon")
    return dev


@pytest.fixture(scope="session")
def example_das_config_path() -> Path:
    return EXAMPLE_DAS_CONFIG


@pytest.fixture
def das_example_cfg() -> MpcConfig:
    """Valid zoned MpcConfig built from config.example-das.yaml's ``mpc`` section."""
    return load_config(EXAMPLE_DAS_CONFIG).mpc


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
