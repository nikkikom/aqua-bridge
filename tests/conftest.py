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
from aqua_bridge.hw import w1_netlink
from aqua_bridge.hw.aquacomputer import AQUAERO
from aqua_bridge.hw.hidraw import (
    DEFAULT_DEV_DIR,
    DEFAULT_SYSFS_ROOT,
    HidrawInfo,
    list_hidraw_devices,
    matches_kind,
)
from aqua_bridge.model import MpcConfig, SolverKind

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLE_CONFIG = REPO_ROOT / "config.example.yaml"
EXAMPLE_DAS_CONFIG = REPO_ROOT / "config.example-das.yaml"

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


def find_aquaero_hidraw(
    sysfs_root: Path = DEFAULT_SYSFS_ROOT, dev_dir: Path = DEFAULT_DEV_DIR
) -> HidrawInfo | None:
    """The aquaero's status/control hidraw node (USB 0c70:f001, interface 2), or ``None``.

    Several attached aquaeros: the first one. Discovery only; nothing is opened.
    """
    for info in list_hidraw_devices(sysfs_root, dev_dir):
        if matches_kind(info, AQUAERO):
            return info
    return None


@pytest.fixture(autouse=True)
def no_netlink_socket(monkeypatch: pytest.MonkeyPatch) -> None:
    """Every test runs with no netlink family, so nothing can open that socket.

    The 1-Wire netlink tier (``hw/w1_netlink.py``) talks to the running
    kernel's w1 connector. On a dev machine or a CI runner that means a socket
    which either answers about somebody else's hardware or -- far more likely
    -- answers nothing at all until the bounded wait expires, which is neither
    offline nor fast. Patching the module's one seam is enough and touches
    nothing else: a missing family raises the same
    ``W1NetlinkUnavailable`` a kernel without the connector does, so the
    tier-fall-through tests see the real failure shape, while the tests of the
    transport itself inject a fake socket and never consult the family
    (``tests/test_hw_w1_netlink.py``). The HTTP and MQTT suites keep binding
    real loopback ports; only netlink is out of reach.
    """
    monkeypatch.setattr(w1_netlink, "netlink_family", lambda: None)


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip ``hardware`` tests when no aquaero hidraw device is present."""
    if find_aquaero_hidraw() is not None:
        return
    skip = pytest.mark.skip(reason=f"no aquaero hidraw device under {DEFAULT_SYSFS_ROOT}")
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
    * ``mpc_das``      -- the same DAS config with the ``mpc`` solver, i.e. the DAS MPC
      (``control/solver_das.py``); the DAS suites run it with ``model_accept_prior: true``
      so its MPC path acts rather than its PI-like fallback

    A ``SolverCase`` is a ``str``, so ``dataclasses.replace(cfg, solver=case)`` works
    for the legacy cases; :attr:`kind` is the ``SolverKind`` of every case.
    """

    PI = "pi"
    MPC = "mpc"
    PI_DAS = "pi_das"
    MPC_DAS = "mpc_das"

    @property
    def kind(self) -> SolverKind:
        return SolverKind.MPC if self in (SolverCase.MPC, SolverCase.MPC_DAS) else SolverKind.PI

    @property
    def das(self) -> bool:
        return self in (SolverCase.PI_DAS, SolverCase.MPC_DAS)


LEGACY_SOLVER_CASES: tuple[SolverCase, ...] = (SolverCase.PI, SolverCase.MPC)
DAS_SOLVER_CASES: tuple[SolverCase, ...] = (SolverCase.PI_DAS, SolverCase.MPC_DAS)


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
def aquaero_hidraw() -> HidrawInfo:
    """The live aquaero hidraw node; skips when absent."""
    info = find_aquaero_hidraw()
    if info is None:
        pytest.skip(f"no aquaero hidraw device under {DEFAULT_SYSFS_ROOT}")
    return info


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
