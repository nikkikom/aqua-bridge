"""Hot swap on the DAS truth simulator, closed loop through ``mpc.step`` (plan sections 2 and 9).

Scenarios on ``config.example-das.yaml`` (every bay ``occupied: auto``, no SMART
unless stated): a drive pulled and a warm one pushed in later (through
``empty``), a drive replaced within seconds by one of another class, and a bay
that is empty at boot. Every tick goes through :func:`invariants.checked_step`.
Each must keep every true drive temperature below its limit, never fault a
zone, drop the constraints of an empty bay and raise the fans of the zone
within ``estimator.bay_settle_s`` of an insert.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable

import pytest

from aqua_bridge.config import load_config
from aqua_bridge.model import Mode, MpcConfig
from aqua_bridge.sim.das import (
    SENSOR_TYPES,
    DasRun,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)
from conftest import EXAMPLE_DAS_CONFIG
from invariants import checked_step

INSERT_TEMP_C = 45.0


def base_cfg(**estimator_keys: float) -> MpcConfig:
    """The example DAS config, with ``estimator`` keys replaced through the config."""
    cfg = load_config(EXAMPLE_DAS_CONFIG).mpc
    if not estimator_keys:
        return cfg
    data = cfg.to_dict()
    data["estimator"].update(estimator_keys)
    return MpcConfig.from_mapping(data)


def run(
    cfg: MpcConfig,
    ticks: int,
    *,
    occupied: dict[str, bool] | None = None,
    controller: Callable = checked_step,
    dropout: float = 0.0,
    **plant_kw,
) -> DasRun:
    topo = topology_from_config(cfg)
    for bay, value in (occupied or {}).items():
        topo["bays"][bay]["occupied"] = value
    for entry in topo["sensors"].values():
        entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
        if dropout and entry["type"] == "ds18b20":
            entry["dropout_prob"] = dropout
    plant = build_das_plant(topo, preset="basic", dt=cfg.dt, initial_pwm=0.5, **plant_kw)
    return run_das_closed_loop(plant, cfg, controller, ticks)


def index_at(run_: DasRun, t_s: float) -> int:
    ts = run_.series["ts"]
    assert ts[-1] >= t_s, f"the run ends at {ts[-1]} s, before {t_s} s"
    return next(i for i, t in enumerate(ts) if t >= t_s)


def assert_safe_and_fault_free(cfg: MpcConfig, run_: DasRun) -> None:
    assert run_.violations() == 0, f"worst true margin {run_.worst_margin_c():.2f} degC"
    modes = Counter(r.cmd.mode for r in run_.records)
    assert modes[Mode.DEGRADED] == 0 and modes[Mode.FALLBACK] == 0, modes


def fans_rise_after(cfg: MpcConfig, run_: DasRun, zone: str, t_insert: float) -> float:
    """Largest rise of the zone's own channels within ``bay_settle_s`` of ``t_insert``."""
    start = index_at(run_, t_insert)
    stop = index_at(run_, t_insert + cfg.estimator.bay_settle_s)
    rises = []
    for ch in cfg.topology.zones[zone].channels:
        seq = run_.series["pwm_cmd"][ch]
        rises.append(max(seq[start:stop]) - seq[start])
    return max(rises)


# ---------------------------------------------------------------------------
# a drive pulled, a warm one pushed in 15 minutes later
# ---------------------------------------------------------------------------

REMOVE_S, INSERT_S = 300.0, 1200.0


@pytest.fixture(scope="module")
def slow_swap() -> tuple[MpcConfig, DasRun]:
    cfg = base_cfg()
    return cfg, run(
        cfg,
        int((INSERT_S + cfg.estimator.bay_settle_s) / cfg.dt) + 2,
        seed=31,
        heat_schedule={"b06": [(0.0, 1.0)], "b07": [(0.0, 0.5)]},
        bay_schedule=[
            {"t_s": REMOVE_S, "bay": "b06", "action": "remove"},
            {"t_s": INSERT_S, "bay": "b06", "action": "insert", "temp_c": INSERT_TEMP_C},
        ],
    )


def test_slow_swap_is_safe_and_never_faults_a_zone(slow_swap):
    cfg, run_ = slow_swap
    assert_safe_and_fault_free(cfg, run_)


def test_slow_swap_removes_the_constraints_of_the_empty_bay(slow_swap):
    cfg, run_ = slow_swap
    confirm = cfg.estimator.empty_confirm_s
    states = [r.cmd.diagnostics["bays"]["b06"]["occupancy"] for r in run_.records]
    before = states[: index_at(run_, REMOVE_S)]
    assert set(before) == {"occupied"}
    empty_at = next(i for i, s in enumerate(states) if s == "empty")
    assert run_.series["ts"][empty_at] >= REMOVE_S + confirm
    assert run_.series["ts"][empty_at] <= REMOVE_S + confirm + 120.0
    for r in run_.records[empty_at : index_at(run_, INSERT_S)]:
        assert "b06" not in r.cmd.diagnostics["estimates"]
        assert "b06" not in r.cmd.diagnostics["solver_diag"]["worst_bay"].values()


def test_slow_swap_insert_is_seen_at_once_and_raises_the_fans(slow_swap):
    cfg, run_ = slow_swap
    at = index_at(run_, INSERT_S)
    states = [r.cmd.diagnostics["bays"]["b06"]["occupancy"] for r in run_.records[at:]]
    seen = next(i for i, s in enumerate(states) if s != "empty")
    assert seen * cfg.dt <= 60.0
    entry = run_.records[at + seen].cmd.diagnostics["estimates"]["b06"]
    assert entry["sigma_c"] > 3.0  # a wide margin from the first tick it is back
    assert fans_rise_after(cfg, run_, "z1", INSERT_S) > 0.05


# ---------------------------------------------------------------------------
# a drive replaced within seconds by one of another class
# ---------------------------------------------------------------------------

SWAP_OUT_S, SWAP_IN_S = 600.0, 630.0


@pytest.fixture(scope="module")
def quick_swap() -> tuple[MpcConfig, MpcConfig, DasRun]:
    """b12, declared ssd_sata, holds an idle SSD that is replaced within 30 s by a warm,
    busy HDD. The owner declares the new drive's serial when it goes in (``POST
    /api/bay``); its SMART model matches the hdd regex, so the bay's class follows the
    drive to the stricter limit."""
    cfg = base_cfg()
    m = cfg.to_dict()
    m["drive_classes"]["hdd"]["models"] = ["^WD"]
    m["topology"]["bays"]["b12"]["class"] = "ssd_sata"
    before = MpcConfig.from_mapping(m)
    m["topology"]["bays"]["b12"]["serial"] = "NEW-HDD"
    after = MpcConfig.from_mapping(m)

    def controller(obs, cfg_, state):
        return checked_step(obs, after if obs.ts >= SWAP_IN_S else before, state)

    return (
        before,
        after,
        run(
            before,
            int((SWAP_IN_S + cfg.estimator.bay_settle_s) / cfg.dt) + 2,
            controller=controller,
            seed=32,
            heat_schedule={"b12": [(0.0, 0.0), (SWAP_IN_S, 1.0)], "b11": [(0.0, 0.5)]},
            bay_schedule=[
                {"t_s": SWAP_OUT_S, "bay": "b12", "action": "remove"},
                {
                    "t_s": SWAP_IN_S,
                    "bay": "b12",
                    "action": "insert",
                    "class": "hdd",
                    "serial": "NEW-HDD",
                    "model": "WD80EFZX",
                    "temp_c": INSERT_TEMP_C,
                },
            ],
        ),
    )


def test_quick_swap_is_safe_and_never_faults_a_zone(quick_swap):
    before, _, run_ = quick_swap
    assert_safe_and_fault_free(before, run_)


def test_quick_swap_never_passes_through_empty_and_widens_the_margin(quick_swap):
    before, _, run_ = quick_swap
    lo, hi = index_at(run_, SWAP_OUT_S - 60.0), index_at(run_, SWAP_IN_S + 120.0)
    window = run_.records[lo:hi]
    assert all(r.cmd.diagnostics["bays"]["b12"]["occupancy"] != "empty" for r in window)
    sigma_before = window[0].cmd.diagnostics["estimates"]["b12"]["sigma_c"]
    sigma_max = max(r.cmd.diagnostics["estimates"]["b12"]["sigma_c"] for r in window)
    assert sigma_max > sigma_before + 1.0  # the fast-swap rule


def test_quick_swap_class_follows_the_new_drive(quick_swap):
    before, after, run_ = quick_swap
    old = run_.records[index_at(run_, SWAP_OUT_S - 5.0)].cmd.diagnostics["bays"]["b12"]
    assert old["class"] == "ssd_sata" and old["class_source"] == "declared"
    new = run_.records[-1].cmd.diagnostics
    assert new["bays"]["b12"]["class"] == "hdd"
    assert new["bays"]["b12"]["class_source"] == "smart_model"
    assert new["estimates"]["b12"]["limit_c"] == after.drive_classes["hdd"].limit_c


def test_quick_swap_raises_the_fans_for_the_stricter_busy_drive(quick_swap):
    before, _, run_ = quick_swap
    assert fans_rise_after(before, run_, "z2", SWAP_IN_S) > 0.05


# ---------------------------------------------------------------------------
# a bay empty at boot
# ---------------------------------------------------------------------------

BOOT_INSERT_S = 1000.0


@pytest.fixture(scope="module")
def empty_at_boot() -> tuple[MpcConfig, DasRun]:
    cfg = base_cfg()
    return cfg, run(
        cfg,
        int((BOOT_INSERT_S + cfg.estimator.bay_settle_s) / cfg.dt) + 2,
        occupied={"b03": False},
        seed=33,
        heat_schedule={"b01": [(0.0, 0.7)], "b02": [(0.0, 0.7)]},
        bay_schedule=[
            {"t_s": BOOT_INSERT_S, "bay": "b03", "action": "insert", "temp_c": INSERT_TEMP_C},
        ],
    )


def test_empty_at_boot_is_safe_and_never_faults_a_zone(empty_at_boot):
    cfg, run_ = empty_at_boot
    assert_safe_and_fault_free(cfg, run_)


def test_empty_at_boot_starts_constrained_then_turns_empty(empty_at_boot):
    cfg, run_ = empty_at_boot
    first = run_.records[0].cmd.diagnostics
    assert first["bays"]["b03"]["occupancy"] == "unknown" and "b03" in first["estimates"]
    states = [r.cmd.diagnostics["bays"]["b03"]["occupancy"] for r in run_.records]
    empty_at = states.index("empty")
    assert run_.series["ts"][empty_at] >= cfg.estimator.empty_confirm_s
    assert all(s == "empty" for s in states[empty_at : index_at(run_, BOOT_INSERT_S)])
    assert "b03" not in run_.records[empty_at].cmd.diagnostics["estimates"]


def test_empty_at_boot_insert_raises_the_fans(empty_at_boot):
    cfg, run_ = empty_at_boot
    after = [r.cmd.diagnostics["bays"]["b03"]["occupancy"] for r in run_.records]
    assert after[-1] == "occupied"
    assert fans_rise_after(cfg, run_, "z0", BOOT_INSERT_S) > 0.05


# ---------------------------------------------------------------------------
# occupancy debounce through 1-Wire dropouts (item 19)
# ---------------------------------------------------------------------------

#: Per-tick dropout probability of every DS18B20 (CRC failures and the like).
DROPOUT = 0.02


def _empty_churn(run_: DasRun, bay: str) -> tuple[int, int]:
    """(times the bay left ``empty``, ticks it was not ``empty``) after it first was."""
    leaves = away = 0
    prev = None
    seen = False
    for rec in run_.records:
        occ = rec.cmd.diagnostics["bays"][bay]["occupancy"]
        seen = seen or occ == "empty"
        if seen:
            leaves += prev == "empty" and occ != "empty"
            away += occ != "empty"
        prev = occ
    return leaves, away


@pytest.mark.parametrize("hold_s", [0.0, None])
def test_dropouts_shake_an_empty_bay_loose_only_without_the_debounce(hold_s):
    """Item 19: with ``occupancy_hold_s`` a one-tick dropout is not a loss of the bay."""
    cfg = base_cfg() if hold_s is None else base_cfg(occupancy_hold_s=hold_s)
    run_ = run(
        cfg,
        600,
        seed=5,
        dropout=DROPOUT,
        heat_schedule={"b01": [(0.0, 0.7)], "b02": [(0.0, 1.0)], "b10": [(0.0, 1.0)]},
        bay_schedule=[{"t_s": 60.0, "bay": "b06", "action": "remove"}],
    )
    leaves, away = _empty_churn(run_, "b06")
    assert run_.violations() == 0  # neither rule is unsafe
    if hold_s == 0.0:  # the old rule: a dropout costs the bay its state
        assert leaves >= 2 and away > 100, (leaves, away)
    else:
        assert leaves == 0, (leaves, away)
        assert away == 0
