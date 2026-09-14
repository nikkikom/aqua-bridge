"""``zones.trust_rule: sigma`` (PROJECT.md section 3 per-zone trust, section 8 item 8).

A zone is trusted under ``sigma`` iff the time status is ``first`` / ``ok``, no
unknown key arrived, its setpoint groups hold (zoned setpoint configs), every
bay declared ``occupied: true`` / ``auto`` that the estimator does not report
``empty`` has a drive sigma of at most ``estimator.sigma_fault_c``, and the
zone's air sigma is at most ``estimator.sigma_air_fault_c``. ``step`` runs the
estimator before zone trust, so the verdict reads this tick's sigma.

The rule on scripted estimator updates first; then ``step`` (the tick ordering,
an estimator fault, a zone returning without its lost sensor, switching rules by
config); then the DAS truth simulator: sensor dropouts fault far fewer zones than
under ``strict`` with every drive within its limit, a lost proximal sensor does not
lower the PI-like DAS solver's cooling (a replay of the same observations without
it), and losing a bay's or a zone's sensors for good still faults the zone once
sigma passes its threshold, with its channels held and then raised. The sigma floor
(``mpc.step`` 4b) keeps the reach of a zone with a lost group at or above ``prev``:
without it the DAS MPC, which had followed a measured warming that the blind estimate
does not show, lowered the fans on the tick of the loss.

The simulator runs use the ``basic`` preset with each sensor type's noise (the
DAS golden physics). The ``rich`` sweep (nightly) masks the example's two
redundant proximal sensors: on ``rich`` a bay's two sensors sit at different
drawn placements, the estimator fuses both into one sensor node, and its
fast-swap rule inflates that bay's sigma on every tick (up to 7 degC measured),
which faults the zone under ``sigma`` on a healthy plant (reported separately).
"""

from __future__ import annotations

import dataclasses
import math
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any
from unittest import mock

import pytest
import yaml

from aqua_bridge.config import load_config
from aqua_bridge.control import estimator, zones
from aqua_bridge.control.gate import evaluate_gate
from aqua_bridge.control.mpc import step
from aqua_bridge.control.solver_pi import PiSolver, SolverRequest, SolverResult
from aqua_bridge.model import (
    SIGMA_UNCALIBRATED_C,
    ConfigError,
    Mode,
    MpcConfig,
    MpcState,
    PlantObservation,
    ZonePolicy,
)
from aqua_bridge.sim.das import (
    SENSOR_TYPES,
    DasRun,
    build_das_plant,
    run_das_closed_loop,
    topology_from_config,
)
from conftest import EXAMPLE_DAS_CONFIG
from das_fixtures import das_cfg, das_obs, default_temps
from invariants import checked_step

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------


def with_rule(cfg: MpcConfig, rule: str, **estimator_keys: float) -> MpcConfig:
    """``cfg`` with ``zones.trust_rule`` and estimator keys replaced, through the config."""
    data = cfg.to_dict()
    data["zones"]["trust_rule"] = rule
    data["estimator"].update(estimator_keys)
    return MpcConfig.from_mapping(data)


def lcfg(rule: str = "sigma") -> MpcConfig:
    """The small zoned fixture regulating drive limits (no setpoints)."""
    return das_cfg(setpoints={}, zones={"trust_rule": rule})


def gate_for(cfg: MpcConfig, obs: PlantObservation):
    return evaluate_gate(obs, cfg, last_good_obs=None, last_raw_temps=None, window=())


def fixture_update(
    cfg: MpcConfig,
    *,
    sigma: Mapping[str, Any] | None = None,
    air: Mapping[str, Any] | None = None,
    occupancy: Mapping[str, str] | None = None,
    drop_estimates: tuple[str, ...] = (),
    uninitialised: tuple[str, ...] = (),
) -> estimator.EstimatorUpdate:
    """A real first-tick estimator update of the fixture, with sigmas, occupancy and
    zone state overridden (the rule reads nothing else)."""
    temps = {k: v for k, v in default_temps(cfg).items() if v is not None}
    up = estimator.update(None, cfg, temps=temps, u=dict.fromkeys(cfg.channels, 0.5), ts=0.0)
    est = {b: dict(e) for b, e in up.estimates.items() if b not in drop_estimates}
    for bay, value in (sigma or {}).items():
        est[bay]["sigma"] = value
    zones_out = {z: dict(info) for z, info in up.zones.items()}
    for zone, value in (air or {}).items():
        zones_out[zone]["sigma_air_c"] = value
    for zone in uninitialised:
        zones_out[zone] = {"initialised": False}
    bays = {b: dict(info) for b, info in up.bays.items()}
    for bay, occ in (occupancy or {}).items():
        bays[bay]["occupancy"] = occ
    return dataclasses.replace(up, estimates=est, zones=zones_out, bays=bays)


def trusted(verdicts: Mapping[str, zones.ZoneTrust]) -> set[str]:
    return {z for z, v in verdicts.items() if v.trusted}


# ---------------------------------------------------------------------------
# the rule on scripted estimator updates
# ---------------------------------------------------------------------------


def test_fixture_update_starts_within_both_thresholds():
    cfg = lcfg()
    up = fixture_update(cfg)
    assert set(up.estimates) == {"a1", "a2", "b1"}  # c1 is declared empty
    assert all(e["sigma"] < cfg.estimator.sigma_fault_c for e in up.estimates.values())
    assert all(
        info["initialised"] and info["sigma_air_c"] < cfg.estimator.sigma_air_fault_c
        for info in up.zones.values()
    )
    v = zones.evaluate(gate_for(cfg, das_obs(cfg, 0.0)), "first", cfg, up)
    assert trusted(v) == {"za", "zb", "zc"}


def test_sigma_at_its_threshold_is_trusted_and_above_it_names_the_bay():
    cfg = lcfg()
    limit, air_limit = cfg.estimator.sigma_fault_c, cfg.estimator.sigma_air_fault_c
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    at = fixture_update(cfg, sigma={"a2": limit}, air={"zb": air_limit})
    assert trusted(zones.evaluate(gate, "ok", cfg, at)) == {"za", "zb", "zc"}
    above = fixture_update(
        cfg, sigma={"a2": math.nextafter(limit, math.inf)}, air={"zb": air_limit + 0.25}
    )
    v = zones.evaluate(gate, "ok", cfg, above)
    assert trusted(v) == {"zc"}
    assert v["za"].reasons == ("sigma:bay:a2=4.000>4",)
    assert v["zb"].reasons == ("sigma:zone_air=2.250>2",)
    assert zones.sigma_reasons("za", cfg, above) == list(v["za"].reasons)


def test_sigma_rule_does_not_check_the_drive_and_air_groups():
    cfg = lcfg()
    obs = das_obs(cfg, 0.0, prox_a2=None, air_b=None, air_c=math.nan)
    gate = gate_for(cfg, obs)
    up = fixture_update(cfg)
    assert trusted(zones.evaluate(gate, "ok", cfg, up)) == {"za", "zb", "zc"}
    strict = lcfg("strict")
    assert trusted(zones.evaluate(gate, "ok", strict, up)) == set()


def test_empty_and_undeclared_bays_carry_no_sigma_check():
    cfg = lcfg()
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    # b1 found empty by the estimator (no estimate), c1 declared occupied: false
    up = fixture_update(cfg, occupancy={"b1": "empty"}, drop_estimates=("b1",))
    assert trusted(zones.evaluate(gate, "ok", cfg, up)) == {"za", "zb", "zc"}
    # an unknown bay without an estimate is not trusted
    up = fixture_update(cfg, occupancy={"b1": "unknown"}, drop_estimates=("b1",))
    v = zones.evaluate(gate, "ok", cfg, up)
    assert v["zb"].reasons == ("sigma:bay:b1=none>4",)


def test_a_zone_without_an_air_estimate_is_not_trusted():
    cfg = lcfg()
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    v = zones.evaluate(gate, "ok", cfg, fixture_update(cfg, uninitialised=("zc",)))
    assert trusted(v) == {"za", "zb"}
    assert v["zc"].reasons == ("sigma:zone_air=none>2",)


@pytest.mark.parametrize("bad", [None, math.nan, math.inf, "1.0", True], ids=repr)
def test_a_sigma_that_is_not_a_number_is_not_trusted(bad):
    cfg = lcfg()
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    v = zones.evaluate(gate, "ok", cfg, fixture_update(cfg, sigma={"a1": bad}, air={"zb": bad}))
    assert trusted(v) == {"zc"}


@pytest.mark.parametrize("status", ["gap", "not_advancing"])
def test_time_faults_and_unknown_keys_still_fault_every_zone(status):
    cfg = lcfg()
    up = fixture_update(cfg)
    v = zones.evaluate(gate_for(cfg, das_obs(cfg, 0.0)), status, cfg, up)
    assert trusted(v) == set() and v["zc"].reasons == (f"time:{status}",)
    v = zones.evaluate(gate_for(cfg, das_obs(cfg, 0.0, gpu=40.0)), "ok", cfg, up)
    assert trusted(v) == set() and v["zc"].reasons == ("unknown_keys:gpu",)


def test_setpoint_groups_stay_required_under_sigma():
    cfg = das_cfg(zones={"trust_rule": "sigma"})  # setpoints on air_a, air_b, air_c
    up = fixture_update(cfg)
    gate = gate_for(cfg, das_obs(cfg, 0.0, air_a=200.0, prox_b1=None))
    v = zones.evaluate(gate, "ok", cfg, up)
    assert trusted(v) == {"zb", "zc"}
    assert v["za"].reasons == ("setpoint:air_a:air_a=range",)
    # groups_confirmed: the setpoint group only, a confirming setpoint sensor counts against
    gate = gate_for(cfg, das_obs(cfg, 0.0, prox_a2=None))
    assert zones.groups_confirmed("za", gate, {}, cfg, "sigma")
    assert not zones.groups_confirmed("za", gate, {}, cfg, "strict")
    assert not zones.groups_confirmed("za", gate, {"air_a": 1}, cfg, "sigma")


def test_effective_rule():
    sigma = lcfg()
    up = fixture_update(sigma)
    assert zones.effective_trust_rule(sigma, up) == "sigma"
    assert zones.effective_trust_rule(sigma, None) == "strict"  # an estimator fault
    assert zones.effective_trust_rule(lcfg("strict"), up) == "strict"


def test_legacy_mode_has_no_sigma_rule(cfg):
    assert cfg.zones is None and zones.effective_trust_rule(cfg) == "strict"
    cmd, state = checked_step(
        PlantObservation(temps={}, rpm={}, pwm={}, ts=0.0), cfg, MpcState.cold()
    )
    assert "trust_rule" not in cmd.diagnostics and "sensor_confirm" not in state.solver_memory


@pytest.mark.parametrize(
    "patch",
    [{}, {"prox_a1": None}, {"prox_a1": None, "prox_a1b": None}, {"air_b": 500.0}, {"gpu": 1.0}],
    ids=["clean", "redundant", "bay", "air", "unknown"],
)
@pytest.mark.parametrize("status", ["first", "ok", "gap"])
def test_strict_ignores_the_estimator_update(patch, status):
    cfg = lcfg("strict")
    gate = gate_for(cfg, das_obs(cfg, 0.0, **patch))
    wide = fixture_update(cfg, sigma=dict.fromkeys(("a1", "a2", "b1"), 50.0))
    assert zones.evaluate(gate, status, cfg, wide) == zones.evaluate(gate, status, cfg, None)


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------


def test_sigma_fault_c_must_exceed_the_uncalibrated_floor_under_sigma():
    floor = SIGMA_UNCALIBRATED_C
    with pytest.raises(ConfigError, match="sigma_fault_c must be > the uncalibrated"):
        with_rule(lcfg("strict"), "sigma", sigma_fault_c=floor)
    assert with_rule(lcfg("strict"), "strict", sigma_fault_c=floor).zones.trust_rule == "strict"
    assert with_rule(lcfg("strict"), "sigma", sigma_fault_c=floor + 0.01).estimator.sigma_fault_c
    for key in ("sigma_fault_c", "sigma_air_fault_c"):
        with pytest.raises(ConfigError, match=f"{key} must be > 0"):
            with_rule(lcfg("strict"), "strict", **{key: 0.0})


def test_switching_rules_is_config_only(tmp_path):
    text = EXAMPLE_DAS_CONFIG.read_text()
    assert "zones: {trust_rule: strict," in text
    path = tmp_path / "sigma.yaml"
    path.write_text(text.replace("zones: {trust_rule: strict,", "zones: {trust_rule: sigma,"))
    strict_cfg, sigma_cfg = load_config(EXAMPLE_DAS_CONFIG).mpc, load_config(path).mpc
    assert sigma_cfg.zones == ZonePolicy(trust_rule="sigma", fault_coupling="declared")
    assert dataclasses.replace(sigma_cfg, zones=strict_cfg.zones) == strict_cfg
    assert yaml.safe_load(path.read_text())["mpc"]["zones"]["trust_rule"] == "sigma"
    for mpc_cfg, rule in ((strict_cfg, "strict"), (sigma_cfg, "sigma")):
        obs = PlantObservation(
            temps=dict.fromkeys(mpc_cfg.temps, 30.0),
            rpm={},
            pwm=dict.fromkeys(mpc_cfg.channels, 0.5),
            ts=0.0,
        )
        cmd, _ = checked_step(obs, mpc_cfg, MpcState.cold())
        assert cmd.diagnostics["trust_rule"] == rule


# ---------------------------------------------------------------------------
# step: ordering, estimator fault, return without the lost sensor
# ---------------------------------------------------------------------------


def run_fixture(
    cfg: MpcConfig,
    temps_at: Callable[[int], Mapping[str, float | None]],
    ticks: int,
    *,
    state: MpcState | None = None,
    t0: int = 0,
    ts_at: Callable[[int], float] | None = None,
) -> list[tuple[Any, MpcState]]:
    state = MpcState.cold() if state is None else state
    out = []
    for i in range(t0, t0 + ticks):
        pwm = dict(state.last_cmd.pwm) if state.last_cmd is not None else 0.5
        ts = float(i) * cfg.dt if ts_at is None else ts_at(i)
        cmd, state = checked_step(das_obs(cfg, ts, temps=dict(temps_at(i)), pwm=pwm), cfg, state)
        out.append((cmd, state))
    return out


def lost(cfg: MpcConfig, *names: str) -> dict[str, float | None]:
    t = default_temps(cfg)
    t.update(dict.fromkeys(names, None))
    return t


def test_the_verdict_reads_this_ticks_sigma():
    """A bay's only proximal sensor is lost: the zone is trusted exactly on the ticks whose
    own estimate has every sigma within its threshold, including the crossing tick."""
    base = lcfg()
    first = run_fixture(base, lambda i: default_temps(base), 3)
    sigma_now = first[-1][0].diagnostics["estimates"]["a2"]["sigma_c"]
    cfg = with_rule(base, "sigma", sigma_fault_c=sigma_now + 0.02)
    runs = run_fixture(cfg, lambda i: lost(cfg, "prox_a2") if i >= 3 else default_temps(cfg), 400)
    seen = Counter()
    crossed = None
    for k, (cmd, _state) in enumerate(runs):
        d = cmd.diagnostics
        assert d["trust_rule"] == "sigma"
        est, air = d["estimates"], d["estimator"]["zones"]
        for zone in ("za", "zb", "zc"):
            within = (
                all(
                    e["sigma_c"] <= cfg.estimator.sigma_fault_c
                    for e in est.values()
                    if e["zone"] == zone
                )
                and air[zone]["sigma_air_c"] <= cfg.estimator.sigma_air_fault_c
            )
            assert d["zones"][zone]["trusted"] == within, (k, zone)
            seen[within] += 1
        if crossed is None and not d["zones"]["za"]["trusted"]:
            crossed = k
            assert runs[k - 1][0].diagnostics["zones"]["za"]["trusted"]
            assert d["zones"]["za"]["reasons"][0].startswith("sigma:bay:a2=")
    assert crossed is not None and seen[True] and seen[False]
    assert runs[crossed][0].diagnostics["zones_in_fault"] == ["za"]


def test_an_estimator_fault_applies_strict_for_that_tick():
    cfg = das_cfg(zones={"trust_rule": "sigma"}, confirm_s=2.0)  # setpoints: only reported
    base = run_fixture(cfg, lambda i: default_temps(cfg), 4)[-1][1]
    obs = das_obs(cfg, 4.0, temps=lost(cfg, "prox_a2"), pwm=dict(base.last_cmd.pwm))
    with mock.patch.object(estimator, "update", side_effect=FloatingPointError("boom")):
        cmd, state = checked_step(obs, cfg, base)
    d = cmd.diagnostics
    assert d["trust_rule"] == "strict" and d["estimator"]["status"] == "error"
    assert d["zones"]["za"]["reasons"] == ["bay:a2:prox_a2=null"]
    assert d["zones_in_fault"] == ["za"]
    # the next tick has an update again: sigma, and the zone confirms without prox_a2
    after = run_fixture(cfg, lambda i: lost(cfg, "prox_a2"), cfg.confirm_ticks, state=state, t0=5)
    assert [c.diagnostics["trust_rule"] for c, _ in after] == ["sigma"] * cfg.confirm_ticks
    assert after[-1][0].diagnostics["zones_in_fault"] == []


@pytest.mark.parametrize(("rule", "returns"), [("sigma", True), ("strict", False)])
def test_a_zone_in_fault_returns_without_its_lost_sensor_only_under_sigma(rule, returns):
    cfg = lcfg(rule)
    base = run_fixture(cfg, lambda i: default_temps(cfg), 4)[-1][1]
    gap = run_fixture(cfg, lambda i: default_temps(cfg), 1, state=base, t0=4, ts_at=lambda i: 10.0)
    assert set(gap[-1][0].diagnostics["zones_in_fault"]) == {"za", "zb", "zc"}
    after = run_fixture(
        cfg, lambda i: lost(cfg, "prox_b1"), 8, state=gap[-1][1], t0=11, ts_at=lambda i: float(i)
    )
    faults = after[-1][0].diagnostics["zones_in_fault"]
    assert ("zb" not in faults) == returns
    assert "za" not in faults and "zc" not in faults


class DescendingSolver:
    """The PI solver's result with every demand 0.05 below ``prev``: a solver that wants
    less cooling everywhere, whatever the estimates say."""

    name = "pi"

    def __init__(self) -> None:
        self.inner = PiSolver()

    def initialise(self, cfg: MpcConfig, req: SolverRequest):
        return self.inner.initialise(cfg, req)

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        res = self.inner.solve(cfg, req)
        pwm = {ch: float(req.prev_pwm[ch]) - 0.05 for ch in cfg.channels}
        return dataclasses.replace(res, pwm=pwm)


def run_descending(
    cfg: MpcConfig, temps_at: Callable[[int], Mapping[str, float | None]], ticks: int
) -> list[Any]:
    solver = DescendingSolver()
    state = MpcState.cold()
    out = []
    for i in range(ticks):
        pwm = dict(state.last_cmd.pwm) if state.last_cmd is not None else 0.9
        obs = das_obs(cfg, float(i) * cfg.dt, temps=dict(temps_at(i)), pwm=pwm)
        cmd, state = checked_step(obs, cfg, state, solver=solver)
        out.append(cmd)
    return out


def test_a_lost_group_under_sigma_never_lowers_its_zones_channels():
    """``prox_a2`` is bay a2's only proximal sensor: under ``sigma`` zone za stays trusted,
    and while the group is lost the channels of its reach (fa1, fa2 and, coupled, fb1)
    never go below ``prev`` even when the solver asks for less; fc1 keeps descending. A
    lost redundant member (``prox_a1``, ``prox_a1b`` still there) floors nothing, and the
    floor is released when the sensor returns."""
    cfg = lcfg()
    reach = set(cfg.zone_layout.reach["za"])
    assert reach == {"fa1", "fa2", "fb1"}

    def temps_at(i: int) -> dict[str, float | None]:
        if 5 <= i < 15:
            return lost(cfg, "prox_a2")
        if 15 <= i < 20:
            return lost(cfg, "prox_a1")
        return default_temps(cfg)

    cmds = run_descending(cfg, temps_at, 25)
    for i, cmd in enumerate(cmds[1:], start=1):
        d = cmd.diagnostics
        assert d["zones_in_fault"] == [], i
        prev = cmds[i - 1].pwm
        floored = set(d["sigma_floor_channels"])
        if 5 <= i < 15:
            assert floored == reach, i
            for ch in reach:
                assert cmd.pwm[ch] >= prev[ch] - 1e-12, (i, ch)
            assert cmd.pwm["fc1"] < prev["fc1"] or prev["fc1"] <= cfg.pwm_min, i
        else:
            assert floored == set(), i
            for ch in cfg.channels:
                assert cmd.pwm[ch] < prev[ch] or prev[ch] <= cfg.pwm_min, (i, ch)


def test_strict_has_no_sigma_floor():
    cfg = lcfg("strict")
    cmds = run_descending(cfg, lambda i: default_temps(cfg), 6)
    assert all(c.diagnostics["sigma_floor_channels"] == [] for c in cmds)


# ---------------------------------------------------------------------------
# the DAS truth simulator
# ---------------------------------------------------------------------------

#: The example config at dt = 5 s: 40 minutes (PR) and 75 minutes (nightly).
PR_TICKS = 480
LONG_TICKS = 900
#: Busy bays across all four zones, so the fans regulate above pwm_min.
HEAT = {
    "b01": [(0.0, 0.6)],
    "b02": [(0.0, 1.0)],
    "b06": [(0.0, 0.8)],
    "b10": [(0.0, 1.0)],
    "b13": [(0.0, 0.7)],
}
#: Per-tick dropout probability of every DS18B20 (1-Wire CRC failures and the like).
DROPOUT = 0.02
#: The example's redundant proximal sensors (masked on ``rich``, module docstring).
REDUNDANT = ("prox_b03b", "prox_b10b")

ObsHook = Callable[[int, PlantObservation], PlantObservation]


def example_cfg(rule: str, solver: str = "pi", **estimator_keys: float) -> MpcConfig:
    cfg = load_config(EXAMPLE_DAS_CONFIG).mpc
    cfg = dataclasses.replace(cfg, solver=solver, model_accept_prior=solver == "mpc")
    return with_rule(cfg, rule, **estimator_keys)


def sim_run(
    cfg: MpcConfig,
    ticks: int,
    *,
    preset: str = "basic",
    seed: int = 1,
    dropout: float = 0.0,
    hook: ObsHook | None = None,
    controller: Callable = step,
    **plant_kw: Any,
) -> DasRun:
    topology = topology_from_config(cfg)
    for entry in topology["sensors"].values():
        if preset == "basic":
            entry["noise_sigma_c"] = SENSOR_TYPES[entry["type"]].noise_sigma_c
        if entry["type"] == "ds18b20":
            entry["dropout_prob"] = dropout
    plant_kw.setdefault("heat_schedule", HEAT)
    plant = build_das_plant(
        topology, preset=preset, seed=seed, dt=cfg.dt, initial_pwm=0.5, **plant_kw
    )
    return run_das_closed_loop(plant, cfg, controller, ticks, observe_hook=hook)


def without(names: tuple[str, ...], since_s: float = 0.0) -> ObsHook:
    def hook(i: int, obs: PlantObservation) -> PlantObservation:
        if obs.ts < since_s:
            return obs
        temps = {k: (None if k in names else v) for k, v in obs.temps.items()}
        return dataclasses.replace(obs, temps=temps)

    return hook


def zone_faults(run: DasRun) -> tuple[int, int]:
    """(fault episodes, zone-fault ticks) over the run."""
    episodes = ticks = 0
    before: set[str] = set()
    for rec in run.records:
        now = set(rec.cmd.diagnostics["zones_in_fault"])
        episodes += len(now - before)
        ticks += len(now)
        before = now
    return episodes, ticks


def assert_dropouts_fault_fewer_zones(ticks: int, preset: str, seed: int, solver: str) -> None:
    hook = without(REDUNDANT) if preset == "rich" else None
    strict = sim_run(
        example_cfg("strict", solver), ticks, preset=preset, seed=seed, dropout=DROPOUT, hook=hook
    )
    sigma = sim_run(
        example_cfg("sigma", solver), ticks, preset=preset, seed=seed, dropout=DROPOUT, hook=hook
    )
    strict_faults, sigma_faults = zone_faults(strict), zone_faults(sigma)
    assert strict_faults[0] > 50, strict_faults  # the dropouts do fault zones under strict
    # Under sigma none, except a sensor that drops out on the very first tick: the
    # estimator starts that bay's sensor node at the air, the reading that returns trips
    # the fast-swap rule for one tick (rich seed 5, strict faulted that tick as well).
    assert sigma_faults[0] <= 1, sigma_faults
    assert sigma_faults[1] <= sigma_faults[0] * (1 + example_cfg("sigma").confirm_ticks)
    early = [r.obs.ts for r in sigma.records if r.cmd.diagnostics["zones_in_fault"]]
    assert all(ts <= 60.0 for ts in early), early
    assert strict.violations() == 0 and sigma.violations() == 0
    assert all(r.cmd.diagnostics["trust_rule"] == "sigma" for r in sigma.records), (
        "every tick had an estimator update"
    )


@pytest.mark.parametrize("solver", ["pi", "mpc"])
def test_dropouts_fault_far_fewer_zones_under_sigma_than_strict(solver):
    assert_dropouts_fault_fewer_zones(PR_TICKS, "basic", 1, solver)


@pytest.mark.nightly
@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize(
    ("preset", "seed"), [("basic", s) for s in (2, 3)] + [("rich", s) for s in range(6)]
)
def test_dropouts_fault_far_fewer_zones_under_sigma_sweep(preset, seed, solver):
    assert_dropouts_fault_fewer_zones(LONG_TICKS, preset, seed, solver)


#: When the replayed sensor is lost (the fans have taken up the busy bays by then).
LOSS_S = 600.0


def replay_without(cfg: MpcConfig, healthy: DasRun, name: str) -> list[Any]:
    """``healthy``'s observations again from ``LOSS_S`` on, from the same state, with
    ``name`` missing; the plant is not closed on the new commands."""
    start = next(i for i, r in enumerate(healthy.records) if r.obs.ts >= LOSS_S)
    state = healthy.records[start - 1].state
    out = []
    for rec in healthy.records[start:]:
        obs = dataclasses.replace(
            rec.obs, temps={k: (None if k == name else v) for k, v in rec.obs.temps.items()}
        )
        cmd, state = step(obs, cfg, state)
        out.append((rec.cmd, cmd))
    return out


@pytest.fixture(scope="module")
def healthy_sigma_run() -> tuple[MpcConfig, DasRun]:
    cfg = example_cfg("sigma")
    return cfg, sim_run(cfg, PR_TICKS)


def zone_airflow(cfg: MpcConfig, zone: str, pwm: Mapping[str, float]) -> float:
    """The zone's airflow at ``pwm`` by the configured fan curves, in fan units (each
    channel's fans split over the zones that list it)."""
    layout = cfg.zone_layout
    total = 0.0
    for ch in layout.zone_channels[zone]:
        model = cfg.fan_models[cfg.fans[ch].model]
        frac = min(1.0, max(0.0, (pwm[ch] - model.deadband) / (1.0 - model.deadband)))
        total += cfg.fans[ch].count / len(layout.channel_zones[ch]) * frac**model.exponent
    return total


def test_a_lost_proximal_sensor_never_lowers_cooling(healthy_sigma_run):
    """b02's only proximal sensor, PI-like DAS: its sigma and margin grow, the zone does not
    fault, and z0's airflow never drops below the run with the sensor by more than the
    innovation the sensor would have added (measured -0.9 % at worst on three seeds); from
    ten minutes on it is higher (measured +3.8 % to +14 %, +62 % at the end)."""
    cfg, healthy = healthy_sigma_run
    pairs = replay_without(cfg, healthy, "prox_b02")
    est = [cmd.diagnostics["estimates"]["b02"] for _, cmd in pairs]
    # sigma grows while the bay is unobserved (the airflow coupling may take back ~1e-5)
    assert all(b["sigma_c"] >= a["sigma_c"] - 1e-3 for a, b in zip(est, est[1:], strict=False))
    assert all(b["soft_c"] <= a["soft_c"] + 2e-3 for a, b in zip(est, est[1:], strict=False))
    assert est[-1]["sigma_c"] > est[0]["sigma_c"] + 1.0
    ten_minutes = int(600.0 / cfg.dt)
    for k, (with_sensor, without_sensor) in enumerate(pairs):
        assert without_sensor.diagnostics["zones_in_fault"] == [], k
        q0 = zone_airflow(cfg, "z0", with_sensor.pwm)
        q1 = zone_airflow(cfg, "z0", without_sensor.pwm)
        assert q1 >= 0.98 * q0, (k, q0, q1)
        if k >= ten_minutes:
            assert q1 >= 1.02 * q0, (k, q0, q1)


def test_a_lost_proximal_sensor_never_lowers_the_das_mpc_command():
    """The DAS MPC replayed without b02's only proximal sensor: with the sensor it had
    followed a measured warming the blind estimate does not show, and it planned less
    airflow for z0 (up to 21 %, the PWM falling by 0.04 on the first tick); the sigma floor
    keeps every channel of z0's reach at or above its previous command while the sensor
    is lost."""
    cfg = example_cfg("sigma", "mpc")
    healthy = sim_run(cfg, int(LOSS_S / cfg.dt) + 1)
    pairs = replay_without(cfg, healthy, "prox_b02")
    reach = cfg.zone_layout.reach["z0"]
    start = next(i for i, r in enumerate(healthy.records) if r.obs.ts >= LOSS_S)
    prev = healthy.records[start - 1].cmd.pwm
    for k, (_with_sensor, without_sensor) in enumerate(pairs):
        assert without_sensor.diagnostics["zones_in_fault"] == [], k
        for ch in reach:
            assert without_sensor.pwm[ch] >= prev[ch] - 1e-12, (k, ch)
        prev = without_sensor.pwm


def test_a_lost_redundant_proximal_sensor_changes_nothing(healthy_sigma_run):
    """b03 keeps its other proximal sensor: sigma and the commands stay as they were."""
    cfg, healthy = healthy_sigma_run
    for name in ("prox_b03",):
        for with_sensor, without_sensor in replay_without(cfg, healthy, name):
            s0 = with_sensor.diagnostics["estimates"]["b03"]["sigma_c"]
            s1 = without_sensor.diagnostics["estimates"]["b03"]["sigma_c"]
            assert abs(s1 - s0) < 0.01
            assert without_sensor.diagnostics["zones_in_fault"] == []
            for ch in cfg.channels:
                assert abs(without_sensor.pwm[ch] - with_sensor.pwm[ch]) < 0.005


#: When the sensors of the observability-loss runs go missing for good.
LOST_S = 300.0


def assert_observability_loss_faults(
    cfg: MpcConfig, names: tuple[str, ...], zone: str, ticks: int
) -> tuple[DasRun, int]:
    """``names`` missing from ``LOST_S`` on: the zone runs on the solver with rising fans
    until a sigma passes its threshold, then faults for good; its reach holds and then
    rises to ``fallback_pwm``. Returns the run and the first fault tick."""
    run = sim_run(cfg, ticks, hook=without(names, since_s=LOST_S), controller=checked_step)
    at = next(
        (i for i, r in enumerate(run.records) if zone in r.cmd.diagnostics["zones_in_fault"]), None
    )
    assert at is not None, f"{zone} never faulted after losing {names}"
    lost = next(i for i, r in enumerate(run.records) if r.obs.ts >= LOST_S)
    assert at > lost + int(300.0 / cfg.dt), "no graceful period: the zone faulted at once"
    reasons = run.records[at].cmd.diagnostics["zones"][zone]["reasons"]
    assert reasons and all(why.startswith("sigma:bay:") for why in reasons), reasons
    if cfg.solver.value == "pi":  # the margin deficit rose with sigma (the MPC re-plans)
        before, faulted = run.records[lost].cmd.pwm, run.records[at].cmd.pwm
        assert zone_airflow(cfg, zone, faulted) > zone_airflow(cfg, zone, before)
    reach = cfg.zone_layout.reach[zone]
    hold_ticks = int(cfg.fallback_hold_s / cfg.dt)
    assert at + hold_ticks + 10 < len(run.records)
    for rec in run.records[at:]:
        d = rec.cmd.diagnostics
        assert zone in d["zones_in_fault"] and rec.cmd.mode is Mode.DEGRADED
        assert set(reach) <= set(d["fallback_channels"])
    end = run.records[-1].cmd
    for ch in reach:
        assert end.pwm[ch] >= min(cfg.fallback_pwm[ch], cfg.pwm_max) - 1e-9
        assert end.diagnostics["policy_by_channel"][ch] == "ramp_high"
    assert run.violations() == 0
    return run, at


@pytest.mark.parametrize("solver", ["pi", "mpc"])
def test_losing_a_bays_only_sensor_for_good_faults_its_zone_once_sigma_passes(solver):
    # a lower threshold than the default keeps the run short: sigma passes 2 degC about
    # 8 minutes after the loss here, the default 4 degC about 44 minutes (PI-like DAS)
    cfg = example_cfg("sigma", solver, sigma_fault_c=2.0)
    run, at = assert_observability_loss_faults(cfg, ("prox_b02",), "z0", 240)
    sigma = [r.cmd.diagnostics["estimates"]["b02"]["sigma_c"] for r in run.records]
    assert sigma[at] > 2.0 >= sigma[at - 1]


@pytest.mark.parametrize("solver", ["pi", "mpc"])
def test_losing_every_sensor_of_a_zone_faults_it_once_sigma_passes(solver):
    # at the default thresholds: about 17 minutes after the loss; the air sigma stays far
    # below sigma_air_fault_c (the model binds the air node), the drives' sigmas pass
    cfg = example_cfg("sigma", solver)
    names = tuple(n for n in cfg.temps if cfg.sensors[n].zone == "z0")
    assert set(names) >= {"air_z0", "prox_b01", "prox_b02", "prox_b03", "prox_b03b", "prox_b04"}
    assert_observability_loss_faults(cfg, names, "z0", 300)


@pytest.mark.nightly
@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize(("preset", "seed"), [("basic", 2), ("rich", 0), ("rich", 1), ("rich", 2)])
@pytest.mark.parametrize("sensor", ["prox_b02", "prox_b13"])
def test_a_bays_only_sensor_lost_for_good_keeps_every_drive_within_its_limit(
    preset, seed, solver, sensor
):
    """75 minutes with the sensor lost from 10 minutes on, default thresholds."""
    masked = REDUNDANT if preset == "rich" else ()

    def hook(i: int, obs: PlantObservation) -> PlantObservation:
        return without((sensor,), since_s=600.0)(i, without(masked)(i, obs))

    run = sim_run(example_cfg("sigma", solver), LONG_TICKS, preset=preset, seed=seed, hook=hook)
    assert run.violations() == 0


def test_a_hot_swap_holds_its_zone_for_a_few_ticks_under_sigma():
    """The estimator widens a swapped bay's sigma on purpose (the fast-swap rule on the
    removal, a 25 degC^2 drive variance on the insert), above sigma_fault_c for a tick:
    under ``sigma`` the zone faults for that tick plus its confirmation, shorter than
    ``fallback_hold_s``, so its channels only hold; the insert still raises the fans.
    ``tests/test_hotswap.py`` runs the same swap under ``strict`` without a zone fault."""
    remove_s, insert_s = 300.0, 1200.0
    cfg = example_cfg("sigma")
    run = sim_run(
        cfg,
        int((insert_s + cfg.estimator.bay_settle_s) / cfg.dt),
        seed=31,
        controller=checked_step,
        heat_schedule={"b06": [(0.0, 1.0)], "b07": [(0.0, 0.5)]},
        bay_schedule=[
            {"t_s": remove_s, "bay": "b06", "action": "remove"},
            {"t_s": insert_s, "bay": "b06", "action": "insert", "temp_c": 45.0},
        ],
    )
    episodes, ticks = zone_faults(run)
    assert 1 <= episodes <= 2 and ticks <= episodes * (cfg.confirm_ticks + 1)
    for rec in run.records:
        d = rec.cmd.diagnostics
        assert set(d["zones_in_fault"]) <= {"z1"}
        assert all(p in ("solver", "hold") for p in d["policy_by_channel"].values())
        for reason in d["zones"]["z1"]["reasons"]:
            assert reason.startswith("sigma:bay:b06="), reason
    assert run.violations() == 0
    at = next(i for i, t in enumerate(run.series["ts"]) if t >= insert_s)
    rise = max(
        max(run.series["pwm_cmd"][ch][at:]) - run.series["pwm_cmd"][ch][at]
        for ch in cfg.topology.zones["z1"].channels
    )
    assert rise > 0.05
