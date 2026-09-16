"""``zones.trust_rule: sigma`` (PROJECT.md section 3 per-zone trust, section 8 items 8,
67, 69 and 70).

A zone is trusted under ``sigma`` iff the time status is ``first`` / ``ok``, no
unknown key arrived, its setpoint groups hold (zoned setpoint configs), every
bay declared ``occupied: true`` / ``auto`` that the estimator does not report
``empty`` has a drive sigma of at most ``estimator.sigma_fault_c`` (unless the
estimator reports it ``settling`` and ``observed``), the
zone's air sigma is at most ``estimator.sigma_air_fault_c``, and its air node has
had a trusted reading within ``estimator.air_blind_fault_s``. ``step`` runs the
estimator before zone trust, so the verdict reads this tick's sigma.

The rule on scripted estimator updates first; then ``step`` (the tick ordering,
an estimator fault, a zone returning without its lost sensor, switching rules by
config); then the DAS truth simulator: sensor dropouts fault no zone at all where
``strict`` faults dozens, with every drive within its limit; a lost proximal sensor does
not lower the PI-like DAS solver's cooling (a replay of the same observations without
it); a hot swap no longer faults its zone while the bay is still watched, but a swapped
bay that goes blind faults at once; and losing a bay's or a zone's sensors for good still
faults the zone, on the drive sigma or on the blind air clock, with its channels held and
then raised. The sigma floor
(``mpc.step`` 4b) keeps the reach of a zone with a lost group at or above ``prev``:
without it the DAS MPC, which had followed a measured warming that the blind estimate
does not show, lowered the fans on the tick of the loss.

The simulator runs use the ``basic`` preset with each sensor type's noise (the
DAS golden physics); the ``rich`` sweeps (nightly) draw the placements, so the example's
two redundant proximal pairs sit several degC apart there -- the case item 67 was about,
now carried as a placement offset per member rather than as one fused node.
"""

from __future__ import annotations

import dataclasses
import json
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
    ESTIMATOR_DEFAULTS,
    SIGMA_FLOOR_DEFAULTS,
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
    blind: Mapping[str, Any] | None = None,
    occupancy: Mapping[str, str] | None = None,
    bay_flags: Mapping[str, Mapping[str, Any]] | None = None,
    drop_estimates: tuple[str, ...] = (),
    uninitialised: tuple[str, ...] = (),
) -> estimator.EstimatorUpdate:
    """A real first-tick estimator update of the fixture, with sigmas, occupancy, the blind
    air clock, the per-bay flags and zone state overridden (the rule reads nothing else)."""
    temps = {k: v for k, v in default_temps(cfg).items() if v is not None}
    up = estimator.update(None, cfg, temps=temps, u=dict.fromkeys(cfg.channels, 0.5), ts=0.0)
    est = {b: dict(e) for b, e in up.estimates.items() if b not in drop_estimates}
    for bay, value in (sigma or {}).items():
        est[bay]["sigma"] = value
    zones_out = {z: dict(info) for z, info in up.zones.items()}
    for zone, value in (air or {}).items():
        zones_out[zone]["sigma_air_c"] = value
    for zone, value in (blind or {}).items():
        zones_out[zone]["air_blind_s"] = value
    for zone in uninitialised:
        zones_out[zone] = {"initialised": False}
    bays = {b: dict(info) for b, info in up.bays.items()}
    for bay, occ in (occupancy or {}).items():
        bays[bay]["occupancy"] = occ
    for bay, flags in (bay_flags or {}).items():
        bays[bay].update(flags)
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
    assert v["zc"].reasons == ("sigma:zone_air=none>2", "sigma:zone_air_blind=none>900")


def test_a_zone_blind_on_air_for_too_long_is_not_trusted():
    """Item 70: the air sigma barely grows, so the blind time decides."""
    cfg = lcfg()
    window = cfg.estimator.air_blind_fault_s
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    at = fixture_update(cfg, blind={"za": window})
    assert trusted(zones.evaluate(gate, "ok", cfg, at)) == {"za", "zb", "zc"}
    past = fixture_update(cfg, blind={"za": math.nextafter(window, math.inf)})
    v = zones.evaluate(gate, "ok", cfg, past)
    assert trusted(v) == {"zb", "zc"}
    assert v["za"].reasons == ("sigma:zone_air_blind=900.000>900",)


@pytest.mark.parametrize("bad", [None, math.nan, "600", True], ids=repr)
def test_a_blind_time_that_is_not_a_number_is_not_trusted(bad):
    cfg = lcfg()
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    v = zones.evaluate(gate, "ok", cfg, fixture_update(cfg, blind={"za": bad}))
    assert trusted(v) == {"zb", "zc"}


def test_air_blind_fault_s_is_a_config_key():
    gate_cfg = lcfg()
    tight = with_rule(gate_cfg, "sigma", air_blind_fault_s=10.0)
    gate = gate_for(tight, das_obs(tight, 0.0))
    up = fixture_update(tight, blind={"za": 11.0})
    assert trusted(zones.evaluate(gate, "ok", tight, up)) == {"zb", "zc"}
    wide = with_rule(gate_cfg, "sigma", air_blind_fault_s=10_000.0)
    assert trusted(zones.evaluate(gate, "ok", wide, up)) == {"za", "zb", "zc"}


def test_a_settling_observed_bay_carries_no_sigma_check():
    """Item 69: the estimator widened that bay on purpose while it was still watching it."""
    cfg = lcfg()
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    wide = {"a2": 9.0}
    faulted = fixture_update(cfg, sigma=wide)
    assert trusted(zones.evaluate(gate, "ok", cfg, faulted)) == {"zb", "zc"}
    settling = fixture_update(
        cfg, sigma=wide, bay_flags={"a2": {"settling": True, "observed": True}}
    )
    assert trusted(zones.evaluate(gate, "ok", cfg, settling)) == {"za", "zb", "zc"}


@pytest.mark.parametrize(
    "flags",
    [{"settling": True, "observed": False}, {"settling": False, "observed": True}],
    ids=["blind", "settled"],
)
def test_a_bay_that_is_not_both_settling_and_observed_still_faults(flags):
    cfg = lcfg()
    gate = gate_for(cfg, das_obs(cfg, 0.0))
    up = fixture_update(cfg, sigma={"a2": 9.0}, bay_flags={"a2": flags})
    v = zones.evaluate(gate, "ok", cfg, up)
    assert trusted(v) == {"zb", "zc"}
    assert v["za"].reasons == ("sigma:bay:a2=9.000>4",)


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
    floor = ESTIMATOR_DEFAULTS["sigma_uncalibrated_c"]
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


def floor_cfg(hold_s: float = 4.0, growth_c: float = 50.0, per_min: float = 0.6) -> MpcConfig:
    """:func:`lcfg` under ``sigma`` with the soft floor's keys (``dt`` 1 s: ``per_min`` 0.6
    lowers the floor by 0.01 per tick)."""
    return das_cfg(
        setpoints={},
        zones={
            "trust_rule": "sigma",
            "sigma_floor_hold_s": hold_s,
            "sigma_floor_growth_c": growth_c,
            "sigma_floor_release_per_min": per_min,
        },
    )


def test_the_soft_floor_holds_the_command_before_the_loss_then_releases_it():
    """``prox_a2`` is bay a2's only proximal sensor: under ``sigma`` zone za stays trusted.
    From the tick of the loss the channels of its reach (fa1, fa2 and, coupled, fb1) stay at
    the command of the tick before the loss even though the solver asks for less, for
    ``sigma_floor_hold_s``; then the floor falls by ``sigma_floor_release_per_min * dt / 60``
    per tick. fc1 keeps descending throughout. The floor closes when the sensor returns, a
    lost redundant member (``prox_a1``, ``prox_a1b`` still there) floors nothing, and a new
    loss opens a new hold at the command of its own previous tick."""
    cfg = floor_cfg()
    reach = set(cfg.zone_layout.reach["za"])
    assert reach == {"fa1", "fa2", "fb1"}

    def temps_at(i: int) -> dict[str, float | None]:
        if 5 <= i < 15:
            return lost(cfg, "prox_a2")
        if 15 <= i < 20:
            return lost(cfg, "prox_a1")
        if 25 <= i < 28:
            return lost(cfg, "prox_a2")
        return default_temps(cfg)

    cmds = run_descending(cfg, temps_at, 30)
    for i, cmd in enumerate(cmds[1:], start=1):
        d = cmd.diagnostics
        assert d["zones_in_fault"] == [], i
        prev = cmds[i - 1].pwm
        floored = set(d["sigma_floor_channels"])
        episode = d["sigma_floor"].get("za")
        if 5 <= i < 15 or 25 <= i < 28:
            opened = 5 if i < 15 else 25
            assert floored == reach, i
            assert episode["lost"] == ["bay:a2"], i
            held = i - opened
            assert episode["held_s"] == held * cfg.dt
            level = cmds[opened - 1].pwm["fa1"]
            if held < 4:
                assert episode["phase"] == zones.FLOOR_HOLD, i
                want = level
            else:
                assert episode["phase"] == zones.FLOOR_RELEASE, i
                want = level - 0.01 * (held - 3)
            for ch in reach:
                assert cmd.pwm[ch] == pytest.approx(max(want, prev[ch] - 0.05), abs=1e-12), (i, ch)
            assert cmd.pwm["fc1"] < prev["fc1"] or prev["fc1"] <= cfg.pwm_min, i
        else:
            assert floored == set() and episode is None, i
            for ch in cfg.channels:
                assert cmd.pwm[ch] < prev[ch] or prev[ch] <= cfg.pwm_min, (i, ch)


def test_the_soft_floor_hold_ends_once_every_lost_groups_sigma_has_grown():
    """The hold ends on the first tick on which the sigma of every lost group has grown by
    ``sigma_floor_growth_c`` since the loss, before ``sigma_floor_hold_s``; a further group
    lost during the episode opens a new hold at the higher of the floor and ``prev``, with a
    new sigma reference; the air group's growth reads the zone's air sigma."""
    cfg = floor_cfg(hold_s=1000.0, growth_c=0.5, per_min=6.0)
    reach = cfg.zone_layout.reach["za"]
    prev = dict.fromkeys(cfg.channels, 0.6)

    def update(a2: float, air: float) -> estimator.EstimatorUpdate:
        return estimator.EstimatorUpdate(
            estimates={"a2": {"sigma": a2}},
            memory={},
            zones={"za": {"initialised": True, "sigma_air_c": air}},
            bays={},
            summary={},
        )

    def advance(memory, lost_now, upd, prev_now, eligible=("za",)):
        return zones.advance_sigma_floor(
            memory, cfg, lost={"za": lost_now}, estimator=upd, prev=prev_now, eligible=eligible
        )

    soft = advance(None, ("bay:a2",), update(1.5, 0.2), prev)
    assert soft.zones["za"]["phase"] == zones.FLOOR_HOLD
    assert soft.floor == dict.fromkeys(reach, 0.6)
    soft = advance(soft.memory, ("bay:a2",), update(1.99, 0.2), dict.fromkeys(cfg.channels, 0.2))
    assert soft.zones["za"]["phase"] == zones.FLOOR_HOLD and soft.floor["fa1"] == 0.6
    soft = advance(soft.memory, ("bay:a2",), update(2.0, 0.2), prev)
    assert soft.zones["za"]["phase"] == zones.FLOOR_RELEASE
    assert soft.floor["fa1"] == pytest.approx(0.5)  # 6.0 per minute at dt = 1 s
    assert soft.zones["za"]["sigma_growth_c"] == {"bay:a2": pytest.approx(0.5)}
    # the zone air group is lost as well: a new hold at max(floor, prev), new references
    soft = advance(
        soft.memory, ("zone_air", "bay:a2"), update(2.1, 0.2), dict.fromkeys(cfg.channels, 0.45)
    )
    assert soft.zones["za"]["phase"] == zones.FLOOR_HOLD and soft.zones["za"]["held_s"] == 0.0
    assert soft.floor["fa1"] == pytest.approx(0.5)
    soft = advance(soft.memory, ("zone_air", "bay:a2"), update(2.7, 0.3), prev)
    assert soft.zones["za"]["phase"] == zones.FLOOR_HOLD, "the air sigma has not grown"
    # an estimator fault keeps the episode as it is; a zone in fault gets no floor
    kept = advance(soft.memory, ("zone_air", "bay:a2"), None, prev)
    assert kept.memory == soft.memory and kept.floor == soft.floor
    assert advance(soft.memory, ("zone_air", "bay:a2"), update(2.7, 0.3), prev, ()).floor == {}
    soft = advance(soft.memory, ("zone_air", "bay:a2"), update(2.7, 0.8), prev)
    assert soft.zones["za"]["phase"] == zones.FLOOR_RELEASE
    # the air sensor returns: the bay keeps the episode going; the floor falls to pwm_min
    for _ in range(10):
        soft = advance(soft.memory, ("bay:a2",), update(2.7, 0.3), prev)
    assert soft.zones["za"]["phase"] == zones.FLOOR_RELEASED and soft.floor == {}
    assert json.loads(json.dumps(soft.memory)) == soft.memory
    # every group held again: the episode closes
    assert advance(soft.memory, (), update(1.5, 0.2), prev).memory == {}


@pytest.mark.parametrize(
    "stored",
    [
        "garbage",
        {"lost": "bay:a2", "ticks": 0, "phase": "hold", "sigma0": {}, "level": {}},
        {"lost": ["bay:a2"], "ticks": -1, "phase": "hold", "sigma0": {}, "level": {}},
        {"lost": ["bay:a2"], "ticks": 0, "phase": "later", "sigma0": {}, "level": {}},
        {"lost": ["bay:a2"], "ticks": 0, "phase": "hold", "sigma0": {}, "level": {"fa1": 0.9}},
        {
            "lost": ["bay:a2"],
            "ticks": 0,
            "phase": "hold",
            "sigma0": {},
            "level": {"fa1": math.nan, "fa2": 0.9, "fb1": 0.9},
        },
    ],
)
def test_a_malformed_soft_floor_episode_opens_a_new_hold(stored):
    cfg = floor_cfg()
    upd = estimator.EstimatorUpdate(
        estimates={"a2": {"sigma": 3.0}}, memory={}, zones={}, bays={}, summary={}
    )
    soft = zones.advance_sigma_floor(
        {"za": stored},
        cfg,
        lost={"za": ("bay:a2",)},
        estimator=upd,
        prev=dict.fromkeys(cfg.channels, 0.4),
        eligible=("za",),
    )
    assert soft.zones["za"]["phase"] == zones.FLOOR_HOLD and soft.zones["za"]["held_s"] == 0.0
    assert soft.floor == dict.fromkeys(cfg.zone_layout.reach["za"], 0.4)


def test_soft_floor_config_keys():
    cfg = lcfg()
    assert cfg.zones.sigma_floor_hold_s == SIGMA_FLOOR_DEFAULTS["sigma_floor_hold_s"] == 1800.0
    assert cfg.zones.sigma_floor_growth_c == SIGMA_FLOOR_DEFAULTS["sigma_floor_growth_c"] == 1.0
    assert (
        cfg.zones.sigma_floor_release_per_min
        == SIGMA_FLOOR_DEFAULTS["sigma_floor_release_per_min"]
        == 0.0025
    )
    assert MpcConfig.from_mapping(floor_cfg(7.0, 0.3, 0.1).to_dict()).zones == ZonePolicy(
        trust_rule="sigma",
        sigma_floor_hold_s=7.0,
        sigma_floor_growth_c=0.3,
        sigma_floor_release_per_min=0.1,
    )
    assert floor_cfg(hold_s=0.0).zones.sigma_floor_hold_s == 0.0
    for kw, match in (
        ({"hold_s": -1.0}, "sigma_floor_hold_s must be >= 0"),
        ({"growth_c": 0.0}, "sigma_floor_growth_c must be > 0"),
        ({"per_min": 0.0}, "sigma_floor_release_per_min must be > 0"),
        ({"per_min": math.inf}, "sigma_floor_release_per_min must be finite"),
    ):
        with pytest.raises(ConfigError, match=match):
            floor_cfg(**kw)


def test_strict_has_no_sigma_floor():
    cfg = das_cfg(setpoints={}, zones={"trust_rule": "strict"})
    cmds = run_descending(cfg, lambda i: lost(cfg, "air_c") if i >= 3 else default_temps(cfg), 6)
    assert all(c.diagnostics["sigma_floor_channels"] == [] for c in cmds)
    assert all(c.diagnostics["sigma_floor"] == {} for c in cmds)


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
#: The example's redundant proximal sensors: a second sensor on b03 and on b10.
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
    strict = sim_run(
        example_cfg("strict", solver), ticks, preset=preset, seed=seed, dropout=DROPOUT
    )
    sigma = sim_run(example_cfg("sigma", solver), ticks, preset=preset, seed=seed, dropout=DROPOUT)
    strict_faults, sigma_faults = zone_faults(strict), zone_faults(sigma)
    assert strict_faults[0] > 50, strict_faults  # the dropouts do fault zones under strict
    # Under sigma none at all, the example's two redundant proximal pairs included (items
    # 67 and 69: their placements are an offset the filter carries, and a sensor missing on
    # the estimator's first tick seeds its bay instead of tripping the fast-swap rule).
    assert sigma_faults == (0, 0), sigma_faults
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


def assert_redundant_pairs_do_not_fault_a_healthy_zone(ticks: int, seed: int, solver: str) -> None:
    """Item 67: on ``rich`` the two proximal sensors of b03 and of b10 sit at different
    drawn placements and read degrees apart. Their disagreement is a placement offset the
    filter carries, not a swap, so the bays' sigmas stay at the uncalibrated floor and no
    zone faults on a healthy plant."""
    run = sim_run(example_cfg("sigma", solver), ticks, preset="rich", seed=seed)
    assert zone_faults(run) == (0, 0)
    for bay in ("b03", "b10"):
        sigma = [
            r.cmd.diagnostics["estimates"][bay]["sigma_c"]
            for r in run.records
            if bay in r.cmd.diagnostics["estimates"]
        ]
        assert max(sigma) < ESTIMATOR_DEFAULTS["sigma_uncalibrated_c"] + 0.1, (bay, max(sigma))
    offsets = run.records[-1].cmd.diagnostics["bays"]
    assert set(offsets["b03"]["offsets_c"]) == {"prox_b03b"}
    assert set(offsets["b10"]["offsets_c"]) == {"prox_b10b"}
    assert run.violations() == 0


@pytest.mark.parametrize("solver", ["pi", "mpc"])
def test_the_redundant_pairs_do_not_fault_a_healthy_zone_on_rich(solver):
    assert_redundant_pairs_do_not_fault_a_healthy_zone(PR_TICKS, 1, solver)


@pytest.mark.nightly
@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize("seed", range(6))
def test_the_redundant_pairs_do_not_fault_a_healthy_zone_sweep(seed, solver):
    assert_redundant_pairs_do_not_fault_a_healthy_zone(LONG_TICKS, seed, solver)


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


def no_sigma_floor(*_args: Any, **_kwargs: Any) -> zones.SigmaFloor:
    """``zones.advance_sigma_floor`` replaced by no floor at all (the reference runs)."""
    return zones.SigmaFloor(floor={}, memory={}, zones={})


@dataclasses.dataclass(frozen=True)
class FloorCase:
    """One bay's only proximal sensor lost from ``LOSS_S`` on, closed loop, three runs on
    the same seed: with the sensor, without it and no floor, without it and the soft floor."""

    cfg: MpcConfig
    bay: str
    zone: str
    start: int
    healthy: DasRun
    bare: DasRun
    soft: DasRun

    def margin_loss(self, run: DasRun) -> list[float]:
        """The bay's true margin lost against the run with the sensor, per tick from the loss."""
        h, r = self.healthy.array("margin_c", self.bay), run.array("margin_c", self.bay)
        return [float(x) for x in (h - r)[self.start :]]

    def uncovered_loss(self, run: DasRun) -> float:
        """The largest margin loss beyond ``k_sigma`` times the sigma growth since the loss."""
        sigma = [rec.cmd.diagnostics["estimates"][self.bay]["sigma_c"] for rec in run.records]
        k, s0 = self.cfg.estimator.k_sigma, sigma[self.start - 1]
        return max(
            loss - k * max(0.0, s - s0)
            for loss, s in zip(self.margin_loss(run), sigma[self.start :], strict=True)
        )

    def fault_at(self, run: DasRun) -> int:
        """The first tick the zone is in fault after the loss (the run's length if never)."""
        return next(
            (
                i
                for i, rec in enumerate(run.records)
                if i >= self.start and self.zone in rec.cmd.diagnostics["zones_in_fault"]
            ),
            len(run.records),
        )

    def mean_noise_db(self, run: DasRun, end: int | None = None) -> float:
        """Mean modelled noise from the loss to ``end`` (default: the zone's first fault)."""
        stop = self.fault_at(run) if end is None else end
        seq = run.series["noise_db"][self.start : stop]
        return sum(seq) / len(seq)

    def mean_pwm(self, run: DasRun, lo: int, hi: int) -> float:
        """Mean command over the zone's channels on ticks ``[lo, hi)``."""
        chans = self.cfg.zone_layout.zone_channels[self.zone]
        vals = [run.records[i].cmd.pwm[ch] for i in range(lo, hi) for ch in chans]
        return sum(vals) / len(vals)


def floor_case(
    solver: str,
    ticks: int,
    sensor: str = "prox_b02",
    preset: str = "basic",
    seed: int = 1,
    reference_ticks: int | None = None,
) -> FloorCase:
    """The three runs; ``reference_ticks`` shortens the runs with the sensor and without a
    floor (the PR test only compares the first minutes after the loss) and then runs the
    soft floor without the per-tick invariant checks, which the nightly sweep makes."""
    cfg = example_cfg("sigma", solver)
    healthy_hook = None
    lost_hook = without((sensor,), since_s=LOSS_S)
    kw: dict[str, Any] = {"preset": preset, "seed": seed}
    short = ticks if reference_ticks is None else reference_ticks
    healthy = sim_run(cfg, short, hook=healthy_hook, **kw)
    with mock.patch.object(zones, "advance_sigma_floor", no_sigma_floor):
        bare = sim_run(cfg, short, hook=lost_hook, **kw)
    controller = step if reference_ticks is not None else checked_step
    soft = sim_run(cfg, ticks, hook=lost_hook, controller=controller, **kw)
    return FloorCase(
        cfg=cfg,
        bay=cfg.sensors[sensor].bay,
        zone=cfg.sensors[sensor].zone,
        start=next(i for i, t in enumerate(healthy.series["ts"]) if t >= LOSS_S),
        healthy=healthy,
        bare=bare,
        soft=soft,
    )


#: Documented bounds of the soft sigma floor (PROJECT.md section 3 per-zone trust).
#: The true margin a bay loses against the run with its sensor stays within ``k_sigma``
#: times its sigma growth since the loss plus this much, degC (measured 0.00; without a
#: floor 0.06, in the first minutes after the loss).
UNCOVERED_LOSS_C = 0.05
#: ...and within this much altogether over 65 minutes of loss, degC (measured up to 1.03
#: on basic seeds 1-5; 2.39 without a floor). Not a bound for a longer loss: once the floor
#: has released, the loss converges to that of the run without a floor.
SOFT_FLOOR_LOSS_C = 1.25
#: ...and the mean modelled noise until the zone's first fault within this much of the run
#: without a floor, dB (the hard floor: 32.5 against 27.1 dB).
SOFT_FLOOR_NOISE_DB = 1.5


#: Ticks of the PR soft-floor test: the hold ends on the sigma growth about 15 minutes
#: after the loss, and the reference runs cover the first 5 minutes after it.
FLOOR_PR_TICKS = 320
FLOOR_REFERENCE_TICKS = int(LOSS_S / 5.0) + 60


def test_the_soft_sigma_floor_on_the_das_mpc():
    """The DAS MPC loses b02's only proximal sensor after 10 minutes. Without a floor it
    reproduces the case the floor exists for: it had followed a warming the blind estimate
    does not show, lowers z0's command by about 0.04 on the tick of the loss and plans
    about 21 % less z0 airflow than the run with the sensor within the first minutes. With
    the soft floor nothing goes below the command before the loss while the floor holds,
    the bay's sigma growth ends the hold before ``sigma_floor_hold_s``, the release lowers
    the floor by at most its rate, and no drive exceeds its limit. The margin and noise
    bounds over 65 minutes are the nightly sweep's."""
    case = floor_case("mpc", FLOOR_PR_TICKS, reference_ticks=FLOOR_REFERENCE_TICKS)
    cfg, start = case.cfg, case.start
    reach = cfg.zone_layout.reach[case.zone]
    before = case.soft.records[start - 1].cmd.pwm
    assert before == case.bare.records[start - 1].cmd.pwm == case.healthy.records[start - 1].cmd.pwm
    # without a floor
    drop = before["xt1"] - case.bare.records[start].cmd.pwm["xt1"]
    assert drop > 0.03, drop
    ratio = min(
        zone_airflow(cfg, case.zone, b.cmd.pwm) / zone_airflow(cfg, case.zone, h.cmd.pwm)
        for b, h in zip(case.bare.records[start:], case.healthy.records[start:], strict=True)
    )
    assert ratio < 0.82, ratio
    # the soft floor
    step_down = cfg.zones.sigma_floor_release_per_min * cfg.dt / 60.0
    phases, floors = [], []
    for rec in case.soft.records[start:]:
        d = rec.cmd.diagnostics
        episode = d["sigma_floor"][case.zone]
        phases.append(episode["phase"])
        floors.append(episode["floor"])
        assert episode["lost"] == [f"bay:{case.bay}"]
        assert set(d["sigma_floor_channels"]) <= set(reach)
        for ch, level in episode["floor"].items():
            assert level >= before[ch] - step_down * len(phases) - 1e-9
            if d["policy_by_channel"][ch] == "solver":
                assert (
                    rec.cmd.pwm[ch]
                    >= min(level, rec.cmd.diagnostics["prev_pwm"][ch] - cfg.d_pwm_max) - 1e-12
                )
        if episode["phase"] == zones.FLOOR_HOLD:
            assert episode["floor"] == {ch: before[ch] for ch in reach}
            assert episode["sigma_growth_c"][f"bay:{case.bay}"] < cfg.zones.sigma_floor_growth_c
    assert phases[0] == zones.FLOOR_HOLD and zones.FLOOR_RELEASE in phases
    released = phases.index(zones.FLOOR_RELEASE)
    assert released * cfg.dt < cfg.zones.sigma_floor_hold_s, "the sigma growth ended the hold"
    for old, new in zip(floors[released - 1 :], floors[released:], strict=False):
        for ch in new:
            assert old[ch] - new[ch] == pytest.approx(step_down)
    assert case.soft.violations() == 0


@pytest.mark.nightly
@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize(
    ("preset", "seed"), [("basic", s) for s in range(1, 6)] + [("rich", s) for s in range(3)]
)
@pytest.mark.parametrize("sensor", ["prox_b02", "prox_b13"])
def test_the_soft_sigma_floor_sweep(preset, seed, solver, sensor):
    """65 minutes of loss at the default keys. Every drive within its limit; the margin
    lost against the run with the sensor is covered by the k * sigma growth and within
    ``SOFT_FLOOR_LOSS_C``; no ratchet: the zone's mean command over the 5 minutes before
    its first fault (or the end) has not risen from the 5 minutes before the loss by more
    than 0.05 beyond the rise without a floor (if any); the mean noise until that fault is within
    ``SOFT_FLOOR_NOISE_DB`` of the run without a floor until its own fault. Without a floor
    the DAS MPC on ``basic`` loses more than 2 degC of b02's margin (the 2.3 degC case)."""
    case = floor_case(solver, LONG_TICKS, sensor, preset, seed)
    assert case.soft.violations() == 0 and case.bare.violations() == 0
    assert case.uncovered_loss(case.soft) <= UNCOVERED_LOSS_C
    assert max(case.margin_loss(case.soft)) <= SOFT_FLOOR_LOSS_C

    def rise(run: DasRun) -> float:
        fault = case.fault_at(run)
        before = case.mean_pwm(run, case.start - 60, case.start)
        return case.mean_pwm(run, max(case.start, fault - 60), fault) - before

    assert rise(case.soft) <= max(0.0, rise(case.bare)) + 0.05, (rise(case.soft), rise(case.bare))
    assert case.mean_noise_db(case.soft) <= case.mean_noise_db(case.bare) + SOFT_FLOOR_NOISE_DB
    if solver == "mpc" and preset == "basic" and sensor == "prox_b02":
        assert max(case.margin_loss(case.bare)) > 2.0


@pytest.mark.nightly
@pytest.mark.parametrize(
    ("preset", "seed", "sensor"),
    [("basic", 1, "prox_b02"), ("basic", 4, "prox_b13"), ("rich", 0, "prox_b02")],
)
def test_the_soft_sigma_floor_until_it_has_released(preset, seed, sensor):
    """The DAS MPC with the sensor lost for good, run until the documented end of the
    floor (``sigma_floor_hold_s + 60 * (pwm_max - pwm_min) / sigma_floor_release_per_min``
    after the loss). The floor is released by then; every drive stays within its limit;
    the true margin lost against the run with the sensor stays within ``k_sigma`` times the
    sigma growth plus ``UNCOVERED_LOSS_C`` on every tick. ``SOFT_FLOOR_LOSS_C`` bounds the
    total loss only over the sweep's 65 minutes: once the floor has released, the loss
    converges to that of the run without a floor (2.4 degC on b02, measured 2.42 against
    2.40), so in total it stays within the larger of the two plus ``UNCOVERED_LOSS_C``."""
    cfg = example_cfg("sigma", "mpc")
    policy = cfg.zones
    end_s = policy.sigma_floor_hold_s + 60.0 * (cfg.pwm_max - cfg.pwm_min) / (
        policy.sigma_floor_release_per_min
    )
    ticks = int((LOSS_S + end_s) / cfg.dt) + 2
    case = floor_case("mpc", ticks, sensor, preset, seed)
    phases = [
        rec.cmd.diagnostics["sigma_floor"].get(case.zone, {}).get("phase")
        for rec in case.soft.records[case.start :]
    ]
    assert zones.FLOOR_RELEASED in phases
    assert case.soft.violations() == 0
    assert case.uncovered_loss(case.soft) <= UNCOVERED_LOSS_C
    bare = max(case.margin_loss(case.bare))
    assert max(case.margin_loss(case.soft)) <= max(bare, SOFT_FLOOR_LOSS_C) + UNCOVERED_LOSS_C


@pytest.mark.parametrize(("name", "sigma_cost"), [("prox_b03b", 0.001), ("prox_b03", 0.05)])
def test_a_lost_proximal_member_of_a_redundant_pair_changes_almost_nothing(
    healthy_sigma_run, name, sigma_cost
):
    """b03 keeps its other proximal sensor: sigma and the commands stay as they were.
    Losing the member that carries a placement offset (``prox_b03b``) costs nothing;
    losing the anchor leaves the bay observed through an offset whose own uncertainty the
    sigma now carries, which is a hundredth of a degree, far below ``sigma_fault_c``."""
    cfg, healthy = healthy_sigma_run
    for with_sensor, without_sensor in replay_without(cfg, healthy, name):
        s0 = with_sensor.diagnostics["estimates"]["b03"]["sigma_c"]
        s1 = without_sensor.diagnostics["estimates"]["b03"]["sigma_c"]
        assert abs(s1 - s0) < sigma_cost
        assert without_sensor.diagnostics["zones_in_fault"] == []
        for ch in cfg.channels:
            assert abs(without_sensor.pwm[ch] - with_sensor.pwm[ch]) < 0.005


#: When the sensors of the observability-loss runs go missing for good.
LOST_S = 300.0


def assert_observability_loss_faults(
    cfg: MpcConfig,
    names: tuple[str, ...],
    zone: str,
    ticks: int,
    *,
    why: str = "sigma:bay:",
) -> tuple[DasRun, int]:
    """``names`` missing from ``LOST_S`` on: the zone runs on the solver with rising fans
    until a sigma passes its threshold (or its air node has been blind for
    ``estimator.air_blind_fault_s``), then faults for good; its reach holds and then
    rises to ``fallback_pwm``. Returns the run and the first fault tick."""
    run = sim_run(cfg, ticks, hook=without(names, since_s=LOST_S), controller=checked_step)
    at = next(
        (i for i, r in enumerate(run.records) if zone in r.cmd.diagnostics["zones_in_fault"]), None
    )
    assert at is not None, f"{zone} never faulted after losing {names}"
    lost = next(i for i, r in enumerate(run.records) if r.obs.ts >= LOST_S)
    assert at > lost + int(300.0 / cfg.dt), "no graceful period: the zone faulted at once"
    reasons = run.records[at].cmd.diagnostics["zones"][zone]["reasons"]
    assert reasons and all(reason.startswith(why) for reason in reasons), reasons
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
def test_losing_every_sensor_of_a_zone_faults_it_on_the_blind_air_clock(solver):
    """Item 70. The air sigma stays far below ``sigma_air_fault_c`` (every proximal sensor
    reads the air beside its drive, and the inlet and the fan command pin the rest), so
    what decides is ``air_blind_fault_s``: 15 minutes after the loss, just before the
    drives' sigmas reach ``sigma_fault_c`` at about 17 minutes."""
    cfg = example_cfg("sigma", solver)
    names = tuple(n for n in cfg.temps if cfg.sensors[n].zone == "z0")
    assert set(names) >= {"air_z0", "prox_b01", "prox_b02", "prox_b03", "prox_b03b", "prox_b04"}
    run, at = assert_observability_loss_faults(cfg, names, "z0", 300, why="sigma:zone_air_blind=")
    info = run.records[at].cmd.diagnostics["estimator"]["zones"]["z0"]
    assert info["sigma_air_c"] < cfg.estimator.sigma_air_fault_c / 4
    assert info["air_blind_s"] == pytest.approx(cfg.estimator.air_blind_fault_s + cfg.dt)


@pytest.mark.parametrize("solver", ["pi", "mpc"])
def test_losing_every_sensor_of_a_zone_faults_on_sigma_with_the_blind_clock_wide(solver):
    """The same loss with ``air_blind_fault_s`` far away: the drives' sigmas decide, about
    17 minutes after the loss, which is what this rule did before item 70."""
    cfg = example_cfg("sigma", solver, air_blind_fault_s=10_000.0)
    names = tuple(n for n in cfg.temps if cfg.sensors[n].zone == "z0")
    run, at = assert_observability_loss_faults(cfg, names, "z0", 300)
    assert run.records[at].obs.ts - LOST_S > 900.0


def test_a_zone_that_loses_only_its_air_sensor_faults_on_the_blind_clock():
    """The case no sigma ever caught: the bays keep their proximal sensors, so every drive
    sigma stays at the uncalibrated floor and the air sigma below 0.1 degC for ever."""
    cfg = example_cfg("sigma")
    run, at = assert_observability_loss_faults(
        cfg, ("air_z0",), "z0", 300, why="sigma:zone_air_blind="
    )
    every = [
        e["sigma_c"]
        for r in run.records
        for b, e in r.cmd.diagnostics["estimates"].items()
        if r.cmd.diagnostics["bays"][b]["zone"] == "z0"
    ]
    assert max(every) < cfg.estimator.sigma_fault_c / 2, max(every)
    air = [r.cmd.diagnostics["estimator"]["zones"]["z0"]["sigma_air_c"] for r in run.records]
    assert max(air[1:]) < 0.1, max(air[1:])  # air[0] is the prior sqrt(p0_t_air)
    wide = example_cfg("sigma", air_blind_fault_s=10_000.0)
    never = sim_run(wide, 300, hook=without(("air_z0",), since_s=LOST_S))
    assert not any(r.cmd.diagnostics["zones_in_fault"] for r in never.records)


@pytest.mark.nightly
@pytest.mark.parametrize("solver", ["pi", "mpc"])
@pytest.mark.parametrize(("preset", "seed"), [("basic", 2), ("rich", 0), ("rich", 1), ("rich", 2)])
@pytest.mark.parametrize("sensor", ["prox_b02", "prox_b13"])
def test_a_bays_only_sensor_lost_for_good_keeps_every_drive_within_its_limit(
    preset, seed, solver, sensor
):
    """75 minutes with the sensor lost from 10 minutes on, default thresholds."""
    hook = without((sensor,), since_s=600.0)

    run = sim_run(example_cfg("sigma", solver), LONG_TICKS, preset=preset, seed=seed, hook=hook)
    assert run.violations() == 0


def hot_swap_run(cfg: MpcConfig, insert_s: float = 1200.0, **kw: Any) -> DasRun:
    """b06's drive pulled at 300 s and a warm one inserted at ``insert_s``."""
    return sim_run(
        cfg,
        int((insert_s + cfg.estimator.bay_settle_s) / cfg.dt),
        seed=31,
        controller=checked_step,
        heat_schedule={"b06": [(0.0, 1.0)], "b07": [(0.0, 0.5)]},
        bay_schedule=[
            {"t_s": 300.0, "bay": "b06", "action": "remove"},
            {"t_s": insert_s, "bay": "b06", "action": "insert", "temp_c": 45.0},
        ],
        **kw,
    )


def test_a_hot_swap_no_longer_faults_its_zone_under_sigma():
    """Item 69. The estimator widens a swapped bay's sigma on purpose (the fast-swap rule
    on the removal, a 25 degC^2 drive variance on the insert), above ``sigma_fault_c`` for
    a tick. That is the filter following the swap while it is still watching the bay, not
    an observability loss, so the sigma check is suspended while the bay is ``settling``
    and ``observed``: no zone fault, while the margin the widened sigma carries still
    raises the fans. ``tests/test_hotswap.py`` runs the same swap under ``strict``."""
    cfg = example_cfg("sigma")
    insert_s = 1200.0
    run = hot_swap_run(cfg, insert_s)
    assert zone_faults(run) == (0, 0)
    for rec in run.records:
        d = rec.cmd.diagnostics
        assert d["zones_in_fault"] == []
        assert all(p == "solver" for p in d["policy_by_channel"].values())
    peak = max(
        r.cmd.diagnostics["estimates"]["b06"]["sigma_c"]
        for r in run.records
        if "b06" in r.cmd.diagnostics["estimates"]
    )
    assert peak > cfg.estimator.sigma_fault_c  # the widening is still there
    assert run.violations() == 0
    at = next(i for i, t in enumerate(run.series["ts"]) if t >= insert_s)
    rise = max(
        max(run.series["pwm_cmd"][ch][at:]) - run.series["pwm_cmd"][ch][at]
        for ch in cfg.topology.zones["z1"].channels
    )
    assert rise > 0.05


#: A loose probe steps its bay's proximal sensor this far, once every FLAP_EVERY ticks.
FLAP_STEP_C = 3.5
FLAP_EVERY = 48


def flapping(name: str, step_c: float, every: int) -> ObsHook:
    """``name`` reads ``step_c`` high on every ``every``-th tick and normally in between: a
    loose probe, an intermittent 1-Wire contact, a drive repeatedly reseated. The step is
    inside the gate's slew limit (0.7 degC/s against ``dT_max_c_per_s: 1.0``), so every
    value is gate-trusted and reaches the estimator, where it trips the fast-swap rule."""

    def hook(i: int, obs: PlantObservation) -> PlantObservation:
        if i % every or obs.temps.get(name) is None:
            return obs
        temps = dict(obs.temps)
        temps[name] = float(temps[name]) + step_c  # type: ignore[arg-type]
        return dataclasses.replace(obs, temps=temps)

    return hook


@pytest.mark.parametrize("solver", ["pi", "mpc"])
def test_a_flapping_proximal_sensor_still_faults_its_zone(solver):
    """Item 69's exemption is bounded in wall-clock, not by the sigma coming back. A bay's
    sigma falls back within a tick or two of every jump, so a rule that re-armed on a
    recovered sigma renewed the window at every cadence slower than one jump per tick and
    the zone never faulted again. The windows of one bay may now total ``bay_settle_max_s``
    of suspended check; after that the zone faults on every tick the sigma is over, as it
    did before item 69."""
    cfg = example_cfg("sigma", solver)
    hook = flapping("prox_b06", FLAP_STEP_C, FLAP_EVERY)
    run = sim_run(cfg, LONG_TICKS, hook=hook, controller=checked_step)
    episodes, fault_ticks = zone_faults(run)
    assert episodes > 0, "a flapping sensor kept its zone exempt for the whole run"
    bound = cfg.estimator.bay_settle_max_s + cfg.estimator.bay_settle_s
    first = next(r.obs.ts for r in run.records if r.cmd.diagnostics["zones_in_fault"])
    assert first <= bound + cfg.dt, first  # the budget plus the window it ran out inside
    exempt = sum(1 for r in run.records if r.cmd.diagnostics["bays"]["b06"]["settling"])
    assert exempt * cfg.dt <= bound, exempt
    assert all(
        r.cmd.diagnostics["zones_in_fault"] in ([], ["z1"]) for r in run.records
    )  # only b06's zone
    assert run.violations() == 0
    # ``bay_settle_max_s: 0`` is the rule before item 69: every jump faults the zone
    none = sim_run(
        example_cfg("sigma", solver, bay_settle_s=0.0, bay_settle_max_s=0.0),
        LONG_TICKS,
        hook=hook,
    )
    assert zone_faults(none)[0] > episodes > 0


def test_a_swapped_bay_that_goes_blind_faults_at_once():
    """The exemption needs the bay observed: lose its sensor right after the insert and the
    widened sigma faults the zone on that tick, sooner than it would have before item 69."""
    cfg = example_cfg("sigma")
    insert_s = 1200.0
    blind_from = insert_s + 2 * cfg.dt
    run = hot_swap_run(cfg, insert_s, hook=without(("prox_b06",), since_s=blind_from))
    at = next(
        (i for i, r in enumerate(run.records) if "z1" in r.cmd.diagnostics["zones_in_fault"]), None
    )
    assert at is not None, "a blind bay with a wide sigma did not fault its zone"
    assert run.records[at].obs.ts <= blind_from + cfg.dt
    reasons = run.records[at].cmd.diagnostics["zones"]["z1"]["reasons"]
    assert all(reason.startswith("sigma:bay:b06=") for reason in reasons), reasons
    assert run.violations() == 0
