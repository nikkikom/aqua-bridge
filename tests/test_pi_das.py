"""PI-like DAS solver: margin-deficit form of ``PiSolver`` (plan section 4) and its
integration in ``mpc.step`` (plan section 6: ``SolverRequest.estimates``,
``zone_trust``, ``fixed_channels``).

The small zoned config of :mod:`das_fixtures` without setpoints::

    za  fa1, fa2   bays a1 (hdd, auto), a2 (ssd_sata, occupied)   coupled_to zb
    zb  fb1        bay  b1 (default class hdd, auto)              coupled_to za
    zc  fc1        bay  c1 (occupied: false)                      no coupling
"""

from __future__ import annotations

import dataclasses
import json

import pytest

from aqua_bridge.control import estimates as est
from aqua_bridge.control.solver_pi import (
    PiSolver,
    SolverRequest,
    SolverResult,
    channel_errors,
    channel_margin_errors,
)
from aqua_bridge.model import Mode, MpcConfig, MpcState
from das_fixtures import PROX_C, SP, das_cfg, das_mapping, das_obs, default_temps
from invariants import TOL, checked_step

HDD_E = est.prior_drive_temp(PROX_C, SP) + 3.0 - 42.0  # default temps, hdd bay
SSD_E = est.prior_drive_temp(PROX_C, SP) + 3.0 - 52.0


@pytest.fixture
def lcfg() -> MpcConfig:
    return das_cfg(setpoints={})


def prox_for(t_drive: float, t_air: float = SP) -> float:
    """Proximal reading whose prior-map estimate is ``t_drive``."""
    return (1.0 - est.PRIOR_BETA) * t_drive + est.PRIOR_BETA * t_air + est.PRIOR_OFFSET_C


def block(cfg: MpcConfig, **overrides: float) -> dict:
    temps = {k: v for k, v in default_temps(cfg).items() if v is not None}
    temps.update(overrides)
    return est.prior_estimates(cfg, temps)


ALL_TRUSTED = {"za": True, "zb": True, "zc": True}


def request(cfg: MpcConfig, prev: float = 0.5, **kw) -> SolverRequest:
    kw.setdefault("estimates", block(cfg))
    kw.setdefault("zone_trust", dict(ALL_TRUSTED))
    return SolverRequest(temps={}, prev_pwm=dict.fromkeys(cfg.channels, prev), **kw)


# ---------------------------------------------------------------------------
# mode selection and the served zones
# ---------------------------------------------------------------------------


def test_limit_regulation_needs_topology_and_no_setpoints(cfg, lcfg):
    assert lcfg.regulates_drive_limits
    assert not das_cfg().regulates_drive_limits  # zoned but still on setpoints
    assert not cfg.regulates_drive_limits  # legacy


def test_served_zones_are_own_plus_declared_coupling(lcfg, cfg):
    served = lcfg.zone_layout.served
    assert served == {"fa1": ("za", "zb"), "fa2": ("za", "zb"), "fb1": ("za", "zb"), "fc1": ("zc",)}
    # independent of fault_coupling: the air physics does not change with the fault policy
    none = das_cfg(setpoints={}, zones={"fault_coupling": "none"})
    assert none.zone_layout.served == served
    assert cfg.zone_layout.served == {ch: ("all",) for ch in cfg.channels}


# ---------------------------------------------------------------------------
# channel_margin_errors
# ---------------------------------------------------------------------------


def test_margin_deficit_is_the_worst_served_bay(lcfg):
    errors, worst = channel_margin_errors(lcfg, block(lcfg), ALL_TRUSTED)
    assert errors["fa1"] == pytest.approx(HDD_E) and worst["fa1"] == "a1"
    assert errors["fb1"] == pytest.approx(HDD_E) and worst["fb1"] == "a1"
    # c1 is empty: nothing to regulate, hold
    assert errors["fc1"] == 0.0 and worst["fc1"] is None
    # a hot SSD in a2 takes over fa1 once its deficit is larger
    hot = block(lcfg, prox_a2=prox_for(60.0))
    errors, worst = channel_margin_errors(lcfg, hot, ALL_TRUSTED)
    assert worst["fa1"] == "a2" and errors["fa1"] == pytest.approx(60.0 + 3.0 - 52.0)
    # ... and so does a hot drive next door (b1 in the coupled zone zb)
    hot = block(lcfg, prox_b1=prox_for(48.0))
    errors, worst = channel_margin_errors(lcfg, hot, ALL_TRUSTED)
    assert worst["fa2"] == "b1" and errors["fa2"] == pytest.approx(48.0 + 3.0 - 42.0)


def test_formula_counts_k_sigma_as_the_plan_writes_it(lcfg):
    """e = t + k*sigma - soft with soft = limit - comfort - k*sigma (plan section 4)."""
    b = block(lcfg)["a1"]
    errors, _ = channel_margin_errors(lcfg, {"a1": b, "a2": b, "b1": b}, ALL_TRUSTED)
    assert errors["fa1"] == pytest.approx(b["t"] - (50.0 - 5.0) + 2 * 2.0 * 1.5)


def test_untrusted_zones_bays_do_not_count(lcfg):
    trust = {"za": True, "zb": False, "zc": True}
    hot = block(lcfg, prox_b1=prox_for(70.0))
    errors, worst = channel_margin_errors(lcfg, hot, trust)
    assert worst["fa1"] == "a1"
    assert errors["fb1"] == pytest.approx(HDD_E)  # fb1 still serves za's bays


def test_missing_estimate_of_a_trusted_zone_is_a_contract_violation(lcfg):
    b = block(lcfg)
    del b["a2"]
    with pytest.raises(KeyError):
        channel_margin_errors(lcfg, b, ALL_TRUSTED)


def test_fixed_channels_are_skipped(lcfg):
    errors, worst = channel_margin_errors(lcfg, {}, {}, skip={"fa1", "fa2", "fb1"})
    assert set(errors) == {"fc1"} and worst == {"fc1": None}


def test_margin_form_needs_topology(cfg):
    with pytest.raises(ValueError):
        channel_margin_errors(cfg, {}, {})


# ---------------------------------------------------------------------------
# PiSolver in the DAS form
# ---------------------------------------------------------------------------


def test_initialise_then_solve_is_bumpless(lcfg):
    solver = PiSolver()
    req = request(lcfg, prev=0.37, estimates=block(lcfg, prox_a2=prox_for(58.0)))
    integrator, memory = solver.initialise(lcfg, req)
    result = solver.solve(lcfg, dataclasses.replace(req, integrator=integrator, memory=memory))
    assert result.pwm == pytest.approx(dict.fromkeys(lcfg.channels, 0.37), abs=1e-12)


def test_solve_raises_on_a_deficit_and_lowers_on_a_surplus(lcfg):
    solver = PiSolver()
    hot = request(lcfg, integrator=dict.fromkeys(lcfg.channels, 0.5))
    res = solver.solve(lcfg, hot)
    assert res.pwm["fa1"] == pytest.approx(0.5 + lcfg.pi_kp * HDD_E)
    assert res.integrator["fa1"] == pytest.approx(0.5 + lcfg.pi_ki * HDD_E * lcfg.dt)
    assert res.pwm["fc1"] == pytest.approx(0.5)  # unconstrained channel holds
    assert res.integrator["fc1"] == pytest.approx(0.5)
    assert res.diagnostics["form"] == "margin_deficit"
    assert res.diagnostics["worst_bay"] == {"fa1": "a1", "fa2": "a1", "fb1": "a1", "fc1": None}
    assert res.diagnostics["unconstrained"] == ["fc1"]
    cool = block(lcfg, prox_a1=prox_for(30.0), prox_a1b=prox_for(30.0), prox_b1=prox_for(30.0))
    res = solver.solve(
        lcfg, request(lcfg, integrator=dict.fromkeys(lcfg.channels, 0.5), estimates=cool)
    )
    assert res.pwm["fa1"] < 0.5 and res.pwm["fb1"] < 0.5
    json.dumps(res.diagnostics, allow_nan=False)


def test_fixed_channels_get_their_command_and_no_integrator(lcfg):
    req = request(lcfg, fixed_channels={"fc1": 0.9}, integrator=dict.fromkeys(lcfg.channels, 0.5))
    res = PiSolver().solve(lcfg, req)
    assert res.pwm["fc1"] == 0.9 and "fc1" not in res.integrator


def test_legacy_form_on_zoned_setpoints_is_unchanged():
    dcfg = das_cfg()
    temps = {k: v for k, v in default_temps(dcfg).items() if v is not None}
    temps["air_a"] = SP + 2.0
    req = SolverRequest(
        temps=temps,
        prev_pwm=dict.fromkeys(dcfg.channels, 0.5),
        integrator=dict.fromkeys(dcfg.channels, 0.5),
        zone_trust=dict(ALL_TRUSTED),
        estimates=block(dcfg),
    )
    res = PiSolver().solve(dcfg, req)
    assert res.diagnostics["error"] == channel_errors(dcfg, temps)
    assert set(res.diagnostics) == {"error", "p_term", "i_term", "saturated_high", "saturated_low"}


# ---------------------------------------------------------------------------
# mpc.step integration
# ---------------------------------------------------------------------------


class SpySolver(PiSolver):
    def __init__(self) -> None:
        self.requests: list[SolverRequest] = []

    def solve(self, cfg: MpcConfig, req: SolverRequest) -> SolverResult:
        self.requests.append(req)
        return super().solve(cfg, req)


def run_ticks(cfg, ticks, *, state=None, solver=None, t0=0, **temps):
    state = MpcState.cold() if state is None else state
    cmd = None
    for i in range(t0, t0 + ticks):
        pwm = 0.5 if state.last_cmd is None else state.last_cmd.pwm
        kwargs = {"solver": solver} if solver is not None else {}
        cmd, state = checked_step(das_obs(cfg, float(i), pwm=pwm, **temps), cfg, state, **kwargs)
    return cmd, state


def test_step_regulates_hot_drives_up_and_cool_drives_down(lcfg):
    hot = {"prox_a1": prox_for(47.0), "prox_a1b": prox_for(47.0)}
    cmd, _ = run_ticks(lcfg, 8, **hot)
    assert cmd.mode is Mode.AUTO
    assert cmd.pwm["fa1"] > 0.5 + 0.05 and cmd.pwm["fb1"] > 0.5 + 0.05
    assert cmd.pwm["fc1"] == pytest.approx(0.5)  # empty zone: held
    cool = {name: prox_for(30.0) for name in ("prox_a1", "prox_a1b", "prox_a2", "prox_b1")}
    cmd, _ = run_ticks(lcfg, 8, **cool)
    assert cmd.pwm["fa1"] < 0.5 and cmd.pwm["fb1"] < 0.5


def test_step_reports_estimates_in_diagnostics(lcfg):
    cmd, _ = run_ticks(lcfg, 1)
    diag = cmd.diagnostics["estimates"]
    assert set(diag) == {"a1", "a2", "b1"}
    a1 = diag["a1"]
    assert a1["t_c"] == pytest.approx(est.prior_drive_temp(PROX_C, SP))
    assert (a1["soft_c"], a1["hard_c"], a1["margin_c"], a1["sigma_c"]) == (42.0, 47.0, 3.0, 1.5)
    assert a1["zone_trusted"] and a1["source"] == "prior_map" and a1["calibrated"] is False
    assert cmd.diagnostics["solver_diag"]["form"] == "margin_deficit"


def test_request_carries_only_eligible_zones_estimates(lcfg):
    spy = SpySolver()
    _, state = run_ticks(lcfg, 3, solver=spy)
    assert set(spy.requests[-1].estimates) == {"a1", "a2", "b1"}
    assert spy.requests[-1].zone_trust == ALL_TRUSTED
    # zb loses its only air sensor: zb and, through coupling, za go under fallback;
    # zc (no coupling) keeps its channel with the solver.
    spy.requests.clear()
    cmd, state = run_ticks(lcfg, 2, state=state, solver=spy, t0=3, air_b=None)
    assert cmd.mode is Mode.DEGRADED
    req = spy.requests[-1]
    assert req.zone_trust == {"za": True, "zb": False, "zc": True}
    assert set(req.fixed_channels) == {"fa1", "fa2", "fb1"}
    assert "b1" not in req.estimates  # never built from an untrusted zone
    assert cmd.diagnostics["estimates"]["a1"]["zone_trusted"]
    assert "b1" not in cmd.diagnostics["estimates"]  # air_b untrusted: no estimate at all


def test_unconstrained_channel_holds_through_many_ticks(lcfg):
    cmd, _ = run_ticks(lcfg, 30, prox_a1=prox_for(47.0), prox_a1b=prox_for(47.0))
    assert cmd.pwm["fc1"] == pytest.approx(0.5, abs=TOL)


def test_hot_ssd_is_regulated_against_its_own_class():
    """a2 is ssd_sata (65 / 10): 55 degC is below its soft target, above an hdd's."""
    cfg = das_cfg(setpoints={})
    ok = {"prox_a2": prox_for(40.0 - 3.0)}  # 37 + 3 - 52 < 0: no deficit from a2
    cmd, _ = run_ticks(
        cfg, 10, prox_a1=prox_for(30.0), prox_a1b=prox_for(30.0), prox_b1=prox_for(30.0), **ok
    )
    assert cmd.diagnostics["solver_diag"]["error"]["fa1"] < 0
    m = das_mapping()
    m["setpoints"] = {}
    m["topology"]["bays"]["a2"]["class"] = "hdd"
    as_hdd = MpcConfig.from_mapping(m)
    cmd, _ = run_ticks(
        as_hdd,
        10,
        prox_a1=prox_for(30.0),
        prox_a1b=prox_for(30.0),
        prox_b1=prox_for(30.0),
        prox_a2=prox_for(44.0),
    )
    assert cmd.diagnostics["solver_diag"]["error"]["fa1"] == pytest.approx(44.0 + 3.0 - 42.0)
