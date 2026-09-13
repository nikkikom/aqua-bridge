"""Active identification experiments (DAS plan section 5, ``control/ident.py``).

Pure machine: groups and served zones, the seeded two-level sequence, every start
precondition with its reason, the envelope at its thresholds and the abort list.
Through ``Supervisor`` + ``Loop`` on the small DAS fixture: the experiment is a
composed override (``d_pwm_max``, clamp, fallback beats it), the control mode stays
``auto``, every human intent aborts, the release is bumpless, a frozen sensor during
a symmetric run faults its zone and aborts, and a restart never resumes.
"""

from __future__ import annotations

import contextlib
import copy
import dataclasses
import json
import math
from collections.abc import Mapping
from typing import Any

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.control import ident
from aqua_bridge.control.intents import (
    ClearOverride,
    ControlMode,
    Ident,
    IntentConflict,
    IntentInvalid,
    Preset,
    SetBay,
    SetLimit,
    SetMode,
    SetPreset,
    SetPwm,
    SetSetpoint,
    parse_intent,
)
from aqua_bridge.control.loop import Loop
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import ConfigError, Mode, MpcCommand, MpcConfig, PlantObservation
from das_fixtures import das_cfg, das_mapping, das_obs
from invariants import TOL

PROX = ("prox_a1", "prox_a1b", "prox_a2", "prox_b1", "prox_c1")


def ident_cfg(**changes: Any) -> MpcConfig:
    """The DAS fixture with experiments on, short holds and a 120 s experiment.

    ``k_sigma: 1`` keeps the fixture's warm drives (37.5 degC proximal, 35 degC air)
    inside the start band; ``bay_settle_s: 0`` lets bays settle at once."""
    base: dict[str, Any] = {
        "estimator": {"k_sigma": 1.0, "bay_settle_s": 0.0},
        "ident_enabled": True,
        "ident_settle_s": 5.0,
        "ident_max_duration_s": 120.0,
        "ident_hold_s": [5, 10, 15],
    }
    base.update(changes)
    return das_cfg(**base)


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------


def test_ident_defaults_are_the_plan_values_and_legacy_stays_valid(cfg):
    assert cfg.ident_enabled is False
    assert cfg.ident_amplitude == 0.15 and cfg.ident_levels == "above"
    assert cfg.ident_hold_s == (60.0, 120.0, 180.0)
    assert cfg.ident_max_duration_s == 1800.0 and cfg.ident_settle_s == 600.0
    assert cfg.ident_max_over_c == 3.0 and cfg.ident_seed == 1
    # A long dt does not invalidate a config that does not enable experiments.
    MpcConfig.from_mapping({**cfg.to_dict(), "dt": 30.0, "confirm_s": 60.0, "fallback_hold_s": 90})


@pytest.mark.parametrize(
    ("changes", "match"),
    [
        ({"ident_amplitude": 0.0}, "ident_amplitude"),
        ({"ident_amplitude": 0.31}, "ident_amplitude"),
        ({"ident_levels": "below"}, "ident_levels"),
        ({"ident_hold_s": []}, "ident_hold_s"),
        ({"ident_hold_s": [10, -1]}, "ident_hold_s"),
        ({"ident_hold_s": [4]}, "5 \\* dt"),
        ({"ident_max_duration_s": 7201}, "ident_max_duration_s"),
        ({"ident_settle_s": 1.0}, "confirm_s"),
        ({"ident_start_band_c": 0}, "ident_start_band_c"),
        ({"ident_max_over_c": 0}, "ident_max_over_c"),
        ({"ident_seed": -1}, "ident_seed"),
        ({"ident_seed": 1.5}, "ident_seed"),
        ({"ident_enabled": "yes"}, "ident_enabled"),
    ],
)
def test_ident_config_rules(changes, match):
    with pytest.raises(ConfigError, match=match):
        ident_cfg(**changes)


def test_ident_enabled_needs_topology(cfg):
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        MpcConfig.from_mapping({**cfg.to_dict(), "ident_enabled": True})


# ---------------------------------------------------------------------------
# groups, targets, sequence
# ---------------------------------------------------------------------------


def test_groups_targets_and_served_zones():
    cfg = ident_cfg()
    assert ident.groups(cfg) == {"front": ("fa1", "fa2"), "fb1": ("fb1",), "fc1": ("fc1",)}
    assert ident.target_channels(cfg, "group", "front") == ("fa1", "fa2")
    assert ident.target_channels(cfg, "channel", "fa2") == ("fa2",)
    assert ident.target_channels(cfg, "group", "fc1") == ("fc1",)
    for kind, name in (("group", "nope"), ("channel", "front"), ("zone", "za")):
        with pytest.raises(KeyError):
            ident.target_channels(cfg, kind, name)
    # za is coupled to zb: the front group may warm both; zc stands alone.
    assert ident.served_zones(cfg, ("fa1", "fa2")) == ("za", "zb")
    assert ident.served_zones(cfg, ("fc1",)) == ("zc",)


def good_facts(cfg: MpcConfig, ts: float = 1000.0, pwm: float = 0.5, **changes: Any):
    """A tick on which every precondition holds (drives 5 degC under soft)."""
    zones = {z: {"trusted": True, "fault": False} for z in cfg.zone_layout.zones}
    estimates: dict[str, Any] = {}
    bays: dict[str, Any] = {}
    assert cfg.topology is not None
    for bay, spec in cfg.topology.bays.items():
        occ = "empty" if spec.occupied is False else "occupied"
        bays[bay] = {
            "zone": spec.zone,
            "occupancy": occ,
            "since_ts": 0.0,
            "pending_empty_s": 0.0,
            "pending_occupied_ticks": 0,
            "calibration": None,
        }
        if occ != "empty":
            estimates[bay] = {
                "t_c": 38.0,
                "margin_c": 2.0,
                "soft_c": 45.0,
                "hard_c": 48.0,
                "limit_c": 50.0,
            }
    facts = ident.TickFacts(
        ts=ts,
        mode="auto",
        applied=True,
        pwm=dict.fromkeys(cfg.channels, pwm),
        zones=zones,
        estimates=estimates,
        bays=bays,
        saturated=dict.fromkeys(cfg.channels, False),
        fan_stall=dict.fromkeys(cfg.channels, False),
    )
    return dataclasses.replace(facts, **changes)


def settled_tracker(cfg: MpcConfig) -> dict[str, Any]:
    return {"ts": 999.0, "ok_since": dict.fromkeys(cfg.zone_layout.zones, 0.0)}


def test_sequence_is_seeded_two_level_and_inside_the_band():
    cfg = ident_cfg(ident_max_duration_s=600.0)
    facts = good_facts(cfg, pwm=0.4)
    a = ident.start(cfg, facts, "group", "front")
    b = ident.start(cfg, facts, "group", "front")
    assert a == b  # deterministic for a seed
    json.dumps(a, allow_nan=False)
    other = ident.start(dataclasses.replace(cfg, ident_seed=7), facts, "group", "front")
    assert other["phases"] != a["phases"]
    # group phase, then each channel alone
    assert [p["channels"] for p in a["phases"]] == [["fa1", "fa2"], ["fa1"], ["fa2"]]
    assert a["start_ts"] == facts.ts + cfg.dt
    for phase in a["phases"]:
        segs = phase["segments"]
        assert segs[0] == [phase["start_s"], ident.LEVEL_HIGH]
        levels = [lvl for _, lvl in segs]
        assert all(x != y for x, y in zip(levels, levels[1:], strict=False))  # alternates
        holds = [t2 - t1 for (t1, _), (t2, _) in zip(segs, segs[1:], strict=False)]
        assert set(holds) <= set(cfg.ident_hold_s)
        assert len(set(holds)) > 1
    for offset in range(0, 600, 1):
        lv = ident.levels_at(a, float(offset))
        active = set(a["phases"][lv["phase"]]["channels"])
        for ch, u in lv["overrides"].items():
            base = a["base"][ch]
            assert cfg.pwm_min <= u <= cfg.pwm_max
            assert base - TOL <= u <= base + cfg.ident_amplitude + TOL  # above: never below
            if ch not in active:
                assert u == base  # a sibling of a single-channel phase holds its base


def test_hold_sequence_uses_every_hold_and_depends_on_the_seed():
    cfg = ident_cfg(ident_hold_s=[5, 10, 15])
    seq = ident.hold_sequence(cfg, 2000.0)
    assert set(seq) == {5.0, 10.0, 15.0} and sum(seq) >= 2000.0
    assert seq == ident.hold_sequence(cfg, 2000.0)
    assert seq != ident.hold_sequence(dataclasses.replace(cfg, ident_seed=2), 2000.0)


def test_symmetric_levels_straddle_the_base():
    cfg = ident_cfg(ident_levels="symmetric", ident_amplitude=0.2)
    exp = ident.start(cfg, good_facts(cfg), "channel", "fb1")
    assert exp["levels"]["fb1"] == pytest.approx([0.3, 0.7])
    assert [p["channels"] for p in exp["phases"]] == [["fb1"]]


# ---------------------------------------------------------------------------
# preconditions
# ---------------------------------------------------------------------------


def test_good_facts_allow_a_start():
    cfg = ident_cfg()
    assert (
        ident.check_start(
            cfg, settled_tracker(cfg), good_facts(cfg), "group", "front", human_control=False
        )
        == []
    )


def _patched(facts, attr: str, key: str, value: Any, sub: str | None = None):
    data = copy.deepcopy(getattr(facts, attr))
    if sub is None:
        data[key] = value
    else:
        data[key][sub] = value
    return dataclasses.replace(facts, **{attr: data})


PRECONDITIONS = [
    ("control_mode", lambda cfg, f, t: (f, t, True)),
    ("mode:saturated", lambda cfg, f, t: (dataclasses.replace(f, mode="saturated"), t, False)),
    ("mode:degraded", lambda cfg, f, t: (dataclasses.replace(f, mode="degraded"), t, False)),
    ("mode:fallback", lambda cfg, f, t: (dataclasses.replace(f, mode="fallback"), t, False)),
    ("no_tick", lambda cfg, f, t: (dataclasses.replace(f, mode=None), t, False)),
    ("saturated:fa2", lambda cfg, f, t: (_patched(f, "saturated", "fa2", True), t, False)),
    ("no_command:fa1", lambda cfg, f, t: (_patched(f, "pwm", "fa1", math.nan), t, False)),
    ("band:fa1", lambda cfg, f, t: (_patched(f, "pwm", "fa1", 0.9), t, False)),
    ("fan_stall:fc1", lambda cfg, f, t: (_patched(f, "fan_stall", "fc1", True), t, False)),
    ("settle:zb", lambda cfg, f, t: (f, {"ts": 0, "ok_since": {"za": 0.0, "zc": 0.0}}, False)),
    ("settle:za", lambda cfg, f, t: (f, {"ts": 0, "ok_since": {"za": 996.0, "zb": 0.0}}, False)),
    (
        "bay_unknown:b1",
        lambda cfg, f, t: (_patched(f, "bays", "b1", "unknown", "occupancy"), t, False),
    ),
    (
        "bay_transition:a1",
        lambda cfg, f, t: (_patched(f, "bays", "a1", 30.0, "pending_empty_s"), t, False),
    ),
    (
        "bay_transition:a2",
        lambda cfg, f, t: (_patched(f, "bays", "a2", 2, "pending_occupied_ticks"), t, False),
    ),
    (
        "calibrating:b1",
        lambda cfg, f, t: (_patched(f, "bays", "b1", {"samples": 19}, "calibration"), t, False),
    ),
    (
        "start_band:a1",
        lambda cfg, f, t: (_patched(f, "estimates", "a1", 44.1, "t_c"), t, False),
    ),
    ("hard:a1", lambda cfg, f, t: (_patched(f, "estimates", "a1", 39.5, "hard_c"), t, False)),
    (
        "abort_temp:b1",
        lambda cfg, f, t: (_patched(f, "estimates", "b1", 41.0, "limit_c"), t, False),
    ),
    ("no_estimate:a2", lambda cfg, f, t: (_patched(f, "estimates", "a2", None), t, False)),
]


@pytest.mark.parametrize(("reason", "patch"), PRECONDITIONS, ids=[r for r, _ in PRECONDITIONS])
def test_each_precondition_blocks_the_start_with_its_reason(reason, patch):
    cfg = ident_cfg(ident_max_duration_s=600.0)
    facts, tracker, human = patch(cfg, good_facts(cfg), settled_tracker(cfg))
    reasons = ident.check_start(cfg, tracker, facts, "group", "front", human_control=human)
    assert reason in reasons


def test_preconditions_ignore_zones_the_target_does_not_serve():
    cfg = ident_cfg()
    facts = _patched(good_facts(cfg), "zones", "zc", False, "trusted")
    facts = _patched(facts, "estimates", "b1", 60.0, "t_c")  # zb: served by front, not by fb1?
    tracker = {"ts": 999.0, "ok_since": {"za": 0.0, "zb": 0.0}}
    assert ident.check_start(cfg, tracker, facts, "channel", "fc1", human_control=False) == [
        "settle:zc"
    ]
    assert "envelope:b1" not in ident.check_start(
        cfg, tracker, facts, "channel", "fc1", human_control=False
    )


def test_a_settled_bay_calibration_and_an_empty_bay_do_not_block():
    cfg = ident_cfg()
    facts = _patched(good_facts(cfg), "bays", "a1", {"samples": 20}, "calibration")
    facts = _patched(facts, "bays", "c1", "empty", "occupancy")
    assert (
        ident.check_start(cfg, settled_tracker(cfg), facts, "channel", "fc1", human_control=False)
        == []
    )


def test_bay_settle_s_blocks_a_recent_occupancy_change():
    cfg = ident_cfg(estimator={"k_sigma": 1.0, "bay_settle_s": 300.0})
    facts = _patched(good_facts(cfg), "bays", "b1", 800.0, "since_ts")
    assert "bay_transition:b1" in ident.check_start(
        cfg, settled_tracker(cfg), facts, "group", "fb1", human_control=False
    )
    facts = _patched(good_facts(cfg), "bays", "b1", 700.0, "since_ts")
    assert (
        ident.check_start(cfg, settled_tracker(cfg), facts, "group", "fb1", human_control=False)
        == []
    )


def test_track_counts_trusted_fault_free_time_and_restarts_on_a_break():
    cfg = ident_cfg()
    tracker = ident.new_tracker()
    for ts in (10.0, 11.0, 12.0):
        tracker = ident.track(tracker, cfg, good_facts(cfg, ts=ts))
    assert tracker["ok_since"] == {"za": 10.0, "zb": 10.0, "zc": 10.0}
    broken = _patched(good_facts(cfg, ts=13.0), "zones", "zb", True, "fault")
    tracker = ident.track(tracker, cfg, broken)
    assert tracker["ok_since"] == {"za": 10.0, "zc": 10.0}
    tracker = ident.track(tracker, cfg, good_facts(cfg, ts=14.0))
    assert tracker["ok_since"]["zb"] == 14.0
    # clock backwards or a tick without zone diagnostics: every count restarts
    assert ident.track(tracker, cfg, good_facts(cfg, ts=5.0))["ok_since"] == {}
    no_zones = dataclasses.replace(good_facts(cfg, ts=15.0), zones={})
    assert ident.track(tracker, cfg, no_zones)["ok_since"] == {}


# ---------------------------------------------------------------------------
# envelope and abort list (pure)
# ---------------------------------------------------------------------------


def _running(cfg: MpcConfig, **kw: Any) -> dict[str, Any]:
    return ident.start(cfg, good_facts(cfg, ts=1000.0, **kw), "group", "front")


def test_envelope_aborts_at_the_threshold_not_before():
    cfg = ident_cfg()
    exp = _running(cfg)
    at = _patched(good_facts(cfg, ts=1001.0), "estimates", "a1", 45.0 + 3.0 - 2.0, "t_c")
    assert ident.advance(exp, cfg, at).experiment is not None
    over = _patched(good_facts(cfg, ts=1001.0), "estimates", "a1", 46.01, "t_c")
    out = ident.advance(exp, cfg, over)
    assert out.experiment is None
    assert (out.result, out.reason) == (ident.RESULT_ABORTED, "envelope:a1")


def test_hard_and_absolute_limits_abort_even_inside_the_soft_envelope():
    cfg = ident_cfg()
    exp = _running(cfg)
    facts = _patched(good_facts(cfg, ts=1001.0), "estimates", "b1", 39.9, "hard_c")
    assert ident.advance(exp, cfg, facts).reason == "hard:b1"
    facts = _patched(good_facts(cfg, ts=1001.0), "estimates", "b1", 41.0, "limit_c")
    assert ident.advance(exp, cfg, facts).reason == "abort_temp:b1"
    facts = _patched(good_facts(cfg, ts=1001.0), "estimates", "b1", math.inf, "t_c")
    assert ident.advance(exp, cfg, facts).reason == "no_estimate:b1"


ABORTS = [
    ("fallback", lambda f: dataclasses.replace(f, mode="fallback")),
    ("fallback", lambda f: dataclasses.replace(f, mode=None)),
    ("degraded", lambda f: dataclasses.replace(f, mode="degraded")),
    ("clock", lambda f: dataclasses.replace(f, ts=999.0)),
    ("clock", lambda f: dataclasses.replace(f, ts=None)),
    ("duration", lambda f: dataclasses.replace(f, ts=1000.0 + 1.0 + 120.0 + 1.5)),
    ("zone_untrusted:zb", lambda f: _patched(f, "zones", "zb", False, "trusted")),
    ("bay_unknown:a2", lambda f: _patched(f, "bays", "a2", "unknown", "occupancy")),
    ("fan_stall:fa2", lambda f: _patched(f, "fan_stall", "fa2", True)),
]


@pytest.mark.parametrize(("reason", "patch"), ABORTS)
def test_abort_list(reason, patch):
    cfg = ident_cfg()
    exp = _running(cfg)
    before = copy.deepcopy(exp)
    out = ident.advance(exp, cfg, patch(good_facts(cfg, ts=1001.0)))
    assert exp == before  # pure
    assert (out.experiment, out.result, out.reason) == (None, ident.RESULT_ABORTED, reason)


def test_unserved_zone_and_other_channels_do_not_abort():
    cfg = ident_cfg()
    exp = ident.start(cfg, good_facts(cfg), "channel", "fc1")
    facts = _patched(good_facts(cfg, ts=1001.0), "zones", "za", False, "trusted")
    facts = _patched(facts, "fan_stall", "fa1", True)
    facts = _patched(facts, "estimates", "a1", 49.0, "t_c")
    assert ident.advance(exp, cfg, facts).experiment is not None


def test_two_consecutive_apply_failures_abort():
    cfg = ident_cfg()
    exp = _running(cfg)
    one = ident.advance(exp, cfg, good_facts(cfg, ts=1001.0, applied=False))
    assert one.experiment is not None and one.experiment["apply_failures"] == 1
    ok = ident.advance(one.experiment, cfg, good_facts(cfg, ts=1002.0))
    assert ok.experiment is not None and ok.experiment["apply_failures"] == 0
    one = ident.advance(ok.experiment, cfg, good_facts(cfg, ts=1003.0, applied=False))
    two = ident.advance(one.experiment, cfg, good_facts(cfg, ts=1004.0, applied=False))
    assert (two.result, two.reason) == (ident.RESULT_ABORTED, "apply_failed")


def test_the_experiment_completes_after_its_duration():
    cfg = ident_cfg()
    exp = _running(cfg)
    ts = 1000.0
    while True:
        ts += cfg.dt
        out = ident.advance(exp, cfg, good_facts(cfg, ts=ts))
        if out.experiment is None:
            break
        exp = out.experiment
        json.dumps(exp, allow_nan=False)
    assert out.result == ident.RESULT_COMPLETED and out.reason is None
    # the last tick that ran with overrides was the one at offset duration - dt
    assert ts - (1000.0 + cfg.dt) == pytest.approx(cfg.ident_max_duration_s - cfg.dt)


_facts_strategy = st.fixed_dictionaries(
    {
        "dt": st.floats(0.0, 3.0),
        "mode": st.sampled_from(["auto", "auto", "auto", "saturated", "degraded", "fallback"]),
        "applied": st.booleans(),
        "t_c": st.floats(30.0, 52.0),
        "pwm": st.floats(0.0, 1.5),
        "trusted": st.booleans(),
    }
)


@settings(max_examples=60, deadline=None)
@given(
    seq=st.lists(_facts_strategy, min_size=1, max_size=60),
    levels=st.sampled_from(["above", "symmetric"]),
)
def test_random_tick_sequences_never_raise_and_keep_levels_in_bounds(seq, levels):
    cfg = ident_cfg(ident_levels=levels, ident_amplitude=0.2)
    exp: dict[str, Any] | None = ident.start(cfg, good_facts(cfg, pwm=0.5), "group", "front")
    ts = 1000.0
    for item in seq:
        if exp is None:
            break
        ts += item["dt"]
        facts = good_facts(cfg, ts=ts, pwm=item["pwm"], mode=item["mode"], applied=item["applied"])
        facts = _patched(facts, "estimates", "a1", item["t_c"], "t_c")
        facts = _patched(facts, "zones", "za", item["trusted"], "trusted")
        out = ident.advance(exp, cfg, facts)
        if out.experiment is None:
            assert out.result in (ident.RESULT_ABORTED, ident.RESULT_COMPLETED)
            if out.result == ident.RESULT_COMPLETED:
                assert item["mode"] == "auto" and item["trusted"]
            break
        exp = out.experiment
        assert item["mode"] in ("auto", "saturated")
        assert item["t_c"] + 2.0 <= 45.0 + cfg.ident_max_over_c + TOL
        json.dumps(exp, allow_nan=False)
        for u in exp["overrides"].values():
            assert cfg.pwm_min <= u <= cfg.pwm_max
            lo = 0.5 - 0.2 if levels == "symmetric" else 0.5
            assert lo - TOL <= u <= 0.5 + 0.2 + TOL


# ---------------------------------------------------------------------------
# intent parsing
# ---------------------------------------------------------------------------


def test_parse_ident_intent():
    assert parse_intent("ident", {"action": "start", "group": "front"}) == Ident(
        "start", group="front"
    )
    assert parse_intent("ident", {"action": "start", "channel": "fa1"}) == Ident(
        "start", channel="fa1"
    )
    assert parse_intent("ident", {"action": "stop"}) == Ident("stop")


@pytest.mark.parametrize(
    "body",
    [
        {},
        {"action": "go", "group": "front"},
        {"action": "start"},
        {"action": "start", "group": "front", "channel": "fa1"},
        {"action": "stop", "group": "front"},
        {"action": "start", "group": ""},
        {"action": "start", "channel": 3},
        {"action": "start", "group": "front", "extra": 1},
        {"action": ["start"]},
        [],
    ],
)
def test_parse_ident_rejects_malformed_bodies(body):
    with pytest.raises(IntentInvalid):
        parse_intent("ident", body)


# ---------------------------------------------------------------------------
# Supervisor + Loop
# ---------------------------------------------------------------------------


class Rig:
    """A Loop on the DAS fixture with a plant that holds the commanded PWM."""

    def __init__(self, cfg: MpcConfig, **temps: float) -> None:
        self.cfg = cfg
        self.t = 0.0
        self.temps: dict[str, float | None] = {name: 37.5 for name in PROX}
        self.temps.update(temps)
        self.wiggle: dict[str, float] = {}
        self.drop: tuple[str, ...] = ()
        self.pwm = dict.fromkeys(cfg.channels, 0.5)
        self.fail = 0
        self.sup = Supervisor(cfg)
        self.loop = Loop(self, self, cfg, self.sup)
        self.results: list[Any] = []

    def read(self) -> PlantObservation:
        self.t += self.cfg.dt
        temps = dict(self.temps)
        sign = 1.0 if int(self.t) % 2 else -1.0
        for name, amp in self.wiggle.items():
            base = temps.get(name)
            temps[name] = (35.0 if base is None else base) + sign * amp
        return das_obs(self.cfg, self.t, pwm=self.pwm, drop=self.drop, **temps)

    def apply(self, cmd: MpcCommand) -> None:
        if self.fail:
            self.fail -= 1
            raise OSError("usb gone")
        self.pwm = dict(cmd.pwm)

    def ticks(self, n: int) -> list[Any]:
        out = []
        for _ in range(n):
            prev = dict(self.pwm)
            r = self.loop.tick()
            if r.applied:
                for ch in self.cfg.channels:
                    assert abs(r.cmd.pwm[ch] - prev[ch]) <= self.cfg.d_pwm_max + TOL
                    assert self.cfg.pwm_min - TOL <= r.cmd.pwm[ch] <= self.cfg.pwm_max + TOL
            out.append(r)
        self.results.extend(out)
        return out

    @property
    def status(self) -> dict[str, Any]:
        return self.sup.snapshot().extra["experiment"]


def started_rig(cfg: MpcConfig | None = None, target: Ident | None = None, **temps) -> Rig:
    rig = Rig(cfg or ident_cfg(), **temps)
    rig.ticks(8)
    rig.sup.submit(target or Ident("start", group="front"))
    assert rig.status["running"]
    return rig


def test_an_experiment_runs_as_a_composed_override_in_auto():
    rig = started_rig()
    base = rig.status["base"]
    seen_high = False
    for r in rig.ticks(40):
        snap = rig.sup.snapshot()
        assert snap.control_mode is ControlMode.AUTO and snap.overrides == {}
        sup_diag = r.cmd.diagnostics["supervisor"]
        assert sup_diag["control_mode"] == "auto"
        assert sup_diag["experiment"]["running"] is True
        assert sup_diag["overrides_applied"] is True
        assert r.cmd.pwm["fb1"] == r.mpc_cmd.pwm["fb1"]  # other channels: the solver
        for ch in ("fa1", "fa2"):
            assert r.cmd.pwm[ch] >= base[ch] - TOL  # above: never below the frozen base
            seen_high |= r.cmd.pwm[ch] > base[ch] + 0.1
    assert seen_high
    assert rig.status["target"] == {"kind": "group", "name": "front"}


def test_a_group_runs_its_phases_then_completes_and_releases_bumplessly():
    rig = started_rig()
    phases_seen = set()
    released_at = None
    for i in range(200):
        (r,) = rig.ticks(1)
        st_ = rig.status
        if st_["running"]:
            phases_seen.add(st_["phase"])
            continue
        released_at = i
        break
    assert released_at is not None
    assert phases_seen == {0, 1, 2}
    assert rig.status["last_result"] == "completed"
    prev = dict(rig.pwm)
    (r,) = rig.ticks(1)
    # bumpless: the solver re-initialises on the released channels (first target = prev)
    for ch in ("fa1", "fa2"):
        assert r.mpc_cmd.diagnostics["target_pwm"][ch] == pytest.approx(prev[ch])
    assert "experiment" not in r.cmd.diagnostics["supervisor"]


def _all_intents(cfg: MpcConfig) -> list[Any]:
    return [
        SetMode(ControlMode.MIXED),
        SetMode(ControlMode.AUTO),
        SetPwm("fa1", 0.5),  # rejected in auto (409), still aborts
        SetSetpoint("air_a", 34.0),
        SetPreset(Preset.COOL),
        ClearOverride(),
        SetLimit(limit_c=48.0, drive_class="hdd"),
        SetBay("a1", {"occupied": True}),
    ]


@pytest.mark.parametrize("index", range(8))
def test_every_human_intent_aborts_and_releases(index):
    rig = started_rig()
    rig.ticks(3)
    intent = _all_intents(rig.cfg)[index]
    with contextlib.suppress(IntentConflict, IntentInvalid):
        rig.sup.submit(intent)
    assert not rig.status["running"]
    assert rig.status["last_result"] == "aborted"
    assert rig.status["last_abort_reason"].startswith("human_intent:")
    plan = rig.sup.plan_tick()
    assert {"fa1", "fa2"} <= plan.released
    assert plan.experiment is None


def test_stop_intent_aborts_and_stop_without_an_experiment_is_a_no_op():
    rig = started_rig()
    rig.ticks(2)
    rig.sup.submit(Ident("stop"))
    assert rig.status["last_abort_reason"] == "stop" and not rig.status["running"]
    rig.sup.submit(Ident("stop"))
    assert rig.status["last_abort_reason"] == "stop"


def test_start_refusals_through_the_supervisor():
    cfg = ident_cfg()
    rig = Rig(cfg)
    with pytest.raises(IntentConflict, match="no_tick"):
        rig.sup.submit(Ident("start", group="front"))
    rig.ticks(2)
    with pytest.raises(IntentConflict, match="settle:za"):
        rig.sup.submit(Ident("start", group="front"))
    rig.ticks(6)
    with pytest.raises(IntentInvalid, match="unknown group"):
        rig.sup.submit(Ident("start", group="rear"))
    with pytest.raises(IntentInvalid, match="unknown channel"):
        rig.sup.submit(Ident("start", channel="front"))
    rig.sup.submit(SetMode(ControlMode.MIXED))
    with pytest.raises(IntentConflict, match="control_mode"):
        rig.sup.submit(Ident("start", group="front"))
    rig.sup.submit(SetMode(ControlMode.AUTO))
    rig.sup.submit(Ident("start", channel="fb1"))
    with pytest.raises(IntentConflict, match="already running"):
        rig.sup.submit(Ident("start", group="front"))
    disabled = Rig(ident_cfg(ident_enabled=False))
    disabled.ticks(8)
    with pytest.raises(IntentConflict, match="ident_enabled"):
        disabled.sup.submit(Ident("start", group="front"))


def test_a_hot_drive_refuses_the_start_and_a_warming_drive_aborts():
    hot = Rig(ident_cfg(), prox_b1=40.0)
    hot.ticks(8)
    with pytest.raises(IntentConflict, match="start_band:b1"):
        hot.sup.submit(Ident("start", group="front"))
    rig = started_rig()
    rig.ticks(3)
    for _ in range(40):  # the drive warms at a plausible rate until the envelope trips
        rig.temps["prox_b1"] = float(rig.temps["prox_b1"]) + 0.25  # type: ignore[arg-type]
        rig.ticks(1)
        if not rig.status["running"]:
            break
    assert rig.status["last_abort_reason"] == "envelope:b1"
    assert rig.results[-1].mpc_cmd.mode is Mode.AUTO


def test_fallback_beats_the_experiment_and_aborts_it():
    rig = started_rig()
    rig.ticks(3)
    before = dict(rig.pwm)
    rig.drop = ("air_a", "air_a2")  # zone za loses its air sensors: za and zb under fallback
    (r,) = rig.ticks(1)
    assert r.mpc_cmd.mode is Mode.DEGRADED
    for ch in ("fa1", "fa2", "fb1"):
        assert r.cmd.pwm[ch] == r.mpc_cmd.pwm[ch]  # the override is blocked
        assert r.cmd.pwm[ch] >= before[ch] - TOL  # a fault never reduces cooling
    assert not rig.status["running"] and rig.status["last_abort_reason"] == "degraded"
    for r in rig.ticks(10):
        assert "experiment" not in r.cmd.diagnostics["supervisor"]


def test_every_zone_in_fault_aborts_with_fallback():
    rig = started_rig()
    rig.ticks(2)
    rig.drop = tuple(rig.cfg.temps)
    (r,) = rig.ticks(1)
    assert r.mpc_cmd.mode is Mode.FALLBACK
    assert rig.status["last_abort_reason"] == "fallback"


def test_an_emergency_command_after_a_good_solver_tick_aborts(monkeypatch):
    # Reviewer finding: when compose raises after step succeeded, the loop applies the
    # emergency (fallback) command but still reports the solver's auto command. That
    # tick is a fallback tick and must abort the experiment and restart the settle count.
    rig = started_rig()
    rig.ticks(3)

    def boom(*args, **kwargs):
        raise RuntimeError("compose bug")

    monkeypatch.setattr(rig.sup, "compose", boom)
    (r,) = rig.ticks(1)
    assert r.controller_error is not None and r.cmd.mode is Mode.FALLBACK
    assert r.mpc_cmd is not None and r.mpc_cmd.mode is Mode.AUTO
    assert not rig.status["running"]
    assert rig.status["last_abort_reason"] == "fallback"
    monkeypatch.undo()
    rig.ticks(1)
    with pytest.raises(IntentConflict, match="settle"):
        rig.sup.submit(Ident("start", group="front"))


def test_two_apply_failures_abort_through_the_loop():
    rig = started_rig()
    rig.ticks(2)
    rig.fail = 1
    rig.ticks(2)
    assert rig.status["running"]
    rig.fail = 2
    rig.ticks(2)
    assert rig.status["last_abort_reason"] == "apply_failed"


def test_a_frozen_sensor_during_a_symmetric_run_faults_its_zone_and_aborts():
    m = das_mapping()
    m["sensors"]["air_a"]["stuck_s"] = 6
    m["sensors"]["air_a2"]["stuck_s"] = 6
    m.update(
        estimator={"k_sigma": 1.0, "bay_settle_s": 0.0},
        ident_enabled=True,
        ident_settle_s=5.0,
        ident_max_duration_s=300.0,
        ident_hold_s=[5, 10, 15],
        ident_levels="symmetric",
        ident_amplitude=0.3,
    )
    cfg = MpcConfig.from_mapping(m)
    rig = Rig(cfg)
    # every other sensor is alive; zone za's air sensors are frozen
    rig.wiggle = {n: 0.05 for n in cfg.temps if n not in ("air_a", "air_a2")}
    rig.temps.update({n: 35.0 for n in cfg.temps if n not in PROX and n != "inlet"})
    rig.temps["inlet"] = 25.0
    rig.ticks(8)
    rig.sup.submit(Ident("start", group="front"))
    for _ in range(60):
        rig.ticks(1)
        if not rig.status["running"]:
            break
    assert rig.status["last_abort_reason"] == "degraded"
    last = rig.results[-1]
    assert "za" in last.mpc_cmd.diagnostics["zones_in_fault"]
    assert any(
        "stuck" in str(reason) for reason in last.mpc_cmd.diagnostics["zones"]["za"]["reasons"]
    )


def test_a_restart_never_resumes():
    rig = started_rig()
    rig.ticks(4)
    fresh = Rig(rig.cfg)
    assert fresh.sup.experiment is None
    assert fresh.status["running"] is False and fresh.status["last_result"] is None
    fresh.ticks(2)
    with pytest.raises(IntentConflict, match="settle"):
        fresh.sup.submit(Ident("start", group="front"))


def test_status_is_json_and_counts_time():
    rig = started_rig()
    rig.ticks(5)
    st_ = rig.status
    json.dumps(st_, allow_nan=False)
    assert st_["elapsed_s"] == pytest.approx(5.0)
    assert st_["remaining_s"] == pytest.approx(rig.cfg.ident_max_duration_s - 5.0)
    assert st_["group"] == "front" and st_["phases"] == 3 and st_["level"] in ("high", "low")
    assert st_["channels"] == ["fa1", "fa2"] and st_["channel"] is None


def test_bookkeeping_errors_abort_and_never_raise(monkeypatch):
    rig = started_rig()

    def boom(*args, **kwargs):
        raise RuntimeError("bug")

    monkeypatch.setattr(ident, "advance", boom)
    rig.ticks(1)
    assert rig.status["last_abort_reason"] == "error"


def test_legacy_config_rejects_ident_and_keeps_its_snapshot_and_diagnostics(cfg):
    sup = Supervisor(cfg)
    with pytest.raises(IntentInvalid, match="DAS"):
        sup.submit(Ident("start", channel=cfg.channels[0]))
    assert "experiment" not in sup.snapshot().extra
    plan = sup.plan_tick()
    assert plan.experiment is None
    cmd = MpcCommand(pwm=dict.fromkeys(cfg.channels, 0.5), mode=Mode.AUTO, diagnostics={})
    out = sup.compose(cmd, plan, dict.fromkeys(cfg.channels, 0.5))
    assert "experiment" not in out.diagnostics["supervisor"]


def test_snapshot_extra_experiment_in_das_mode_only(cfg):
    das = Supervisor(ident_cfg())
    shown = das.snapshot().extra["experiment"]
    assert shown["running"] is False and shown["enabled"] is True
    assert isinstance(Supervisor(cfg).snapshot().extra, Mapping)
