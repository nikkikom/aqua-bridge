"""Active identification experiments (DAS plan section 5, ``control/ident.py``).

Pure machine: groups and served zones, the seeded two-level sequence, every start
precondition with its reason, the envelope at its thresholds and the abort list.
Levels re-planned from the live solver demand (item 52): the anchor follows a rise at
once and a fall only at a switch, the echo of the experiment's own level is taken out of
the want first (so no ratchet builds up) while a fan the solver's own floor lifted still
reads as demand, a held sibling follows it too, the levels stay inside the band,
``symmetric`` dips at most ``ident_amplitude`` below the demand, no abort of a tick
moves, the dip and the floor come from the running experiment's own plan, and
``ident_replan: false`` keeps the frozen plan of the Zero W at its old cost.

A whole zone at once (``ident_parallel``, item 102): the widened target, the single
coded phase with an independent code per channel, the band check over every channel it
now drives, the anchor re-planned when *any* channel of the phase switches, and what the
key costs under ``ident_levels: symmetric`` -- the whole zone, not one group of it, a
step under the solver's command, still floored by ``compose``.

Through ``Supervisor`` + ``Loop`` on the small DAS fixture: the experiment is a
composed override (``d_pwm_max``, clamp, fallback beats it), the control mode stays
``auto``, a warming zone is followed instead of held back, a spent excursion is released
instead of pinning the fans at its peak, an experiment that follows the demand ends later
than the frozen one and never earlier, every human intent aborts, the release is
bumpless, a frozen sensor during a symmetric run faults its zone and aborts, and a
restart never resumes.
"""

from __future__ import annotations

import contextlib
import copy
import dataclasses
import json
import math
import statistics
from collections.abc import Mapping
from typing import Any

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from aqua_bridge.control import ident, thermal
from aqua_bridge.control import supervisor as supervisor_mod
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
    assert cfg.ident_replan is True  # item 52: the levels follow the live demand
    assert cfg.ident_parallel is False  # item 102: one group at a time unless asked
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
        ({"ident_replan": "yes"}, "ident_replan"),
        ({"ident_parallel": "yes"}, "ident_parallel"),
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
        demand=dict.fromkeys(cfg.channels, pwm),
        prev=dict.fromkeys(cfg.channels, pwm),
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
# ident_parallel: the target's own zones, each channel on its own code (item 102)
# ---------------------------------------------------------------------------


def test_parallel_widens_the_target_to_its_own_zones():
    cfg = ident_cfg(ident_parallel=True)
    assert ident.zone_channels(cfg, ("fa1",)) == ("fa1", "fa2")  # za lists both
    assert ident.zone_channels(cfg, ("fb1",)) == ("fb1",)
    # zb is only *coupled* to za: coupling is served and aborted on, never excited
    assert ident.served_zones(cfg, ("fa1",)) == ("za", "zb")
    assert ident.target_channels(cfg, "channel", "fa1") == ("fa1", "fa2")
    assert ident.target_channels(cfg, "group", "front") == ("fa1", "fa2")
    assert ident.target_channels(cfg, "channel", "fc1") == ("fc1",)
    # off, the target is what it always was
    off = ident_cfg()
    assert ident.target_channels(off, "channel", "fa1") == ("fa1",)


def test_parallel_runs_one_phase_with_an_independent_code_per_channel():
    cfg = ident_cfg(ident_parallel=True, ident_max_duration_s=600.0)
    exp = ident.start(cfg, good_facts(cfg, pwm=0.4), "channel", "fa1")
    assert exp == ident.start(cfg, good_facts(cfg, pwm=0.4), "channel", "fa1")  # deterministic
    json.dumps(exp, allow_nan=False)
    assert [p["channels"] for p in exp["phases"]] == [["fa1", "fa2"]]
    codes = exp["phases"][0]["codes"]
    assert set(codes) == {"fa1", "fa2"} and "segments" not in exp["phases"][0]
    for segs in codes.values():
        levels = [lvl for _, lvl in segs]
        assert all(x != y for x, y in zip(levels, levels[1:], strict=False))  # alternates
        holds = [t2 - t1 for (t1, _), (t2, _) in zip(segs, segs[1:], strict=False)]
        assert set(holds) <= set(cfg.ident_hold_s)
    # the two codes are not the same telegraph: different start level or different switches
    assert codes["fa1"] != codes["fa2"]
    seen: set[tuple[int, ...]] = set()
    for offset in range(0, 600):
        lv = ident.levels_at(exp, float(offset))
        seen.add(tuple(lv["code"]))
        for ch, u in lv["overrides"].items():
            base = exp["base"][ch]
            assert cfg.pwm_min <= u <= cfg.pwm_max
            assert base - TOL <= u <= base + cfg.ident_amplitude + TOL  # above: never below
    assert seen == {(0, 0), (0, 1), (1, 0), (1, 1)}  # all four corners, so not collinear
    assert ident.hold_sequence(cfg, 600.0) == ident.hold_sequence(ident_cfg(), 600.0)


def test_parallel_leaves_a_single_channel_zone_and_the_dip_alone():
    cfg = ident_cfg(ident_parallel=True, ident_levels="symmetric", ident_amplitude=0.2)
    exp = ident.start(cfg, good_facts(cfg), "channel", "fb1")
    assert [p["channels"] for p in exp["phases"]] == [["fb1"]]
    assert exp["levels"]["fb1"] == pytest.approx([0.3, 0.7])
    assert ident.planned_dip(exp, "fb1", cfg) == pytest.approx(0.2)


def test_parallel_off_leaves_the_schedule_byte_identical():
    off = ident_cfg()
    explicit = ident_cfg(ident_parallel=False)
    facts = good_facts(off, pwm=0.4)
    assert ident.start(off, facts, "group", "front") == ident.start(
        explicit, facts, "group", "front"
    )
    exp = ident.start(off, facts, "group", "front")
    assert all("codes" not in p and "segments" in p for p in exp["phases"])
    assert "code" not in ident.levels_at(exp, 0.0)


def test_a_parallel_start_checks_the_band_on_every_channel_it_drives():
    cfg = ident_cfg(ident_parallel=True, ident_amplitude=0.2)
    # fa2 alone would leave the band; a start on fa1 now drives fa2 too, so it is refused
    base = good_facts(cfg, pwm=0.5)
    facts = dataclasses.replace(base, pwm={**base.pwm, "fa2": 0.9})
    assert "band:fa2" in ident.check_start(
        cfg, settled_tracker(cfg), facts, "channel", "fa1", human_control=False
    )
    assert not ident.check_start(
        ident_cfg(ident_amplitude=0.2),
        settled_tracker(cfg),
        facts,
        "channel",
        "fa1",
        human_control=False,
    )


def test_a_coded_phase_re_anchors_when_any_of_its_channels_switches():
    """The anchor may fall only on a tick that ends a hold (item 52), and with one code
    per channel a hold ends whenever *any* channel switches -- not only the first
    channel, which is all :func:`ident._position` reads. Without that the other
    channels' holds would start from an anchor the last rise left where it was."""
    cfg = ident_cfg(ident_parallel=True, ident_max_duration_s=600.0)
    exp = ident.start(cfg, good_facts(cfg, pwm=0.5), "channel", "fa1")
    assert exp["phases"][0]["channels"] == ["fa1", "fa2"]
    codes = [tuple(ident.levels_at(exp, float(o))["code"]) for o in range(120)]
    # an offset where only the *second* channel switches (so the schedule position
    # ``_position`` reads has not moved), and one where neither does
    only_second = next(
        o
        for o in range(2, 120)
        if codes[o][0] == codes[o - 1][0] and codes[o][1] != codes[o - 1][1]
    )
    assert ident._position(exp, float(only_second)) == ident._position(exp, float(only_second - 1))
    neither = next(o for o in range(2, only_second) if codes[o] == codes[o - 1])

    def anchor_at(offset: int) -> float:
        """Walk to ``offset`` with the demand at 0.7 (the anchor follows a rise at once),
        then one tick with it back at 0.5, and report the anchor that tick planned."""
        run = exp
        for o in range(1, offset + 1):
            want = 0.7 if o < offset else 0.5
            out = _tick(cfg, run, exp["start_ts"] + o - cfg.dt, fa1=want, fa2=want)
            assert out.experiment is not None
            run = out.experiment
        return float(run["plan_base"]["fa1"])

    assert anchor_at(neither) == pytest.approx(0.7)  # mid-hold: the fall waits
    assert anchor_at(only_second) == pytest.approx(0.6)  # switching: down by d_pwm_max


def test_a_symmetric_parallel_phase_can_put_a_whole_zone_under_the_solver_at_once():
    """What ``ident_parallel`` costs under ``ident_levels: symmetric``, and the floor
    that still holds. ``compose`` accepts a dip of ``ident_amplitude`` below the solver's
    command on an experiment channel; before this key at most one *group* of a zone was
    at its low level at a time (a group's siblings are held at base), so that was all the
    cooling a tick could give up. A coded phase draws a level per channel, so the whole
    zone can be under the solver's command on the same tick -- never by more than the
    dip, and never at all under the default ``above``."""
    # za's two channels are two groups here, so the old schedule could only drive one
    fans = {**das_mapping()["fans"], "fa2": {"model": "p12"}}
    kw: dict[str, Any] = {"ident_levels": "symmetric", "ident_amplitude": 0.2, "fans": fans}
    dip = 0.2

    def run(parallel: bool, target: Ident) -> tuple[bool, bool]:
        rig = started_rig(ident_cfg(ident_parallel=parallel, **kw), target)
        zone_under = fa2_under = False
        for r in rig.ticks(60):
            assert rig.status["running"]
            under = []
            for ch in ("fa1", "fa2"):
                assert r.cmd.pwm[ch] >= r.mpc_cmd.pwm[ch] - dip - TOL  # the floor holds
                # a real dip, not the rate limit lagging a rise: at least half of it
                under.append(r.cmd.pwm[ch] < r.mpc_cmd.pwm[ch] - dip / 2.0)
            zone_under |= all(under)
            fa2_under |= under[1]
        return zone_under, fa2_under

    assert run(True, Ident("start", channel="fa1")) == (True, True)
    # one group at a time: fa2 is not the target's group, so it is never taken down
    assert run(False, Ident("start", group="front")) == (False, False)


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
    (  # k sigma counts once: T_hat itself, not T_hat + margin, against soft (item 53)
        "start_band:a1",
        lambda cfg, f, t: (_patched(f, "estimates", "a1", 46.1, "t_c"), t, False),
    ),
    ("hard:a1", lambda cfg, f, t: (_patched(f, "estimates", "a1", 37.5, "hard_c"), t, False)),
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


def _lost(facts, zone: str, *labels: str):
    floor = dict(facts.sigma_floor)
    floor[zone] = {"lost": list(labels), "phase": "hold", "held_s": 0.0, "floor": {}}
    return dataclasses.replace(facts, sigma_floor=floor)


def test_a_lost_sensor_group_blocks_the_start_under_sigma():
    # item 72: under trust_rule sigma the zone is still trusted, so nothing else in the
    # precondition list sees the loss.
    cfg = ident_cfg()
    facts = _lost(good_facts(cfg), "za", "bay:a1")
    assert facts.zones["za"]["trusted"] is True
    reasons = ident.check_start(
        cfg, settled_tracker(cfg), facts, "group", "front", human_control=False
    )
    assert reasons == ["sensor_lost:za"]
    # a zone the target does not serve does not block it
    far = _lost(good_facts(cfg), "zc", "zone_air")
    assert (
        ident.check_start(cfg, settled_tracker(cfg), far, "group", "front", human_control=False)
        == []
    )
    # an episode that has run its course (nothing lost any more) does not block it
    closed = dataclasses.replace(
        good_facts(cfg),
        sigma_floor={"za": {"lost": [], "phase": "released", "held_s": 90.0, "floor": {}}},
    )
    assert (
        ident.check_start(cfg, settled_tracker(cfg), closed, "group", "front", human_control=False)
        == []
    )


def test_a_lost_sensor_group_aborts_a_running_experiment():
    cfg = ident_cfg()
    exp = _running(cfg)
    facts = _lost(good_facts(cfg, ts=1001.0), "zb", "zone_air")
    out = ident.advance(exp, cfg, facts)
    assert (out.experiment, out.result, out.reason) == (
        None,
        ident.RESULT_ABORTED,
        "sensor_lost:zb",
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


def _store_facts(cfg, ts: float, credit: dict[str, float]):
    return dataclasses.replace(
        good_facts(cfg, ts=ts), store={"ident_settle": {"ts": ts, "credit_s": credit}}
    )


def test_a_restored_settle_credit_is_spent_when_the_zone_is_trusted_again():
    # item 20: the seconds a zone had settled before the shutdown, minus the outage,
    # which persist.apply_seed has already subtracted.
    cfg = ident_cfg()
    tracker = ident.resume_tracker(ident.new_tracker(), _store_facts(cfg, 100.0, {"za": 400.0}))
    assert tracker["resume"] == {"ts": 100.0, "credit_s": {"za": 400.0}}
    blind = dataclasses.replace(good_facts(cfg, ts=100.0), zones={})
    tracker = ident.track(tracker, cfg, blind)  # the zones are not trusted yet
    assert tracker["ok_since"] == {} and tracker["resume"]["credit_s"] == {"za": 400.0}
    tracker = ident.track(tracker, cfg, good_facts(cfg, ts=130.0))
    assert tracker["ok_since"]["za"] == pytest.approx(130.0 - (400.0 - 30.0))
    assert tracker["ok_since"]["zb"] == 130.0  # no credit for zb
    assert "resume" not in tracker  # spent
    settled = ident.check_start(
        cfg, tracker, good_facts(cfg, ts=130.0), "group", "front", human_control=False
    )
    assert "settle:za" not in settled and "settle:zb" in settled


def test_a_settle_credit_decays_while_the_zone_stays_untrusted_and_a_bad_clock_drops_it():
    cfg = ident_cfg()
    tracker = ident.resume_tracker(ident.new_tracker(), _store_facts(cfg, 0.0, {"za": 10.0}))
    late = ident.track(tracker, cfg, good_facts(cfg, ts=100.0))
    assert late["ok_since"]["za"] == 100.0  # the credit ran out on the way
    back = ident.track(dict(tracker, ts=50.0), cfg, good_facts(cfg, ts=5.0))
    assert "resume" not in back  # a clock running backwards drops everything


def test_settle_snapshot_is_seconds_and_round_trips_through_the_store():
    cfg = ident_cfg()
    tracker = ident.new_tracker()
    for ts in (10.0, 11.0, 12.0):
        tracker = ident.track(tracker, cfg, good_facts(cfg, ts=ts))
    assert ident.settle_snapshot(tracker) == {"za": 2.0, "zb": 2.0, "zc": 2.0}
    assert ident.settle_snapshot({"ts": None, "ok_since": {"za": 1.0}}) == {}
    assert ident.settle_snapshot("nonsense") == {}


@pytest.mark.parametrize(
    "raw",
    [
        None,
        5,
        {"ts": 1.0},
        {"ts": None, "credit_s": {"za": 1.0}},
        {"ts": 1.0, "credit_s": {"za": -1.0}},
    ],
)
def test_a_malformed_settle_credit_is_ignored(raw):
    cfg = ident_cfg()
    facts = dataclasses.replace(good_facts(cfg, ts=1.0), store={"ident_settle": raw})
    assert "resume" not in ident.resume_tracker(ident.new_tracker(), facts)


# ---------------------------------------------------------------------------
# envelope and abort list (pure)
# ---------------------------------------------------------------------------


def _running(cfg: MpcConfig, **kw: Any) -> dict[str, Any]:
    return ident.start(cfg, good_facts(cfg, ts=1000.0, **kw), "group", "front")


def test_envelope_aborts_at_the_threshold_not_before():
    # soft 45.0 and k sigma is inside soft already (item 53); ident_max_over_c is kept
    # under the absolute abort here (limit 50.0, margin 2.0) so the soft rule is the one
    # that fires
    cfg = ident_cfg(ident_max_over_c=1.0)
    exp = _running(cfg)
    at = _patched(good_facts(cfg, ts=1001.0), "estimates", "a1", 45.0 + 1.0, "t_c")
    assert ident.advance(exp, cfg, at).experiment is not None
    over = _patched(good_facts(cfg, ts=1001.0), "estimates", "a1", 46.01, "t_c")
    out = ident.advance(exp, cfg, over)
    assert out.experiment is None
    assert (out.result, out.reason) == (ident.RESULT_ABORTED, "envelope:a1")


def test_a_settled_enclosure_at_its_soft_target_starts_and_runs(sigma_c=2.0):
    # PI-like DAS regulates T_hat to soft; before item 53 that state was refused by the
    # start band and sat on the abort edge.
    cfg = ident_cfg()
    facts = _patched(good_facts(cfg), "estimates", "a1", 45.0, "t_c")
    facts = _patched(facts, "estimates", "b1", 45.0, "t_c")
    facts = _patched(facts, "estimates", "a2", 45.0, "t_c")
    assert (
        ident.check_start(cfg, settled_tracker(cfg), facts, "group", "front", human_control=False)
        == []
    )
    exp = ident.start(cfg, facts, "group", "front")
    ahead = dataclasses.replace(facts, ts=facts.ts + cfg.dt)
    assert ident.advance(exp, cfg, ahead).experiment is not None


def test_hard_and_absolute_limits_abort_even_inside_the_soft_envelope():
    cfg = ident_cfg()
    exp = _running(cfg)
    facts = _patched(good_facts(cfg, ts=1001.0), "estimates", "b1", 37.9, "hard_c")
    assert ident.advance(exp, cfg, facts).reason == "hard:b1"
    facts = _patched(good_facts(cfg, ts=1001.0), "estimates", "b1", 41.0, "limit_c")
    assert ident.advance(exp, cfg, facts).reason == "abort_temp:b1"
    facts = _patched(good_facts(cfg, ts=1001.0), "estimates", "b1", math.inf, "t_c")
    assert ident.advance(exp, cfg, facts).reason == "no_estimate:b1"


def test_the_absolute_abort_margin_is_a_config_key():
    # item 54: ident_abort_below_limit_c, the only rule measured against the raw limit
    facts = _patched(good_facts(ident_cfg(), ts=1001.0), "estimates", "b1", 43.0, "limit_c")
    wide = ident_cfg(ident_abort_below_limit_c=3.0)
    assert ident.advance(_running(wide), wide, facts).reason == "abort_temp:b1"
    narrow = ident_cfg(ident_abort_below_limit_c=1.0)
    assert ident.advance(_running(narrow), narrow, facts).experiment is not None


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


# ---------------------------------------------------------------------------
# levels re-planned from the live demand (item 52)
# ---------------------------------------------------------------------------


def _tick(
    cfg: MpcConfig,
    exp: dict[str, Any],
    ts: float,
    *,
    ran: dict[str, float] | None = None,
    prev: dict[str, float] | None = None,
    **demand: float,
) -> ident.Advance:
    """One :func:`ident.advance` on a good tick whose solver demand is ``demand``.

    ``ran`` is the override the experiment itself had in force on the tick the want was
    computed against, and defaults to its anchor -- a low tick under ``above``, where
    nothing is taken out of the want. ``prev`` is what was actually on the fan then and
    defaults to ``ran``; pass it higher to model the supervisor's floor lifting the fan
    above the experiment's own level."""
    facts = good_facts(cfg, ts=ts)
    ran = dict(exp["plan_base"]) if ran is None else ran
    prev_pwm = {**facts.prev, **(ran if prev is None else prev)}
    exp = {**exp, "prev_overrides": {**dict(exp.get("prev_overrides") or {}), **ran}}
    return ident.advance(
        exp, cfg, dataclasses.replace(facts, demand={**facts.demand, **demand}, prev=prev_pwm)
    )


def test_a_rise_in_the_demand_wins_over_the_experiment_plan():
    cfg = ident_cfg()  # above, amplitude 0.15, base 0.5
    exp = _running(cfg)
    out = _tick(cfg, exp, 1001.0, fa1=0.62)
    assert out.experiment is not None
    e = out.experiment
    assert e["plan_base"]["fa1"] == pytest.approx(0.62)  # the anchor followed the demand
    assert e["levels"]["fa1"] == pytest.approx([0.62, 0.77])
    assert min(e["levels"]["fa1"]) >= 0.62 - TOL  # never below what the solver asks for
    assert e["overrides"]["fa1"] >= 0.62 - TOL
    assert e["base"]["fa1"] == pytest.approx(0.5)  # the start base is still reported
    assert e["plan_base"]["fa2"] == pytest.approx(0.5)  # a channel of its own


def test_the_experiments_own_level_is_taken_out_of_the_want_before_it_is_followed():
    """The DAS MPC's move penalty pulls its want toward what is on the fan, which during
    an experiment is the experiment's own level; following that raw would ratchet."""
    cfg = ident_cfg()  # above, amplitude 0.15, base 0.5
    exp = _running(cfg)
    high = {"fa1": 0.65, "fa2": 0.65}  # the level the tick actually ran at
    echo = _tick(cfg, exp, 1001.0, ran=high, fa1=0.62).experiment
    assert echo is not None
    assert echo["plan_base"]["fa1"] == pytest.approx(0.5)  # 0.62 - (0.65 - 0.5) < 0.5: the echo
    real = _tick(cfg, exp, 1001.0, ran=high, fa1=0.9).experiment
    assert real is not None  # a want above the level is a real need, minus the same echo
    assert real["plan_base"]["fa1"] == pytest.approx(0.75)
    # and the very next low tick follows the want exactly, nothing subtracted
    clean = _tick(cfg, echo, 1002.0, fa1=0.62).experiment
    assert clean is not None and clean["plan_base"]["fa1"] == pytest.approx(0.62)


def test_a_fan_the_solver_itself_floored_up_is_demand_and_not_an_echo():
    """The echo is bounded by the experiment's own level, so a fan lifted above it by the
    supervisor's floor still reads as demand -- otherwise the anchor would stick where the
    floor found it and the plan would stop tracking the solver at all."""
    cfg = ident_cfg()  # above, amplitude 0.15, base 0.5
    exp = _running(cfg)
    low, on_fan = {"fa1": 0.5, "fa2": 0.5}, {"fa1": 0.7, "fa2": 0.7}
    out = _tick(cfg, exp, 1001.0, ran=low, prev=on_fan, fa1=0.7).experiment
    assert out is not None and out["plan_base"]["fa1"] == pytest.approx(0.7)
    # the same from a high level: only the level's own 0.15 comes off, not the floor's
    high, up = {"fa1": 0.65, "fa2": 0.65}, {"fa1": 0.8, "fa2": 0.8}
    out = _tick(cfg, exp, 1001.0, ran=high, prev=up, fa1=0.8).experiment
    assert out is not None and out["plan_base"]["fa1"] == pytest.approx(0.65)


def test_a_ratchet_cannot_build_up_over_a_run_of_high_ticks():
    """The want of every tick echoes the fan (slope 1, the worst case): the anchor holds."""
    cfg = ident_cfg()
    exp = _running(cfg)
    ts = 1000.0
    for _ in range(40):
        ts += cfg.dt
        ran = dict(exp["overrides"])  # what the tick ran at
        out = _tick(cfg, exp, ts, ran=ran, **{ch: ran[ch] for ch in exp["channels"]})
        assert out.experiment is not None
        exp = out.experiment
        assert exp["plan_base"] == pytest.approx(dict.fromkeys(("fa1", "fa2"), 0.5))


def test_the_anchor_rises_at_once_and_falls_only_at_a_switch():
    cfg = ident_cfg()
    exp = _running(cfg)
    up = _tick(cfg, exp, 1001.0, fa1=0.62).experiment
    assert up is not None
    # inside the hold the level stays put, so the excitation of that hold is preserved
    ts, held = 1001.0, up
    while True:
        ts += cfg.dt
        out = _tick(cfg, held, ts, fa1=0.30).experiment
        assert out is not None
        if (out["phase"], out["level"]) != (held["phase"], held["level"]):
            break
        assert out["plan_base"]["fa1"] == pytest.approx(0.62)
        held = out
    assert out["plan_base"]["fa1"] == pytest.approx(0.62 - cfg.d_pwm_max)  # at most that, at once
    assert out["plan_base"]["fa1"] > 0.30  # and it walks down, it does not jump


def test_a_stale_peak_is_released_but_never_below_the_frozen_base():
    cfg = ident_cfg()
    exp = _running(cfg)  # base 0.5
    peaked = _tick(cfg, exp, 1001.0, fa1=0.9, fa2=0.9).experiment
    assert peaked is not None and peaked["plan_base"]["fa1"] == pytest.approx(0.9)
    ts = 1001.0
    for _ in range(80):  # the burst is over: the demand is back at the start base
        ts += cfg.dt
        out = _tick(cfg, peaked, ts, fa1=0.5, fa2=0.5)
        if out.experiment is None:
            break
        peaked = out.experiment
        assert peaked["plan_base"]["fa1"] >= 0.5 - TOL  # never below the frozen start base
    assert peaked["plan_base"]["fa1"] == pytest.approx(0.5)  # released, not pinned at the peak


def test_a_missing_demand_or_prev_keeps_the_anchor():
    cfg = ident_cfg()
    exp = _running(cfg)
    up = _tick(cfg, exp, 1001.0, fa1=0.62).experiment
    assert up is not None
    facts = good_facts(cfg, ts=1003.0)
    blind = ident.advance(up, cfg, dataclasses.replace(facts, demand={})).experiment
    assert blind is not None and blind["plan_base"]["fa1"] == pytest.approx(0.62)
    deaf = ident.advance(up, cfg, dataclasses.replace(facts, prev={})).experiment
    assert deaf is not None and deaf["plan_base"]["fa1"] == pytest.approx(0.62)


def test_a_held_sibling_follows_the_demand_too():
    cfg = ident_cfg()  # phases: [fa1, fa2], [fa1], [fa2]
    exp = _running(cfg)
    out = _tick(cfg, exp, 1001.0, fa2=0.7).experiment
    assert out is not None
    single = ident.levels_at(out, out["phases"][1]["start_s"] + 1.0)
    assert single["overrides"]["fa2"] == pytest.approx(0.7)  # held at its anchor, not at 0.5


def test_a_replanned_level_stays_inside_the_band():
    cfg = ident_cfg()
    exp = _running(cfg)
    out = _tick(cfg, exp, 1001.0, fa1=1.4, fa2=0.95).experiment  # 1.4: saturated demand
    assert out is not None
    assert out["levels"]["fa1"] == pytest.approx([cfg.pwm_max, cfg.pwm_max])  # no excitation left
    assert out["levels"]["fa2"] == pytest.approx([0.95, cfg.pwm_max])  # squeezed step
    for u in out["overrides"].values():
        assert cfg.pwm_min - TOL <= u <= cfg.pwm_max + TOL


def test_symmetric_dips_at_most_the_amplitude_below_the_live_demand():
    cfg = ident_cfg(ident_levels="symmetric", ident_amplitude=0.2)
    exp = ident.start(cfg, good_facts(cfg), "channel", "fb1")
    out = _tick(cfg, exp, 1001.0, fb1=0.6).experiment
    assert out is not None
    assert out["levels"]["fb1"] == pytest.approx([0.4, 0.8])
    assert min(out["levels"]["fb1"]) >= 0.6 - cfg.ident_amplitude - TOL
    assert ident.dip_below_solver(cfg) == pytest.approx(cfg.ident_amplitude)
    assert ident.dip_below_solver(ident_cfg()) == 0.0


def test_without_replan_the_levels_stay_frozen_at_the_start_base():
    cfg = ident_cfg(ident_replan=False)
    exp = _running(cfg)
    assert exp["replan"] is False
    out = _tick(cfg, exp, 1001.0, fa1=0.62, fa2=0.9).experiment
    assert out is not None
    assert out["plan_base"] == out["base"] == dict.fromkeys(("fa1", "fa2"), 0.5)
    assert out["levels"]["fa1"] == pytest.approx([0.5, 0.65])
    # the low level stays under the demand: the channel is held back (the Zero W behaviour)
    assert min(out["levels"]["fa1"]) < 0.62


@pytest.mark.parametrize(("reason", "patch"), ABORTS)
def test_replanning_moves_no_abort(reason, patch):
    """Same tick, same aborts: the re-plan happens after the abort checks, never before."""
    frozen_cfg = ident_cfg(ident_replan=False)
    live_cfg = ident_cfg()
    frozen, live = _running(frozen_cfg), _running(live_cfg)
    facts = patch(good_facts(live_cfg, ts=1001.0))
    facts = dataclasses.replace(facts, demand={**facts.demand, "fa1": 0.8, "fa2": 0.8})
    a = ident.advance(frozen, frozen_cfg, facts)
    b = ident.advance(live, live_cfg, facts)
    assert (a.result, a.reason) == (b.result, b.reason) == (ident.RESULT_ABORTED, reason)


def test_a_replanned_level_is_never_below_the_frozen_one():
    frozen_cfg, live_cfg = ident_cfg(ident_replan=False), ident_cfg()
    frozen, live = _running(frozen_cfg), _running(live_cfg)
    ts = 1000.0
    for i in range(60):
        ts += frozen_cfg.dt
        want = 0.4 + 0.01 * i  # a demand that rises, then falls back
        if i > 30:
            want = 0.4 + 0.01 * (60 - i)
        a = _tick(frozen_cfg, frozen, ts, fa1=want, fa2=want)
        b = _tick(live_cfg, live, ts, fa1=want, fa2=want)
        assert a.result == b.result and a.reason == b.reason
        if a.experiment is None:
            break
        frozen, live = a.experiment, b.experiment
        assert live["phase"] == frozen["phase"] and live["level"] == frozen["level"]
        for ch, u in live["overrides"].items():
            assert u >= frozen["overrides"][ch] - TOL
            assert u >= min(want, live_cfg.pwm_max) - TOL  # never below the solver


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
        "demand": st.floats(0.0, 1.5),
        "prev": st.floats(0.0, 1.5),
        "trusted": st.booleans(),
    }
)


@settings(max_examples=60, deadline=None)
@given(
    seq=st.lists(_facts_strategy, min_size=1, max_size=60),
    levels=st.sampled_from(["above", "symmetric"]),
    replan=st.booleans(),
)
def test_random_tick_sequences_never_raise_and_keep_levels_in_bounds(seq, levels, replan):
    cfg = ident_cfg(ident_levels=levels, ident_amplitude=0.2, ident_replan=replan)
    exp: dict[str, Any] | None = ident.start(cfg, good_facts(cfg, pwm=0.5), "group", "front")
    ts = 1000.0
    anchor = 0.5  # the model of the anchor: the de-biased demand up at once, down at a switch
    for item in seq:
        if exp is None:
            break
        ts += item["dt"]
        facts = good_facts(cfg, ts=ts, pwm=item["pwm"], mode=item["mode"], applied=item["applied"])
        facts = dataclasses.replace(
            facts,
            demand=dict.fromkeys(cfg.channels, item["demand"]),
            prev=dict.fromkeys(cfg.channels, item["prev"]),
        )
        facts = _patched(facts, "estimates", "a1", item["t_c"], "t_c")
        facts = _patched(facts, "zones", "za", item["trusted"], "trusted")
        was = (exp["phase"], exp["level"])
        ran = dict(exp["prev_overrides"])  # the level the want of this tick was seen at
        out = ident.advance(exp, cfg, facts)
        if out.experiment is None:
            assert out.result in (ident.RESULT_ABORTED, ident.RESULT_COMPLETED)
            if out.result == ident.RESULT_COMPLETED:
                assert item["mode"] == "auto" and item["trusted"]
            break
        exp = out.experiment
        if replan:
            own = ran.get("fa1")
            echo = 0.0 if own is None else max(0.0, min(item["prev"], own) - anchor)
            wanted = min(max(item["demand"] - echo, cfg.pwm_min), cfg.pwm_max)
            if wanted > anchor:
                anchor = wanted
            elif (exp["phase"], exp["level"]) != was:  # a switch: down by at most d_pwm_max
                anchor = max(wanted, 0.5, anchor - cfg.d_pwm_max)
        assert item["mode"] in ("auto", "saturated")
        assert item["t_c"] <= 45.0 + cfg.ident_max_over_c + TOL  # soft envelope
        assert item["t_c"] + 2.0 < 50.0 - cfg.ident_abort_below_limit_c + TOL  # absolute
        json.dumps(exp, allow_nan=False)
        assert exp["plan_base"] == pytest.approx(dict.fromkeys(("fa1", "fa2"), anchor))
        assert anchor >= 0.5 - TOL  # never below the frozen base of the start
        for u in exp["overrides"].values():
            assert cfg.pwm_min <= u <= cfg.pwm_max
            lo = anchor - 0.2 if levels == "symmetric" else anchor
            assert lo - TOL <= u <= min(anchor + 0.2, cfg.pwm_max) + TOL
            # a tick the experiment ran at its anchor carries an honest want: the plan
            # follows it whole (symmetric: to within the owner-accepted dip)
            if replan and min(item["prev"], ran.get("fa1", item["prev"])) <= anchor + TOL:
                assert u >= min(item["demand"], cfg.pwm_max) - ident.dip_below_solver(cfg) - TOL


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


def test_a_group_runs_its_phases_then_completes_and_keeps_the_solvers_own_level():
    """Item 112. The solver ran on every tick of the experiment -- the levels are an
    override applied after it -- so at the end its integrator already holds the command
    it would have given without one. The channels are therefore **not** released: a
    release would drop that integrator entry and re-initialise the solver bumplessly at
    the PWM the experiment left on the fan."""
    rig = started_rig()
    phases_seen = set()
    ended_at = None
    for i in range(200):
        rig.ticks(1)
        st_ = rig.status
        assert {"fa1", "fa2"} <= set(rig.loop.state.integrator)  # never dropped
        if st_["running"]:
            phases_seen.add(st_["phase"])
            continue
        ended_at = i
        break
    assert ended_at is not None
    assert phases_seen == {0, 1, 2}
    assert rig.status["last_result"] == "completed"
    assert not {"fa1", "fa2"} & rig.sup.plan_tick().released
    (r,) = rig.ticks(1)
    assert "experiment" not in r.cmd.diagnostics["supervisor"]
    assert {"fa1", "fa2"} <= set(rig.loop.state.integrator)


def test_an_experiment_stopped_on_its_high_level_hands_back_the_solvers_level():
    """Item 112, the case the old release got wrong: a run that ends while the fan is a
    step above the anchor. The solver's own want is the anchor; before this item the
    release re-initialised it at the level on the fan and the enclosure stayed loud."""
    rig = started_rig()
    high = None
    for _ in range(40):
        rig.ticks(1)
        st_ = rig.status
        if st_["level"] == "high" and rig.pwm["fa1"] > st_["plan_base"]["fa1"] + 0.1:
            high = dict(rig.pwm)
            break
    assert high is not None
    anchor = dict(rig.status["plan_base"])
    rig.sup.submit(Ident("stop"))
    assert rig.status["last_abort_reason"] == "stop"
    (r,) = rig.ticks(1)
    want = r.mpc_cmd.diagnostics["target_pwm"]
    for ch in ("fa1", "fa2"):
        # what the solver asks for, not what the experiment left on the fan
        assert want[ch] == pytest.approx(anchor[ch], abs=0.02)
        assert want[ch] < high[ch] - 0.1
        # and nothing steps: the rate limit walks it back at d_pwm_max
        assert r.cmd.pwm[ch] >= high[ch] - rig.cfg.d_pwm_max - TOL
    # the walk back is monotone and lands on the solver's own level
    for _ in range(6):
        (r,) = rig.ticks(1)
    for ch in ("fa1", "fa2"):
        assert r.cmd.pwm[ch] == pytest.approx(anchor[ch], abs=0.02)


def _warming_run(replan: bool) -> tuple[Rig, int, list[float]]:
    """An experiment while zone za's air warms: the solver wants more and more of the
    front group. Returns the rig, the channel-ticks the experiment held a channel below
    the solver's own command, and the anchors it planned around."""
    cfg = ident_cfg(ident_replan=replan, ident_max_duration_s=300.0)
    rig = Rig(cfg)
    rig.ticks(8)
    rig.sup.submit(Ident("start", group="front"))
    held, anchors = 0, []
    for _ in range(140):
        for name in ("air_a", "air_a2"):
            rig.temps[name] = float(rig.temps.get(name) or 35.0) + 0.02
        (r,) = rig.ticks(1)
        if not rig.status["running"]:
            break
        anchors.append(rig.status["plan_base"]["fa1"])
        held += sum(r.cmd.pwm[ch] < r.mpc_cmd.pwm[ch] - TOL for ch in ("fa1", "fa2"))
    return rig, held, anchors


def test_a_rising_demand_is_followed_and_never_held_back_through_the_loop():
    rig, held, anchors = _warming_run(replan=True)
    assert rig.status["running"]  # the envelope never tripped: the drives kept their cooling
    assert held == 0  # no tick put less on a fan than the solver asked for
    assert anchors[-1] > anchors[0] + 0.3  # the levels followed the demand up
    assert rig.status["levels"]["fa1"][1] <= rig.cfg.pwm_max + TOL


def test_without_replan_the_experiment_holds_the_channel_below_the_solver():
    """The behaviour item 52 replaces, still selectable for the Zero W."""
    rig, held, anchors = _warming_run(replan=False)
    assert set(anchors) == {0.5}  # the base stayed frozen at the start
    assert held > 50  # and the fans ran below what the solver wanted, tick after tick


def _load_run(replan: bool) -> tuple[Rig, list[float], list[float]]:
    """Zone air the front group actually cools, with a heat load that comes and goes: the
    solver wants more while it lasts and no more than before once it is gone. Returns the
    anchors the experiment planned around and what it commanded on fa1."""
    cfg = ident_cfg(ident_replan=replan, ident_max_duration_s=900.0)
    rig = Rig(cfg)
    rig.ticks(8)
    rig.sup.submit(Ident("start", group="front"))
    air, anchors, cmds = 35.0, [], []
    for i in range(300):
        (r,) = rig.ticks(1)
        if not rig.status["running"]:
            break
        anchors.append(rig.status["plan_base"]["fa1"])
        cmds.append(r.cmd.pwm["fa1"])
        fans = (r.cmd.pwm["fa1"] + r.cmd.pwm["fa2"]) / 2.0
        air = max(33.0, air + (0.06 if 20 <= i < 70 else 0.0) - 0.15 * (fans - 0.5))
        for name in ("air_a", "air_a2"):
            rig.temps[name] = air
    return rig, anchors, cmds


def test_a_transient_peak_does_not_pin_the_channel_for_the_rest_of_the_experiment():
    """The anchor follows a load up at once and walks back down once it is gone, so the
    fans do not run at the peak of a spent excursion for the rest of the duration."""
    rig, anchors, cmds = _load_run(replan=True)
    assert rig.status["running"]
    base = rig.status["base"]["fa1"]
    assert max(anchors) > base + 0.15  # the load was followed up
    assert min(anchors) >= base - TOL  # never below the frozen base of the start
    assert anchors[-1] == pytest.approx(base)  # and released again, not pinned at the peak
    _, frozen_anchors, frozen_cmds = _load_run(replan=False)
    assert set(frozen_anchors) == {0.5}  # the frozen plan never moved at all
    # so the noise the following cost is paid back: with the load gone the re-planned run
    # commands no more than the frozen one
    assert statistics.fmean(cmds[-50:]) <= statistics.fmean(frozen_cmds[-50:]) + TOL


def _abort_run(replan: bool) -> tuple[int, str | None] | None:
    """Zone air that keeps warming (so the solver keeps wanting more of the front group)
    over drives that only cool with the PWM that actually reaches the fans. Returns the
    tick the experiment ended on and why, or ``None`` if it ran the whole way."""
    cfg = ident_cfg(ident_replan=replan, ident_max_duration_s=900.0)
    rig = Rig(cfg)
    rig.ticks(8)
    rig.sup.submit(Ident("start", group="front"))
    air, drive = 35.0, 37.5
    for i in range(400):
        (r,) = rig.ticks(1)
        if not rig.status["running"]:
            return i, rig.status["last_abort_reason"]
        fans = (r.cmd.pwm["fa1"] + r.cmd.pwm["fa2"]) / 2.0
        air += 0.03
        drive = max(30.0, drive + 0.03 - 0.12 * (fans - 0.5))
        for name in ("air_a", "air_a2"):
            rig.temps[name] = air
        for name in ("prox_a1", "prox_a1b", "prox_a2"):
            rig.temps[name] = drive
    return None


def test_an_experiment_that_follows_the_demand_ends_later_or_not_at_all_never_earlier():
    """The abort decision of a tick does not move (test_replanning_moves_no_abort pins that
    on the tick itself). Across ticks it does, and in one direction only: the channel the
    experiment followed up cools its own bay, so an excursion the frozen plan ended on the
    envelope runs to completion instead. Fewer envelope aborts are the intended outcome."""
    frozen = _abort_run(replan=False)
    assert frozen is not None and frozen[1] == "envelope:a1"
    live = _abort_run(replan=True)
    assert live is None or live[0] > frozen[0]


def test_the_floor_follows_the_running_experiment_not_a_config_rebuilt_under_it():
    """``ident_replan`` of the running experiment decides, so a config change cannot take
    the floor away from an experiment that was planned with it (or add one to a frozen
    plan): the rule holds for the whole of the experiment that is running."""
    rig = started_rig(ident_cfg(ident_replan=True))
    rig.ticks(3)
    plan = rig.sup.plan_tick()
    assert plan.experiment is not None and plan.experiment["replan"] is True
    frozen_cfg = ident_cfg(ident_replan=False)
    mpc_cmd = rig.results[-1].mpc_cmd
    floor = supervisor_mod._experiment_floor(mpc_cmd, plan, frozen_cfg)
    assert set(floor) == set(plan.experiment["overrides"])
    for ch, value in floor.items():
        assert value == pytest.approx(mpc_cmd.pwm[ch])  # above: the dip is nothing
    stale = dataclasses.replace(plan, experiment={**plan.experiment, "replan": False})
    assert supervisor_mod._experiment_floor(mpc_cmd, stale, ident_cfg()) == {}


def test_the_dip_of_a_running_experiment_comes_from_its_own_levels():
    cfg = ident_cfg(ident_levels="symmetric", ident_amplitude=0.2)
    exp = ident.start(cfg, good_facts(cfg, pwm=0.5), "channel", "fb1")
    assert ident.planned_dip(exp, "fb1", cfg) == pytest.approx(0.2)
    # the live config says above, the running experiment still dips: its own plan decides
    assert ident.planned_dip(exp, "fb1", ident_cfg()) == pytest.approx(0.2)
    # a level clamped at pwm_min shortens the dip, which only raises the floor
    low = ident.start(cfg, good_facts(cfg, pwm=0.25), "channel", "fb1")
    low["levels"]["fb1"][0] = cfg.pwm_min
    assert ident.planned_dip(low, "fb1", cfg) == pytest.approx(0.25 - cfg.pwm_min)
    assert ident.planned_dip(None, "fb1", cfg) == pytest.approx(0.2)  # no plan: the config
    assert ident.planned_dip({"levels": {}}, "fb1", ident_cfg()) == 0.0


def test_the_demand_is_read_out_of_the_diagnostics_only_for_a_running_experiment():
    """The two maps only a running experiment reads are not copied on every DAS tick."""
    rig = Rig(ident_cfg())
    (r,) = rig.ticks(1)
    idle = ident.facts_from_tick(r.mpc_cmd, ts=1.0, applied=True)
    assert idle.demand == {} and idle.prev == {}
    live = ident.facts_from_tick(r.mpc_cmd, ts=1.0, applied=True, with_demand=True)
    assert live.demand == r.mpc_cmd.diagnostics["target_pwm"]
    assert live.prev == r.mpc_cmd.diagnostics["prev_pwm"]
    rig.ticks(7)
    rig.sup.submit(Ident("start", group="front"))
    rig.ticks(2)
    assert rig.sup._ident_facts.demand  # the supervisor asks for them on experiment ticks
    frozen = Rig(ident_cfg(ident_replan=False))
    frozen.ticks(8)
    frozen.sup.submit(Ident("start", group="front"))
    frozen.ticks(2)
    assert frozen.status["running"]  # a frozen plan never reads them, so never copies
    assert frozen.sup._ident_facts.demand == {} and frozen.sup._ident_facts.prev == {}


# ---------------------------------------------------------------------------
# how far a channel can move the PE monitor from where it sits (item 110)
# ---------------------------------------------------------------------------


def test_the_relative_airflow_swing_a_telegraph_reaches_is_the_pe_arithmetic():
    """``pe_min`` is relative, so what a telegraph can reach depends on where the channel
    sits. ``rel_swing`` is that number, on the fan's own curve and with the PE monitor's
    own scale floor, so its square is exactly the ``pe_diag`` entry the monitor would
    report for a group of this one channel (section 8 item 110)."""
    cfg = ident_cfg()  # fa1: p12, deadband 0.1, exponent 1.0
    deadband, exponent = 0.1, 1.0

    def by_hand(lo: float, hi: float) -> float:
        p_lo = thermal.phi(lo, deadband, exponent)
        p_hi = thermal.phi(hi, deadband, exponent)
        return 0.5 * (p_hi - p_lo) / max(0.5 * (p_hi + p_lo), thermal.PE_SCALE_FLOOR)

    for base in (0.2, 0.35, 0.5, 0.8):
        lo, hi = base, base + cfg.ident_amplitude
        assert ident.rel_swing(cfg, "fa1", lo, hi) == pytest.approx(by_hand(lo, hi))
    # the plan's own closed form: above needs A >= 0.576 (u - deadband) to clear the bound
    want = math.sqrt(thermal.PE_MIN)
    for base in (0.2, 0.3, 0.45, 0.6):
        a = 2.0 * want * (base - deadband) / (1.0 - want)
        assert ident.rel_swing(cfg, "fa1", base, base + a) == pytest.approx(want, abs=1e-6)
    # monotone in the amplitude and falling with the base, which is the whole point
    swings = [ident.rel_swing(cfg, "fa1", 0.5, 0.5 + a) for a in (0.05, 0.15, 0.3)]
    assert swings == sorted(swings)
    bases = [ident.rel_swing(cfg, "fa1", u, u + 0.15) for u in (0.2, 0.4, 0.6, 0.8)]
    assert bases == sorted(bases, reverse=True)
    # a level the rail would eat does not move air: the swing is read off the clamped pair
    assert ident.rel_swing(cfg, "fa1", 0.95, 1.3) == pytest.approx(
        ident.rel_swing(cfg, "fa1", 0.95, cfg.pwm_max)
    )


def test_excitation_names_the_channels_that_cannot_reach_the_pe_bound():
    cfg = ident_cfg()
    levels = {"fa1": [0.2, 0.35], "fa2": [0.5, 0.65]}
    out = ident.excitation(cfg, levels)
    assert out["fa1"]["excitable"] is True and out["fa2"]["excitable"] is False
    for ch, entry in out.items():
        assert entry["pe_reach"] == pytest.approx(entry["rel_swing"] ** 2)
        assert entry["pe_min"] == thermal.PE_MIN
        assert entry["excitable"] is (entry["pe_reach"] > thermal.PE_MIN)
        assert entry["rel_swing"] == pytest.approx(ident.rel_swing(cfg, ch, *levels[ch]))
    assert ident.unexcitable(cfg, levels) == ["fa2"]
    assert ident.excitation(cfg, {"fa1": "nonsense", "fa2": [0.5]}) == {}


def test_a_running_experiment_publishes_what_its_channels_can_reach():
    """The fixture parks the front group at 0.5 PWM, where a 0.15 step reaches 0.158 of
    relative airflow variation against the 0.224 the rule needs. That is published, so a
    zone waiting on the PE gate is visible rather than silently pending."""
    rig = started_rig()
    st_ = rig.status
    assert sorted(st_["unexcitable"]) == ["fa1", "fa2"]
    for ch in ("fa1", "fa2"):
        entry = st_["excitation"][ch]
        assert entry["excitable"] is False
        assert entry["rel_swing"] == pytest.approx(0.1579, abs=1e-3)
    assert rig.sup.snapshot().extra["experiment"]["unexcitable"] == st_["unexcitable"]
    # and it is gone with the experiment
    rig.sup.submit(Ident("stop"))
    assert rig.status["excitation"] == {} and rig.status["unexcitable"] == []


def test_ident_require_excitable_refuses_a_start_that_cannot_inform_the_pe_gate():
    """``not_excitable:<ch>``, the way ``band:`` and ``saturated:`` refuse -- off by
    default, because such a run still informs the ``E`` split and the bays."""
    rig = started_rig()  # the default: 0.5 PWM is not excitable and the start went ahead
    assert rig.status["running"] and rig.status["unexcitable"]
    rig.sup.submit(Ident("stop"))

    strict = Rig(ident_cfg(ident_require_excitable=True))
    strict.ticks(8)
    with pytest.raises(IntentConflict, match=r"not_excitable:fa1, not_excitable:fa2"):
        strict.sup.submit(Ident("start", group="front"))
    # the same config with the fans parked low: the telegraph reaches the bound and runs
    facts = strict.sup._ident_facts
    low = dataclasses.replace(facts, pwm=dict.fromkeys(strict.cfg.channels, 0.2))
    assert "not_excitable:fa1" not in ident.check_start(
        strict.cfg, strict.sup._ident_tracker, low, "group", "front", human_control=False
    )
    # a channel the band already refuses is not also reported unexcitable: one reason each
    high = dataclasses.replace(facts, pwm=dict.fromkeys(strict.cfg.channels, 0.95))
    reasons = ident.check_start(
        strict.cfg, strict.sup._ident_tracker, high, "group", "front", human_control=False
    )
    assert "band:fa1" in reasons and "not_excitable:fa1" not in reasons


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
def test_every_human_intent_aborts_and_the_solver_keeps_its_integrator(index):
    rig = started_rig()
    rig.ticks(3)
    intent = _all_intents(rig.cfg)[index]
    with contextlib.suppress(IntentConflict, IntentInvalid):
        rig.sup.submit(intent)
    assert not rig.status["running"]
    assert rig.status["last_result"] == "aborted"
    assert rig.status["last_abort_reason"].startswith("human_intent:")
    plan = rig.sup.plan_tick()
    # item 112: the experiment's own channels are never released; a human override the
    # same intent then cleared is (``ClearOverride`` at index 5 has nothing to clear here)
    assert not {"fa1", "fa2"} & plan.released
    assert plan.experiment is None
    rig.ticks(1)
    assert {"fa1", "fa2"} <= set(rig.loop.state.integrator)


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


def test_a_start_between_plan_tick_and_record_tick_does_not_shift_the_levels():
    # item 20: the loop already holds the plan for the tick that follows, so that tick
    # cannot carry the overrides and offset 0 belongs to the one after it.
    plain = Rig(ident_cfg())
    last = plain.ticks(8)[-1].obs.ts
    plain.sup.submit(Ident("start", group="front"))
    assert plain.sup.experiment["start_ts"] == pytest.approx(last + plain.cfg.dt)

    shifted = Rig(ident_cfg())
    last = shifted.ticks(8)[-1].obs.ts
    plan = shifted.sup.plan_tick()  # the loop holds the plan for the next tick already
    shifted.sup.submit(Ident("start", group="front"))
    assert not plan.overrides  # that tick runs on the solver, it cannot carry the levels
    assert shifted.sup.experiment["start_ts"] == pytest.approx(last + 2 * shifted.cfg.dt)
    assert ident.levels_at(shifted.sup.experiment, 0.0) == {
        k: shifted.sup.experiment[k] for k in ("phase", "level", "overrides")
    }


def test_status_is_json_and_counts_time():
    rig = started_rig()
    rig.ticks(5)
    st_ = rig.status
    json.dumps(st_, allow_nan=False)
    assert st_["elapsed_s"] == pytest.approx(5.0)
    assert st_["remaining_s"] == pytest.approx(rig.cfg.ident_max_duration_s - 5.0)
    assert st_["group"] == "front" and st_["phases"] == 3 and st_["level"] in ("high", "low")
    assert st_["channels"] == ["fa1", "fa2"] and st_["channel"] is None
    assert st_["replan"] is True and st_["plan_base"] == st_["base"]
    for ch, (lo, hi) in st_["levels"].items():
        assert hi - lo == pytest.approx(rig.cfg.ident_amplitude)
        assert st_["plan_base"][ch] == pytest.approx(lo)


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
