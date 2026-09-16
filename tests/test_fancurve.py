"""Online PWM -> RPM curve fit per fan model (PROJECT.md section 8 item 14).

``control/fancurve.py`` is pure: settled ``(pwm, rpm)`` pairs go into bins per fan model,
a grid fit runs every ``fan_curve_refit_s``, and a fit that passes its acceptance rules
lands in ``solver_memory["fan_curves"]`` -- the model store's own section, in the shape
it already validates. From there the thermal model and the DAS MPC plan on it instead of
the ``fan_models`` entry. Off by default: nothing reads a curve without
``mpc.fan_curve_online``.
"""

from __future__ import annotations

import dataclasses
import json
import math
from typing import Any

import pytest

from aqua_bridge.control import fancurve, thermal
from aqua_bridge.control.mpc import step
from aqua_bridge.model import ConfigError, MpcConfig, MpcState
from das_fixtures import das_cfg, das_mapping, das_obs, default_temps

TRUTH = {"p12": (2100.0, 0.08, 0.85), "p14": (1400.0, 0.24, 1.2)}


def online_cfg(**changes: Any) -> MpcConfig:
    base = {"fan_curve_online": True, "fan_curve_settle_s": 2.0, "fan_curve_refit_s": 10.0}
    return das_cfg(**{**base, **changes})


def truth_rpm(cfg: MpcConfig, ch: str, u: float) -> float:
    rpm_max, deadband, exponent = TRUTH[cfg.fans[ch].model]
    return rpm_max * thermal.phi(u, deadband, exponent)


def feed(
    cfg: MpcConfig,
    duties: list[float],
    *,
    ticks_per_duty: int = 8,
    noise: float = 0.0,
    memory: Any = None,
    t0: float = 0.0,
) -> tuple[Any, dict[str, dict[str, float]]]:
    """Hold each duty for ``ticks_per_duty`` ticks on every channel and feed the truth's
    speed back, the way a settled fan reports it."""
    ts = t0
    curves: dict[str, dict[str, float]] = {}
    k = 0
    for duty in duties:
        for _ in range(ticks_per_duty):
            u = dict.fromkeys(cfg.channels, duty)
            rpm = {
                ch: truth_rpm(cfg, ch, duty) + noise * math.sin(0.7 * k + len(ch))
                for ch in cfg.channels
            }
            out = fancurve.update(memory, cfg, u=u, rpm=rpm, ts=ts)
            memory, curves = out.memory, out.curves
            ts += cfg.dt
            k += 1
    return memory, curves


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------


def test_fan_curve_keys_have_defaults_and_rules(das_example_cfg: MpcConfig, cfg: MpcConfig) -> None:
    assert das_example_cfg.fan_curve_online is False
    assert (
        das_example_cfg.fan_curve_settle_s,
        das_example_cfg.fan_curve_refit_s,
        das_example_cfg.fan_curve_max_rmse_frac,
        das_example_cfg.fan_curve_max_age_s,
    ) == (30.0, 600.0, 0.05, 3600.0)
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        dataclasses.replace(cfg, fan_curve_online=True)
    for key, bad in (
        ("fan_curve_settle_s", -1.0),
        ("fan_curve_refit_s", 0.0),
        ("fan_curve_max_rmse_frac", 0.0),
        ("fan_curve_max_rmse_frac", 1.5),
        ("fan_curve_max_age_s", 599.0),  # below fan_curve_refit_s: never re-confirmable
    ):
        with pytest.raises(ConfigError, match=key):
            dataclasses.replace(das_example_cfg, **{key: bad})


# ---------------------------------------------------------------------------
# the fit
# ---------------------------------------------------------------------------


def test_a_swept_fan_is_identified_per_model() -> None:
    cfg = online_cfg()
    _, curves = feed(cfg, [0.2, 0.35, 0.5, 0.65, 0.8, 0.95])
    assert set(curves) == {"p12", "p14"}
    for model, (rpm_max, deadband, exponent) in TRUTH.items():
        got = curves[model]
        assert got["rpm_max"] == pytest.approx(rpm_max, rel=0.05)
        assert got["deadband"] == pytest.approx(deadband, abs=0.05)
        assert got["exponent"] == pytest.approx(exponent, abs=0.1)


def test_one_duty_is_never_enough_to_fit() -> None:
    """A regulator that parks the fans gives one bin: no span, no fit, the configured
    curve stays in force."""
    cfg = online_cfg()
    memory, curves = feed(cfg, [0.6], ticks_per_duty=60)
    assert curves == {}
    summary = fancurve.summary(memory, cfg, None)
    assert summary["models"]["p12"]["source"] == "config"
    assert "bins" in (summary["models"]["p12"]["rejected"] or "")


def test_a_noisy_tachometer_is_refused_by_the_residual() -> None:
    cfg = online_cfg(fan_curve_max_rmse_frac=0.01)
    _, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0], noise=300.0)
    assert curves == {}


def test_a_ramping_command_contributes_nothing() -> None:
    """Every tick a different duty: nothing is ever settled for fan_curve_settle_s."""
    cfg = online_cfg(fan_curve_settle_s=5.0)
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0], ticks_per_duty=1)
    assert curves == {}
    bins = memory["models"]["p12"]["bins"]
    assert sum(row[0] for row in bins) == 0


def test_bins_stay_bounded_and_follow_a_fan_that_changes() -> None:
    cfg = online_cfg()
    duties = [0.2, 0.4, 0.6, 0.8, 1.0]
    memory, curves = feed(cfg, duties * 40, ticks_per_duty=4)
    bins = memory["models"]["p12"]["bins"]
    assert max(row[0] for row in bins) <= fancurve.BIN_CAPACITY
    before = curves["p12"]["rpm_max"]

    # the same fan model, now 20 % slower (a replaced or worn fan)
    TRUTH["p12"] = (before * 0.8, 0.08, 0.85)
    try:
        memory, curves = feed(cfg, duties * 300, ticks_per_duty=4, memory=memory, t0=10_000.0)
    finally:
        TRUTH["p12"] = (2100.0, 0.08, 0.85)
    assert curves["p12"]["rpm_max"] == pytest.approx(before * 0.8, rel=0.05)


def test_the_memory_is_plain_json_and_survives_a_round_trip() -> None:
    cfg = online_cfg()
    memory, curves = feed(cfg, [0.2, 0.5, 0.8, 1.0])
    assert curves
    again = json.loads(json.dumps(memory, allow_nan=False))
    assert again == memory
    u = dict.fromkeys(cfg.channels, 0.5)
    assert (
        fancurve.update(again, cfg, u=u, rpm={}, ts=1000.0).curves
        == fancurve.update(memory, cfg, u=u, rpm={}, ts=1000.0).curves
    )


@pytest.mark.parametrize(
    "broken",
    ["not a mapping", {}, {"v": 99}, {"v": 1, "fp": "other"}],
    ids=["scalar", "empty", "version", "fingerprint"],
)
def test_a_malformed_memory_starts_over(broken: Any) -> None:
    cfg = online_cfg()
    out = fancurve.update(broken, cfg, u=dict.fromkeys(cfg.channels, 0.5), rpm={}, ts=0.0)
    assert out.curves == {}
    fresh = fancurve.fresh_memory(cfg)
    assert out.memory["fp"] == fresh["fp"]
    for model, block in out.memory["models"].items():
        assert block["bins"] == fresh["models"][model]["bins"]
        assert block["fit"] is None


def test_a_changed_fan_model_map_starts_over() -> None:
    """The fingerprint covers channel -> fan model: re-cabling a channel to another fan
    model must not carry the old fan's bins over."""
    cfg = online_cfg()
    memory, curves = feed(cfg, [0.2, 0.5, 0.8, 1.0])
    assert curves
    kept = fancurve.update(memory, cfg, u=dict.fromkeys(cfg.channels, 0.5), rpm={}, ts=1000.0)
    assert set(kept.curves) == set(curves)

    fans = das_mapping()["fans"]
    fans["fb1"]["model"] = "p12"
    swapped = online_cfg(fans=fans)
    started_over = fancurve.update(
        memory, swapped, u=dict.fromkeys(swapped.channels, 0.5), rpm={}, ts=1000.0
    )
    assert started_over.curves == {}


def test_curve_pair_falls_back_to_the_config_for_anything_unusable() -> None:
    assert fancurve.curve_pair(None, 0.1, 1.0, 1800.0) == (0.1, 1.0, 1800.0)
    assert fancurve.curve_pair({"deadband": 0.2}, 0.1, 1.0, 1800.0) == (0.1, 1.0, 1800.0)
    bad = {"deadband": 0.7, "exponent": 1.0, "rpm_max": 1800.0}  # out of bounds
    assert fancurve.curve_pair(bad, 0.1, 1.0, 1800.0) == (0.1, 1.0, 1800.0)
    nan = {"deadband": float("nan"), "exponent": 1.0, "rpm_max": 1800.0}
    assert fancurve.curve_pair(nan, 0.1, 1.0, 1800.0) == (0.1, 1.0, 1800.0)
    good = {"deadband": 0.2, "exponent": 1.1, "rpm_max": 1500.0}
    assert fancurve.curve_pair(good, 0.1, 1.0, 1800.0) == (0.2, 1.1, 1500.0)


# ---------------------------------------------------------------------------
# the model reads it
# ---------------------------------------------------------------------------


def test_model_params_uses_a_fitted_curve_only_when_curves_are_given() -> None:
    cfg = das_cfg()
    curves = {"p12": {"rpm_max": 2000.0, "deadband": 0.3, "exponent": 0.7}}
    configured = thermal.model_params(cfg)
    assert configured.fan["fa1"] == (cfg.fan_models["p12"].deadband, cfg.fan_models["p12"].exponent)
    fitted = thermal.model_params(cfg, curves=curves)
    assert fitted.fan["fa1"] == (0.3, 0.7)
    assert fitted.fan["fb1"] == configured.fan["fb1"]  # p14 has no fitted curve


def test_model_use_rpm_keeps_the_configured_rpm_max_as_its_reference() -> None:
    """A fitted ``rpm_max`` must never normalise the tachometer: it is fitted to those
    same readings, so ``phi`` would reach 1 at full duty however slowly the fan turns and
    a fan that loses speed would look unchanged. ``fan_models.<m>.rpm_max`` is the fixed
    commissioned reference; the fit supplies the shape only."""
    cfg = das_cfg(model_use_rpm=True)
    st = thermal.cached_structure(cfg)
    spec = cfg.fan_models["p12"]
    u = dict.fromkeys(cfg.channels, 0.8)
    healthy = spec.rpm_max * thermal.phi(0.8, spec.deadband, spec.exponent)
    degraded = 0.7 * healthy  # a month of dust and bearing wear

    # the fit follows the worn fan, as it should -- and the identification still sees the
    # loss, because the tach branch divides by the configured rpm_max either way
    fitted = {
        "p12": {
            "rpm_max": 0.7 * spec.rpm_max,
            "deadband": spec.deadband,
            "exponent": spec.exponent,
        }
    }
    rpm = dict.fromkeys(cfg.channels, degraded)
    phis: dict[str, float] = {}
    for curves in (None, {}, fitted):
        phis = thermal._channel_phi(cfg, st, u, rpm, curves)
        assert phis["fa1"] == pytest.approx((degraded / spec.rpm_max) ** spec.exponent)
    healthy_phi = thermal._channel_phi(cfg, st, u, dict.fromkeys(cfg.channels, healthy), fitted)
    assert phis["fa1"] < 0.75 * healthy_phi["fa1"]

    # the fitted shape does reach the rpm branch's exponent
    shaped = {"p12": {"rpm_max": spec.rpm_max, "deadband": spec.deadband, "exponent": 1.4}}
    assert thermal._channel_phi(cfg, st, u, rpm, shaped)["fa1"] == pytest.approx(
        (degraded / spec.rpm_max) ** 1.4
    )


def test_step_publishes_the_fit_into_the_store_section_and_the_diagnostics() -> None:
    """End to end through ``step``: an accumulator that has seen a sweep puts its curves
    into ``solver_memory["fan_curves"]`` -- the model store's own section, which the
    persister writes and ``apply_seed`` reads back -- and says so in the diagnostics."""
    cfg = online_cfg(model_shadow=True)
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0])
    assert set(curves) == {"p12", "p14"}
    state = MpcState(solver_memory={"fan_fit": memory})
    cmd, state = step(das_obs(cfg, 0.0, pwm=0.5), cfg, state)
    assert state.solver_memory["fan_curves"] == curves
    assert curves["p12"]["rpm_max"] == pytest.approx(TRUTH["p12"][0], rel=0.05)
    summary = cmd.diagnostics["fan_curves"]
    assert summary["online"] is True
    assert summary["models"]["p12"]["source"] == "fit"
    assert summary["models"]["p12"]["rpm_max"] == curves["p12"]["rpm_max"]

    # and the curve is what the model plans on, not the fan_models entry
    params = thermal.model_params(cfg, curves=state.solver_memory["fan_curves"])
    assert params.fan["fa1"] == (curves["p12"]["deadband"], curves["p12"]["exponent"])


def test_an_online_curve_round_trips_through_the_model_store(tmp_path) -> None:
    """Item 14 with item 15's file: the section the online fit writes is the one the store
    already saves and validates, so a restart keeps the fitted curve."""
    from aqua_bridge import modelstore
    from aqua_bridge.control import persist

    cfg = dataclasses.replace(online_cfg(model_shadow=True), model_store_interval_s=1.0)
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0])
    state = MpcState(solver_memory={"fan_fit": memory})
    _, state = step(das_obs(cfg, 0.0, pwm=0.5), cfg, state)

    doc = modelstore.build_document(cfg, state.solver_memory, ts=0.0, wall=1_800_000_000.0)
    path = tmp_path / "model.json"
    path.write_text(json.dumps(doc), encoding="utf-8")
    result = modelstore.load(path, cfg, now_wall=1_800_000_060.0)
    assert result.source == "fresh"
    mem: dict[str, Any] = {}
    summary = persist.apply_seed(mem, cfg, result.seed, 0.0)
    assert summary["sections"]["fan_curves"] == len(curves)
    assert mem["fan_curves"] == curves


def test_step_without_the_switch_keeps_no_accumulator_and_reports_nothing() -> None:
    cfg = das_cfg(model_shadow=True)
    state = MpcState.cold()
    for i in range(10):
        obs = das_obs(cfg, float(i) * cfg.dt, pwm=0.5, temps=default_temps(cfg))
        cmd, state = step(obs, cfg, state)
    assert "fan_fit" not in state.solver_memory
    assert "fan_curves" not in cmd.diagnostics


# ---------------------------------------------------------------------------
# the readers the fit reaches, and the ones it deliberately does not (item 107)
# ---------------------------------------------------------------------------

#: A fit far from the fixture's ``fan_models`` on every axis, so any reader that follows
#: it is visible in the arithmetic.
FITTED = {
    "p12": {"rpm_max": 2100.0, "deadband": 0.35, "exponent": 0.6},
    "p14": {"rpm_max": 1400.0, "deadband": 0.0, "exponent": 1.4},
}


def test_every_reader_of_the_fan_curve_data_has_a_decision() -> None:
    """The table is item 107's decision list: a reader added later without a decision (or
    with one that is neither ``curve`` nor ``config``) fails here and in review."""
    assert set(fancurve.READERS) == {
        "thermal_model",
        "mpc_prediction",
        "estimator_airflow",
        "noise_u0",
        "noise_rpm_max",
        "noise_db_at_max",
        "model_use_rpm",
        "fan_health",
        "stuck_airflow",
    }
    assert set(fancurve.READERS.values()) == {"curve", "config"}


def test_the_estimator_airflow_follows_a_fitted_curve_and_falls_back_without_one() -> None:
    """``Q_z`` and ``Qn_z`` must come from the curve the thermal model plans on, or the
    MPC's prediction-error guard reads the difference as a bad model (item 107)."""
    from aqua_bridge.control import estimator

    cfg = online_cfg()
    temps = {k: v for k, v in default_temps(cfg).items() if v is not None}
    u = dict.fromkeys(cfg.channels, 0.5)

    def airflow(curves: Any) -> tuple[float, str]:
        up = estimator.update(None, cfg, temps=temps, u=u, ts=0.0, curves=curves)
        zone = up.zones["za"]
        return zone["airflow_w_per_k"], zone["airflow_curve"]

    configured, source = airflow(None)
    assert source == "config"
    # the same arithmetic by hand, from the fitted pair rather than fan_models'
    e_total = sum(cfg.fans[ch].count for ch in ("fa1", "fa2")) * 33.0
    assert configured == pytest.approx(e_total * thermal.phi(0.5, 0.1, 1.0))
    fitted, source = airflow(FITTED)
    assert source == "fit"
    assert fitted == pytest.approx(e_total * thermal.phi(0.5, 0.35, 0.6))
    assert abs(fitted - configured) > 1.0  # the two curves really do disagree

    # nothing usable for that model: back to the configured curve
    for curves in ({}, {"p12": {"deadband": 0.9, "exponent": 1.0, "rpm_max": 10.0}}):
        value, source = airflow(curves)
        assert value == pytest.approx(configured) and source == "config"


def test_the_noise_objective_u0_follows_the_fit_and_rpm_max_never_does() -> None:
    """The dead band is the objective's own model of where a fan starts to turn, so it
    follows the fit; ``rpm_max`` and ``noise_db_at_max`` do not (item 107)."""
    from aqua_bridge.control import noise

    cfg = online_cfg()
    u = dict.fromkeys(cfg.channels, 0.5)
    assert noise.channel_deadband(cfg, "fa1") == cfg.fan_models["p12"].deadband
    assert noise.channel_deadband(cfg, "fa1", FITTED) == 0.35
    assert noise.channel_deadband(cfg, "fa1", {}) == cfg.fan_models["p12"].deadband

    # a fan that only starts at 0.35 makes almost no noise at 0.5: the surrogate must not
    # charge the solver for noise the fan does not make
    plain = noise.surrogate(cfg, u)
    fitted = noise.surrogate(cfg, u, FITTED)
    assert fitted.value["fa1"] < 0.5 * plain.value["fa1"]
    assert noise.surrogate(cfg, u, {}).value["fa1"] == pytest.approx(plain.value["fa1"])

    # rpm_max stays the commissioned one: at full duty the modelled speed is unchanged
    assert noise.rpm_model(cfg, "fa1", 1.0, FITTED) == cfg.fan_models["p12"].rpm_max
    assert noise.rpm_model(cfg, "fa1", 1.0) == cfg.fan_models["p12"].rpm_max
    # so is P_ref, which is count and noise_db_at_max only
    assert noise.reference_power(cfg) == pytest.approx(
        sum(
            cfg.fans[ch].count * 10.0 ** (cfg.fan_models[cfg.fans[ch].model].noise_db_at_max / 10.0)
            for ch in cfg.channels
        )
    )


def test_the_noise_diagnostics_name_the_curve_behind_the_index() -> None:
    from aqua_bridge.control import noise

    cfg = online_cfg()
    prev = dict.fromkeys(cfg.channels, 0.5)
    rpm: dict[str, float | None] = {"fa1": 900.0, "fa2": None, "fb1": None, "fc1": None}
    plain = noise.noise_diagnostics(cfg, prev=prev, pwm=prev, rpm=rpm)
    fitted = noise.noise_diagnostics(cfg, prev=prev, pwm=prev, rpm=rpm, curves=FITTED)
    assert plain["channels"]["fa1"]["curve"] == "config"
    assert plain["channels"]["fa1"]["u0"] == cfg.fan_models["p12"].deadband
    assert fitted["channels"]["fa1"]["curve"] == "fit"
    assert fitted["channels"]["fa1"]["u0"] == 0.35
    # the tach branch is untouched: a measured speed normalised by the commissioned
    # rpm_max is the same number whatever was fitted
    assert fitted["channels"]["fa1"]["rpm"] == plain["channels"]["fa1"]["rpm"] == 900.0
    # the modelled channels move, and the index with them
    assert fitted["channels"]["fa2"]["rpm"] < plain["channels"]["fa2"]["rpm"]
    assert fitted["db_index"] != plain["db_index"]


def test_fan_health_judges_a_fan_against_the_configured_curve_not_the_fit() -> None:
    """A rule derived from the configured curve must not drift with a fit: the fit is made
    from the very readings the rule judges, so a fan that slows down would take the curve
    with it and the deviation would never show (item 107)."""
    from aqua_bridge.health import FanHealthConfig, HealthMonitor, expected_rpm

    cfg = online_cfg()
    spec = cfg.fan_models["p12"]
    assert fancurve.READERS["fan_health"] == "config"
    assert expected_rpm(cfg, "fa1", 1.0) == pytest.approx(spec.rpm_max)

    # the fan turns at half the commissioned speed, which is what a fit would follow
    mon = HealthMonitor(cfg, FanHealthConfig())
    reading = {
        "device": "aquaero",
        "output": "pwm1",
        "rpm": 0.5 * spec.rpm_max,
        "duty": 1.0,
        "voltage_v": 12.1,
        "power_reported": False,
        "aquabus": True,
    }
    mon.check_channel("fa1", reading, 0.0)
    t = mon.settings.settle_s + 1.0
    mon.check_channel("fa1", reading, t)
    verdict = mon.check_channel("fa1", reading, t + mon.settings.rpm_fault_s)
    assert verdict["expected_rpm"] == pytest.approx(spec.rpm_max)
    assert verdict["problems"], "a fan at half speed must still be a deviation"


def test_the_stuck_airflow_evidence_stays_on_the_configured_curve() -> None:
    """Gate rule 3's airflow evidence is derived from the config once at load
    (``StuckParams.airflow``), and no fit reaches it (item 107)."""
    cfg = online_cfg()
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0])
    state = MpcState(solver_memory={"fan_fit": memory})
    _, state = step(das_obs(cfg, 0.0, pwm=0.5), cfg, state)
    assert state.solver_memory["fan_curves"] == curves  # a fit is in force

    for channel, weight, deadband, exponent in cfg.stuck_params("prox_a1").airflow:
        model = cfg.fan_models[cfg.fans[channel].model]
        assert (deadband, exponent) == (model.deadband, model.exponent)
        assert weight > 0.0


def test_a_fit_nothing_re_confirms_goes_stale_and_every_reader_falls_back() -> None:
    """``fan_curve_max_age_s``: the curve leaves ``solver_memory["fan_curves"]``, so the
    readers that follow it are back on the configured curve, and the diagnostics say the
    fit went stale rather than quietly reporting the config (item 107)."""
    cfg = online_cfg(fan_curve_max_age_s=60.0)
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0])
    assert set(curves) == {"p12", "p14"}
    state = MpcState(solver_memory={"fan_fit": memory})
    cmd, state = step(das_obs(cfg, 0.0, pwm=0.5), cfg, state)
    assert set(state.solver_memory["fan_curves"]) == {"p12", "p14"}
    assert cmd.diagnostics["fan_curves"]["models"]["p12"]["stale"] is False

    # the tachometer stops reporting: no new samples, every refit refused, and the
    # accepted fit ages past fan_curve_max_age_s
    blind = {ch: None for ch in cfg.channels}
    ts = 0.0
    for i in range(1, 12):
        ts = float(i) * 20.0
        cmd, state = step(das_obs(cfg, ts, pwm=0.5, rpm=blind), cfg, state)
    assert state.solver_memory["fan_curves"] == {}
    row = cmd.diagnostics["fan_curves"]["models"]["p12"]
    assert row["stale"] is True and row["source"] == "config"
    # the bins still carry the old sweep, so a refit keeps passing; what has aged out is
    # the evidence under it, and that is what the diagnostics show
    assert row["sample_age_s"] > cfg.fan_curve_max_age_s
    assert row["deadband"] == cfg.fan_models["p12"].deadband
    assert row["exponent"] == cfg.fan_models["p12"].exponent

    # and the readers that followed it are back on fan_models
    assert cmd.diagnostics["noise"]["channels"]["fa1"]["curve"] == "config"
    assert cmd.diagnostics["estimator"]["zones"]["za"]["airflow_curve"] == "config"
    params = thermal.model_params(cfg, curves=state.solver_memory["fan_curves"])
    assert params.fan["fa1"] == (cfg.fan_models["p12"].deadband, cfg.fan_models["p12"].exponent)


def test_a_fit_that_keeps_being_re_confirmed_never_goes_stale() -> None:
    """Ordinary regulation with a live tachometer: both stamps keep moving, however many
    times the age window passes."""
    cfg = online_cfg(fan_curve_max_age_s=60.0)
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0])
    assert set(curves) == {"p12", "p14"}
    # 400 ticks more, ten age windows, every one of them sampled and refitted
    memory, curves = feed(cfg, [0.4, 0.8] * 25, memory=memory, t0=100.0)
    assert set(curves) == {"p12", "p14"}
    out = fancurve.update(
        memory,
        cfg,
        u=dict.fromkeys(cfg.channels, 0.8),
        rpm={ch: truth_rpm(cfg, ch, 0.8) for ch in cfg.channels},
        ts=500.0,
    )
    assert set(out.curves) == {"p12", "p14"} and out.stale == ()


def test_the_diagnostics_carry_the_reader_table_and_the_curve_in_force() -> None:
    """Nobody should have to guess which curve produced a number: the summary names the
    source per fan model and reports the decision per reader (item 107)."""
    cfg = online_cfg()
    memory, curves = feed(cfg, [0.2, 0.4, 0.6, 0.8, 1.0])
    state = MpcState(solver_memory={"fan_fit": memory})
    cmd, state = step(das_obs(cfg, 0.0, pwm=0.5), cfg, state)
    summary = cmd.diagnostics["fan_curves"]
    assert summary["readers"] == fancurve.READERS
    assert summary["models"]["p12"]["source"] == "fit"
    assert summary["models"]["p12"]["stale"] is False
    assert summary["models"]["p12"]["age_s"] == pytest.approx(0.0, abs=1e-6)
    assert cmd.diagnostics["noise"]["channels"]["fa1"]["curve"] == "fit"
    assert cmd.diagnostics["noise"]["channels"]["fa1"]["u0"] == curves["p12"]["deadband"]
    assert cmd.diagnostics["estimator"]["zones"]["za"]["airflow_curve"] == "fit"


def test_a_curve_from_the_store_is_in_force_until_a_fit_replaces_it() -> None:
    """A seeded curve has no fit of this run behind it, so the stale rule has nothing to
    measure: it stays in force (the store's own age rule judged it at load) and reports
    ``store``."""
    cfg = online_cfg(fan_curve_max_age_s=60.0)
    state = MpcState(solver_memory={"fan_curves": {"p12": dict(FITTED["p12"])}})
    cmd, state = step(das_obs(cfg, 1e6, pwm=0.5), cfg, state)
    row = cmd.diagnostics["fan_curves"]["models"]["p12"]
    assert row["source"] == "store" and row["stale"] is False and row["age_s"] is None
    assert state.solver_memory["fan_curves"]["p12"] == FITTED["p12"]
    assert cmd.diagnostics["estimator"]["zones"]["za"]["airflow_curve"] == "fit"


def test_the_max_age_must_leave_room_for_one_refit() -> None:
    with pytest.raises(ConfigError, match="fan_curve_max_age_s"):
        online_cfg(fan_curve_refit_s=600.0, fan_curve_max_age_s=599.0)
