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
    ) == (30.0, 600.0, 0.05)
    with pytest.raises(ConfigError, match="requires mpc.topology"):
        dataclasses.replace(cfg, fan_curve_online=True)
    for key, bad in (
        ("fan_curve_settle_s", -1.0),
        ("fan_curve_refit_s", 0.0),
        ("fan_curve_max_rmse_frac", 0.0),
        ("fan_curve_max_rmse_frac", 1.5),
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
        fancurve.update(again, cfg, u=u, rpm={}, ts=1e4).curves
        == fancurve.update(memory, cfg, u=u, rpm={}, ts=1e4).curves
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
    kept = fancurve.update(memory, cfg, u=dict.fromkeys(cfg.channels, 0.5), rpm={}, ts=1e4)
    assert set(kept.curves) == set(curves)

    fans = das_mapping()["fans"]
    fans["fb1"]["model"] = "p12"
    swapped = online_cfg(fans=fans)
    started_over = fancurve.update(
        memory, swapped, u=dict.fromkeys(swapped.channels, 0.5), rpm={}, ts=1e4
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
