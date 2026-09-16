"""Fan noise model of the DAS (plan section 4, "Noise model"). Pure, no I/O.

Noise is modelled from fan speed. Per PWM channel ``i`` (``fans.<i>``, fan model
``m = fans.<i>.model``)::

    rpm_i(u)  = rpm_max,m * r_i(u)          r_i(u) = clip((u - u0_m) / (1 - u0_m), 0, 1)
    L_i       = L_max,m + 10 log10(count_i) + 10 n log10(rpm_i / rpm_max,m)
    P_i       = count_i * 10 ** (L_max,m / 10) * r_i ** n        (linear sound power)
    noise_db  = 10 log10(sum_i P_i)                               (energetic total)

with ``u0_m = fan_models.<m>.deadband``, ``L_max,m = fan_models.<m>.noise_db_at_max``
and the fan affinity exponent ``n = noise.exponent`` (sound power ~ N^n). ``noise_db``
is an index; it is absolute only when ``noise_db_at_max`` comes from datasheets at
``rpm_max``. A total of zero (every fan below its dead band) reads
:data:`NOISE_FLOOR_DB` so the value stays finite. A tach-less channel uses the
curve of its fan model like any other.

Which curve
-----------
``u0_m`` follows the curve **in force**: every function here takes ``curves``
(``solver_memory["fan_curves"]``, the online fit of
:mod:`aqua_bridge.control.fancurve` with ``mpc.fan_curve_online``) and reads the
fitted dead band where there is a usable entry for the fan model, the configured
``fan_models.<m>.deadband`` otherwise -- ``None`` (the legacy path, and the DAS
with the fit switched off) is always the config. It changes the *objective*, not
the safety: with a real dead band of 0.25 and a configured 0.1 the surrogate
charges the solver for noise over a band where the fan does not turn, and its
gradient sends the command to the wrong place. It also keeps one ``u0`` across
the controller -- the thermal model, the estimator's airflow and this objective
plan on the same fan (PROJECT.md section 8 item 107).

Two figures here never follow a fit. ``L_max,m`` (``noise_db_at_max``) is a
datasheet number at ``rpm_max`` that a PWM -> RPM fit says nothing about. And
``rpm_max,m`` stays the commissioned one wherever it normalises or scales a
speed, so a fan that loses speed shows up as less noise rather than being
normalised away -- the same reason ``model_use_rpm`` keeps it
(:func:`aqua_bridge.control.thermal._channel_phi`).

``diagnostics["noise"]`` (:func:`noise_diagnostics`) reports the index from the
speed the fans turn: a channel's measured rpm where its tachometer reports a finite
value (only the first fan of a splitter drives the tach, so ``count`` scales it),
else the curve at the command on the fans (``prev``); ``db_index_cmd`` is the curve
at the new command.

Cost surrogate for the DAS MPC (:func:`surrogate`)
--------------------------------------------------
The MPC minimises ``weight_noise * sum_i w_i P_i(u_i) / P_ref`` with the per-fan
``fans.<i>.noise_weight`` ``w_i`` (default 1; the owner penalises a fan near the
listening position) and ``P_ref = sum_i count_i 10 ** (L_max,i / 10)``, the power
with every fan at full speed, so the term is dimensionless and 1 at full speed
with unit weights (a choice the plan leaves open: without it a model with
``noise_db_at_max: 30`` would weigh the noise 1000 times more than one at 0 dB
against the same drive penalties). ``P_i`` is convex in ``u`` (``n >= 3``) and
the per-tick problem uses its quadratic expansion at the current command::

    P_i(u) ~ P_i(u_now) + g_i (u - u_now) + 1/2 h_i (u - u_now)^2
    g_i = dP_i/du (u_now),   h_i = max(d2P_i/du2 (u_now), 0.15 * d2P_i/du2 (pwm_max))

The curvature floor (15 % of the largest curvature on the admissible range) keeps
the surrogate strictly convex where the true curve is flat (low speed, below the
dead band). At full speed ``r = 1`` the derivatives are the left ones (lowering the
command lowers the speed), below the dead band they are 0.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from aqua_bridge.control import fancurve
from aqua_bridge.model import MpcConfig

__all__ = [
    "CURVATURE_FLOOR",
    "NOISE_FLOOR_DB",
    "RPM_FRAC_MAX",
    "Surrogate",
    "channel_deadband",
    "channel_power",
    "noise_db",
    "noise_diagnostics",
    "reference_power",
    "rpm_frac",
    "rpm_model",
    "surrogate",
]

#: Index reported when no fan turns (``10 log10`` of this floor on the linear power).
NOISE_FLOOR_DB = -120.0
#: Fraction of the largest curvature on ``[pwm_min, pwm_max]`` that floors ``h_i``.
CURVATURE_FLOOR = 0.15
#: A measured rpm above ``rpm_max`` counts up to this fraction (spread between fans).
RPM_FRAC_MAX = 1.5


def _finite(value: object) -> bool:
    return (
        isinstance(value, int | float)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def rpm_frac(u: float, deadband: float) -> float:
    """``r(u) = clip((u - u0) / (1 - u0), 0, 1)``."""
    return min(1.0, max(0.0, (float(u) - deadband) / (1.0 - deadband)))


def channel_deadband(
    cfg: MpcConfig, channel: str, curves: Mapping[str, Any] | None = None
) -> float:
    """``u0`` of a channel: the fitted dead band where the online fit has a usable curve
    for its fan model, else the configured one (module docstring, *Which curve*)."""
    model = cfg.fan_models[cfg.fans[channel].model]
    if curves is None:
        return float(model.deadband)
    u0, _, _ = fancurve.curve_pair(
        curves.get(cfg.fans[channel].model), model.deadband, model.exponent, model.rpm_max
    )
    return u0


def rpm_model(
    cfg: MpcConfig, channel: str, u: float, curves: Mapping[str, Any] | None = None
) -> float:
    """Modelled rpm of the fans on ``channel`` at PWM ``u`` (``rpm_max`` always the
    commissioned one; only ``u0`` follows a fitted curve)."""
    model = cfg.fan_models[cfg.fans[channel].model]
    return model.rpm_max * rpm_frac(u, channel_deadband(cfg, channel, curves))


def _level(
    cfg: MpcConfig, channel: str, curves: Mapping[str, Any] | None = None
) -> tuple[float, float]:
    """``(count * 10 ** (L_max / 10), u0)`` of a channel. ``L_max`` is the datasheet
    ``noise_db_at_max``, which no fit touches; ``u0`` follows the curve in force."""
    fan = cfg.fans[channel]
    model = cfg.fan_models[fan.model]
    level = fan.count * 10.0 ** (model.noise_db_at_max / 10.0)
    return level, channel_deadband(cfg, channel, curves)


def _exponent(cfg: MpcConfig) -> float:
    assert cfg.noise is not None
    return cfg.noise.exponent


def channel_power(cfg: MpcConfig, channel: str, frac: float) -> float:
    """Linear sound power ``P_i`` of a channel at speed fraction ``frac`` (unweighted)."""
    level, _ = _level(cfg, channel)
    frac = min(RPM_FRAC_MAX, max(0.0, float(frac)))
    return level * frac ** _exponent(cfg) if frac > 0 else 0.0


def reference_power(cfg: MpcConfig) -> float:
    """``P_ref``: every fan at full speed, unweighted (module docstring)."""
    return sum(_level(cfg, ch)[0] for ch in cfg.channels)


def noise_db(cfg: MpcConfig, fracs: Mapping[str, float]) -> float:
    """Energetic noise index from a speed fraction per channel (missing channel: 0)."""
    total = sum(channel_power(cfg, ch, fracs.get(ch, 0.0)) for ch in cfg.channels)
    floor = 10.0 ** (NOISE_FLOOR_DB / 10.0)
    return 10.0 * math.log10(max(total, floor))


@dataclass(frozen=True)
class Surrogate:
    """Quadratic noise surrogate per channel at ``u_now`` (normalised and weighted):
    ``cost_i(u) = value_i + g_i (u - u_now_i) + 1/2 h_i (u - u_now_i)^2``."""

    u_now: dict[str, float]
    value: dict[str, float]
    g: dict[str, float]
    h: dict[str, float]


def _derivatives(level: float, deadband: float, n: float, u: float) -> tuple[float, float, float]:
    """``(P, dP/du, d2P/du2)`` with ``r`` clipped to ``[0, 1]`` (left derivatives at 1)."""
    r = (float(u) - deadband) / (1.0 - deadband)
    if r <= 0.0:
        return 0.0, 0.0, 0.0
    r = min(r, 1.0)
    span = 1.0 - deadband
    return (
        level * r**n,
        level * n * r ** (n - 1.0) / span,
        level * n * (n - 1.0) * r ** (n - 2.0) / (span * span),
    )


def surrogate(
    cfg: MpcConfig, u_now: Mapping[str, float], curves: Mapping[str, Any] | None = None
) -> Surrogate:
    """The DAS MPC's noise surrogate at ``u_now`` (module docstring). ``u_now`` is clamped
    into ``[pwm_min, pwm_max]``; the result excludes ``noise.weight_noise``. ``curves``
    is ``solver_memory["fan_curves"]``: the fitted ``u0`` where there is one (module
    docstring, *Which curve*)."""
    n = _exponent(cfg)
    p_ref = reference_power(cfg)
    u0: dict[str, float] = {}
    value: dict[str, float] = {}
    g: dict[str, float] = {}
    h: dict[str, float] = {}
    for ch in cfg.channels:
        level, deadband = _level(cfg, ch, curves)
        scale = cfg.fans[ch].noise_weight / p_ref
        u = min(cfg.pwm_max, max(cfg.pwm_min, float(u_now[ch])))
        p, dp, d2p = _derivatives(level, deadband, n, u)
        _, _, d2p_max = _derivatives(level, deadband, n, cfg.pwm_max)
        u0[ch] = u
        value[ch] = scale * p
        g[ch] = scale * dp
        h[ch] = scale * max(d2p, CURVATURE_FLOOR * d2p_max)
    return Surrogate(u_now=u0, value=value, g=g, h=h)


def noise_diagnostics(
    cfg: MpcConfig,
    *,
    prev: Mapping[str, float],
    pwm: Mapping[str, float],
    rpm: Mapping[str, float | None],
    curves: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """``diagnostics["noise"]`` (module docstring): ``db_index`` from the speed the fans
    turn, ``db_index_cmd`` from the curve at the command, per channel the rpm used and its
    source (``tach`` | ``model``), the modelled rpm at the command, and the ``u0`` the
    index was computed with together with the curve it came from (``curve``: ``fit`` |
    ``config``), so no one has to guess which curve produced the number."""
    fracs: dict[str, float] = {}
    fracs_cmd: dict[str, float] = {}
    channels: dict[str, Any] = {}
    for ch in cfg.channels:
        model = cfg.fan_models[cfg.fans[ch].model]
        u0 = channel_deadband(cfg, ch, curves)
        fitted = curves is not None and fancurve.usable(curves.get(cfg.fans[ch].model))
        measured = rpm.get(ch)
        if _finite(measured) and float(measured) >= 0.0:  # type: ignore[arg-type]
            used, source = float(measured), "tach"  # type: ignore[arg-type]
            # the commissioned rpm_max normalises the tach, never a fitted one: a fit of
            # those same readings would make the fraction blind to a fan losing speed
            fracs[ch] = min(RPM_FRAC_MAX, used / model.rpm_max)
        else:
            fracs[ch] = rpm_frac(prev[ch], u0)
            used, source = model.rpm_max * fracs[ch], "model"
        fracs_cmd[ch] = rpm_frac(pwm[ch], u0)
        channels[ch] = {
            "rpm": used,
            "source": source,
            "rpm_cmd": model.rpm_max * fracs_cmd[ch],
            "u0": u0,
            "curve": "fit" if fitted else "config",
        }
    return {
        "db_index": noise_db(cfg, fracs),
        "db_index_cmd": noise_db(cfg, fracs_cmd),
        "channels": channels,
    }
