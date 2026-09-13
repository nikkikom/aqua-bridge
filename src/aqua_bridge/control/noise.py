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
:data:`NOISE_FLOOR_DB` so the value stays finite. The plan fits ``rpm(u)`` per fan
model from recorded ``(u, rpm)`` pairs (``tools/fit_fans.py``, a later milestone);
until then the configured ``rpm_max`` and ``deadband`` are the curve, and a
tach-less channel uses the curve of its model like any other.

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

from aqua_bridge.model import MpcConfig

__all__ = [
    "CURVATURE_FLOOR",
    "NOISE_FLOOR_DB",
    "RPM_FRAC_MAX",
    "Surrogate",
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


def rpm_model(cfg: MpcConfig, channel: str, u: float) -> float:
    """Modelled rpm of the fans on ``channel`` at PWM ``u``."""
    model = cfg.fan_models[cfg.fans[channel].model]
    return model.rpm_max * rpm_frac(u, model.deadband)


def _level(cfg: MpcConfig, channel: str) -> tuple[float, float]:
    """``(count * 10 ** (L_max / 10), deadband)`` of a channel."""
    fan = cfg.fans[channel]
    model = cfg.fan_models[fan.model]
    return fan.count * 10.0 ** (model.noise_db_at_max / 10.0), model.deadband


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


def surrogate(cfg: MpcConfig, u_now: Mapping[str, float]) -> Surrogate:
    """The DAS MPC's noise surrogate at ``u_now`` (module docstring). ``u_now`` is clamped
    into ``[pwm_min, pwm_max]``; the result excludes ``noise.weight_noise``."""
    n = _exponent(cfg)
    p_ref = reference_power(cfg)
    u0: dict[str, float] = {}
    value: dict[str, float] = {}
    g: dict[str, float] = {}
    h: dict[str, float] = {}
    for ch in cfg.channels:
        level, deadband = _level(cfg, ch)
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
) -> dict[str, Any]:
    """``diagnostics["noise"]`` (module docstring): ``db_index`` from the speed the fans
    turn, ``db_index_cmd`` from the curve at the command, per channel the rpm used and its
    source (``tach`` | ``model``) and the modelled rpm at the command."""
    fracs: dict[str, float] = {}
    fracs_cmd: dict[str, float] = {}
    channels: dict[str, Any] = {}
    for ch in cfg.channels:
        model = cfg.fan_models[cfg.fans[ch].model]
        measured = rpm.get(ch)
        if _finite(measured) and float(measured) >= 0.0:  # type: ignore[arg-type]
            used, source = float(measured), "tach"  # type: ignore[arg-type]
            fracs[ch] = min(RPM_FRAC_MAX, used / model.rpm_max)
        else:
            fracs[ch] = rpm_frac(prev[ch], model.deadband)
            used, source = model.rpm_max * fracs[ch], "model"
        fracs_cmd[ch] = rpm_frac(pwm[ch], model.deadband)
        channels[ch] = {
            "rpm": used,
            "source": source,
            "rpm_cmd": model.rpm_max * fracs_cmd[ch],
        }
    return {
        "db_index": noise_db(cfg, fracs),
        "db_index_cmd": noise_db(cfg, fracs_cmd),
        "channels": channels,
    }
