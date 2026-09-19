"""Shared invariant checks (PROJECT.md section 4.1).

Every suite -- nominal, failures, lies, fuzzy, closed-loop -- calls
:func:`assert_command_safe` on every ``step`` and :func:`assert_state_finite`
on every returned state. A test whose story passes but whose command
violates one of these is a failure.
"""

from __future__ import annotations

import math
import numbers
from collections.abc import Mapping
from typing import Any

from aqua_bridge.model import (
    SETPOINT_GROUP_PREFIX,
    Mode,
    MpcCommand,
    MpcConfig,
    MpcState,
    PlantObservation,
)

__all__ = [
    "TOL",
    "assert_command_safe",
    "assert_no_non_finite",
    "assert_state_finite",
    "checked_step",
    "make_obs",
    "obs_pwm_trusted",
    "obs_structurally_untrusted",
    "resolve_prev_pwm",
    "structurally_faulted_zones",
]

# Float slack for the rate-limit and bound comparisons (not for NaN/Inf).
TOL = 1e-9


def _is_number(value: object) -> bool:
    return isinstance(value, numbers.Real) and not isinstance(value, bool)


def _walk(value: Any, path: str, sink: list[str]) -> None:
    """Collect paths of every non-finite number inside nested containers."""
    if value is None or isinstance(value, str | bytes | bool):
        return
    if _is_number(value):
        if not math.isfinite(float(value)):
            sink.append(f"{path}={value!r}")
        return
    if isinstance(value, Mapping):
        for k, v in value.items():
            _walk(v, f"{path}[{k!r}]", sink)
        return
    if isinstance(value, list | tuple | set | frozenset):
        for i, v in enumerate(value):
            _walk(v, f"{path}[{i}]", sink)
        return
    tolist = getattr(value, "tolist", None)  # numpy scalars / arrays
    if callable(tolist):
        _walk(tolist(), path, sink)


def assert_no_non_finite(value: Any, what: str = "value") -> None:
    """Fail if any number reachable inside ``value`` is NaN or infinite."""
    bad: list[str] = []
    _walk(value, what, bad)
    assert not bad, f"non-finite numbers in {what}: {bad}"


def obs_pwm_trusted(obs: PlantObservation, cfg: MpcConfig) -> bool:
    """``obs.pwm`` usable as ``prev``: every channel present, finite, and inside
    ``[max(0, pwm_min - d_pwm_max), min(1, pwm_max + d_pwm_max)]``.

    That band is exactly the set of ``prev`` values from which one
    rate-limited step (``|delta| <= d_pwm_max``) can reach ``[pwm_min,
    pwm_max]``; a reading outside it (e.g. fans off at boot with
    ``pwm_min > d_pwm_max``) cannot satisfy both section 4.1 bullets at once
    and therefore falls through to ``cfg.fallback_pwm`` (section 3 item 3).
    Mirrors ``aqua_bridge.control.mpc.obs_pwm_usable``.
    """
    lo = max(0.0, cfg.pwm_min - cfg.d_pwm_max)
    hi = min(1.0, cfg.pwm_max + cfg.d_pwm_max)
    for ch in cfg.channels:
        v = obs.pwm.get(ch)
        if v is None or not math.isfinite(v) or not lo <= v <= hi:
            return False
    return True


def resolve_prev_pwm(state: MpcState, obs: PlantObservation, cfg: MpcConfig) -> dict[str, float]:
    """The ``prev`` a command is rate-limited against (section 4.1).

    In order: ``state.last_cmd.pwm``; else trusted ``obs.pwm``; else
    ``cfg.fallback_pwm``. Never ``pwm_min``.
    """
    if state.last_cmd is not None:
        return dict(state.last_cmd.pwm)
    if obs_pwm_trusted(obs, cfg):
        return {ch: float(obs.pwm[ch]) for ch in cfg.channels}  # type: ignore[arg-type]
    return dict(cfg.fallback_pwm)


def make_obs(
    cfg: MpcConfig,
    ts: float,
    temps: Mapping[str, float | None] | None = None,
    pwm: Mapping[str, float | None] | None = None,
    rpm: Mapping[str, float | None] | None = None,
    **temp_overrides: float | None,
) -> PlantObservation:
    """A well-formed observation for ``cfg`` with sensible defaults.

    Temperatures default to their setpoint (30 degrees C without one), PWM to
    0.5 and RPM to 1000 on every channel. ``temps`` replaces the whole
    temperature dict (so keys can be dropped or added); keyword overrides
    patch single temperatures on top of the defaults.
    """
    if temps is None:
        t: dict[str, float | None] = {name: cfg.setpoints.get(name, 30.0) for name in cfg.temps}
    else:
        t = dict(temps)
    t.update(temp_overrides)
    if pwm is None:
        pwm = dict.fromkeys(cfg.channels, 0.5)
    if rpm is None:
        rpm = dict.fromkeys(cfg.channels, 1000.0)
    return PlantObservation(temps=t, rpm=dict(rpm), pwm=dict(pwm), ts=ts)


def obs_structurally_untrusted(obs: PlantObservation, cfg: MpcConfig) -> bool:
    """True when the gate *must* reject the tick without any history.

    Covers section 3 gate rule 2's history-free part: a temperature in
    ``cfg.temps`` that is missing, ``None``, non-finite or outside the
    absolute valid range, or a key in ``obs.temps`` that is not in
    ``cfg.temps``. Slew and stuck checks need state and are not decided here.

    With ``cfg.median3`` a finite out-of-range value is *not* structural: the
    median of three removes a single Spike, and section 3 requires zero
    untrusted ticks for it. Missing / ``None`` / NaN / unknown keys are never
    filtered away and stay structural in both settings.
    """
    if set(obs.temps) - set(cfg.temps):
        return True
    for name in cfg.temps:
        v = obs.temps.get(name)
        if v is None or not math.isfinite(v):
            return True
        if not cfg.median3 and not cfg.temp_min_c <= v <= cfg.temp_max_c:
            return True
    return False


def _structurally_bad(obs: PlantObservation, cfg: MpcConfig, name: str) -> bool:
    v = obs.temps.get(name)
    if v is None or not math.isfinite(v):
        return True
    return not cfg.median3 and not cfg.temp_min_c <= v <= cfg.temp_max_c


def structurally_faulted_zones(obs: PlantObservation, cfg: MpcConfig) -> set[str]:
    """Zones the gate *must* fault without any history (zoned counterpart of
    :func:`obs_structurally_untrusted`).

    An unknown key faults every zone; otherwise a zone faults when some
    required group (``cfg.zone_layout.required_groups``) has every member
    structurally bad. With ``zones.trust_rule: sigma`` only the setpoint groups
    count: a lost drive or air sensor is the estimator's sigma, which needs
    history (on a tick with an estimator fault ``sigma`` applies ``strict``, which
    faults at least these zones). Legacy mode: the implicit zone iff
    :func:`obs_structurally_untrusted`.
    """
    layout = cfg.zone_layout
    if set(obs.temps) - set(cfg.temps):
        return set(layout.zones)
    sigma = cfg.zones is not None and cfg.zones.trust_rule == "sigma" and not layout.implicit
    out: set[str] = set()
    for zone in layout.zones:
        for label, members in layout.required_groups[zone]:
            if sigma and not label.startswith(SETPOINT_GROUP_PREFIX):
                continue
            if all(_structurally_bad(obs, cfg, name) for name in members):
                out.add(zone)
                break
    return out


def assert_command_safe(
    obs: PlantObservation,
    cfg: MpcConfig,
    cmd: MpcCommand,
    prev_pwm: Mapping[str, float] | MpcCommand,
) -> None:
    """Assert every single-step bullet of section 4.1 on ``cmd``.

    ``prev_pwm`` is what :func:`resolve_prev_pwm` returns (or the previous
    ``MpcCommand``). Determinism and the hold-then-high policy span several
    steps and are asserted by the calling test.
    """
    if isinstance(prev_pwm, MpcCommand):
        prev_pwm = prev_pwm.pwm
    assert isinstance(cmd, MpcCommand), f"step must return an MpcCommand, got {type(cmd)}"

    # Exactly config.channels, no extras, no missing keys.
    assert set(cmd.pwm) == set(cfg.channels), (
        f"cmd.pwm keys {sorted(cmd.pwm)} != config.channels {sorted(cfg.channels)}"
    )
    assert set(prev_pwm) >= set(cfg.channels), (
        f"prev_pwm is missing channels: {sorted(set(cfg.channels) - set(prev_pwm))}"
    )

    for ch in cfg.channels:
        v = cmd.pwm[ch]
        assert _is_number(v) and math.isfinite(v), f"pwm[{ch!r}] not finite: {v!r}"
        assert cfg.pwm_min - TOL <= v <= cfg.pwm_max + TOL, (
            f"pwm[{ch!r}]={v} outside [{cfg.pwm_min}, {cfg.pwm_max}]"
        )
        prev = prev_pwm[ch]
        assert _is_number(prev) and math.isfinite(prev), f"prev_pwm[{ch!r}] not finite: {prev!r}"
        assert abs(v - prev) <= cfg.d_pwm_max + TOL, (
            f"pwm[{ch!r}] moved {v - prev:+.6f} from {prev} (limit {cfg.d_pwm_max})"
        )

    assert_no_non_finite(cmd.diagnostics, "cmd.diagnostics")

    assert isinstance(cmd.mode, Mode) and cmd.mode in tuple(Mode), (
        f"cmd.mode {cmd.mode!r} is not one of {[m.value for m in Mode]}"
    )

    # A tick the gate can reject without history must be fallback.
    if not cfg.is_das:
        if obs_structurally_untrusted(obs, cfg):
            assert cmd.mode is Mode.FALLBACK, (
                f"observation is structurally untrusted but cmd.mode={cmd.mode.value!r}"
            )
        # "A fault never reduces cooling" (section 4.1: never a step toward
        # ``pwm_min`` *because* of the fault). Every fallback tick, not only the
        # untrusted ones: a trusted tick still inside ``confirm_ticks`` and a solver
        # fault follow the same hold-then-high policy. Only the clamp into
        # ``[pwm_min, pwm_max]`` may lower a ``prev`` that came from ``obs.pwm``
        # above ``pwm_max``. The zoned counterpart is in
        # :func:`assert_zone_step_safe`, per faulted zone's reach.
        if cmd.mode is Mode.FALLBACK:
            for ch in cfg.channels:
                floor = min(prev_pwm[ch], cfg.pwm_max)
                assert cmd.pwm[ch] >= floor - TOL, (
                    f"the fault lowered {ch!r}: {cmd.pwm[ch]} < {floor}"
                )
        return
    # Zones: the structurally faulted zones must be in fault, their channels under
    # fallback policy and never commanded below prev (clamped into the box).
    bad = structurally_faulted_zones(obs, cfg)
    if not bad:
        return
    if bad == set(cfg.zone_layout.zones):
        assert cmd.mode is Mode.FALLBACK, (
            f"every zone structurally untrusted but {cmd.mode.value!r}"
        )
    else:
        assert cmd.mode in (Mode.FALLBACK, Mode.DEGRADED), (
            f"zones {sorted(bad)} structurally untrusted but cmd.mode={cmd.mode.value!r}"
        )
    in_fault = set(cmd.diagnostics.get("zones_in_fault", ()))
    assert bad <= in_fault, (
        f"zones {sorted(bad - in_fault)} structurally untrusted but not in fault"
    )
    fixed = set(cmd.diagnostics.get("fallback_channels", ()))
    for zone in bad:
        for ch in cfg.zone_layout.zone_channels[zone]:
            assert ch in fixed, f"channel {ch!r} of faulted zone {zone!r} not under fallback policy"


def assert_state_finite(state: MpcState) -> None:
    """No NaN / Inf anywhere in the returned state; it must also round-trip."""
    assert isinstance(state, MpcState), f"step must return an MpcState, got {type(state)}"
    d = state.to_dict()
    assert_no_non_finite(d, "state")
    assert state.trusted_streak >= 0
    if state.fault_reason is not None or state.fault_since_ts is not None:
        assert state.fault_reason is not None and state.fault_since_ts is not None, (
            "fault_since_ts and fault_reason must be set together: "
            f"{state.fault_since_ts!r}, {state.fault_reason!r}"
        )


def checked_step(
    obs: PlantObservation, cfg: MpcConfig, state: MpcState, **kwargs: Any
) -> tuple[MpcCommand, MpcState]:
    """``mpc.step`` wrapped in every section 4.1 single-step assertion.

    Resolves ``prev`` with :func:`resolve_prev_pwm` *before* the call, asserts
    :func:`assert_command_safe` and :func:`assert_state_finite` on the result,
    checks that the input state was not mutated and that the returned
    command is the new ``state.last_cmd``. Extra keyword arguments go to
    ``step`` (e.g. ``solver=``).
    """
    from aqua_bridge.control.mpc import step  # local import: keep this module control-agnostic

    prev = resolve_prev_pwm(state, obs, cfg)
    before = state.to_dict()
    cmd, nxt = step(obs, cfg, state, **kwargs)
    assert_command_safe(obs, cfg, cmd, prev)
    assert_state_finite(nxt)
    assert state.to_dict() == before, "step mutated its input state"
    assert nxt.last_cmd == cmd, "returned state must carry the returned command as last_cmd"
    assert (cmd.mode in (Mode.FALLBACK, Mode.DEGRADED)) == nxt.in_fault, (
        f"mode {cmd.mode.value!r} disagrees with fault state {nxt.fault_since_ts!r}"
    )
    if cfg.is_das:
        assert_zone_step_safe(cfg, cmd, nxt, prev)
    return cmd, nxt


def assert_zone_step_safe(
    cfg: MpcConfig, cmd: MpcCommand, nxt: MpcState, prev: Mapping[str, float]
) -> None:
    """Per-zone invariants of one zoned ``step`` (plan section 0.1).

    * ``zone_faults`` has exactly the zones; ``fallback`` iff every zone is in
      fault, ``degraded`` iff some are; the global fields are the aggregates;
    * the channels under fallback policy are exactly the reach of the faulted
      zones (own channels plus declared coupling), and none of them is
      commanded below ``prev`` (clamped into ``[pwm_min, pwm_max]``).
    """
    layout = cfg.zone_layout
    assert set(nxt.zone_faults) == set(layout.zones), (
        f"zone_faults {sorted(nxt.zone_faults)} != zones {sorted(layout.zones)}"
    )
    faulted = [z for z in layout.zones if nxt.zone_faults[z].in_fault]
    if len(faulted) == len(layout.zones):
        assert cmd.mode is Mode.FALLBACK, f"every zone in fault but mode={cmd.mode.value!r}"
    elif faulted:
        assert cmd.mode is Mode.DEGRADED, f"zones {faulted} in fault but mode={cmd.mode.value!r}"
    else:
        assert cmd.mode in (Mode.AUTO, Mode.SATURATED), f"no zone in fault but {cmd.mode.value!r}"
    assert cmd.diagnostics["zones_in_fault"] == faulted
    since = [nxt.zone_faults[z].since_ts for z in faulted]
    assert nxt.fault_since_ts == (min(since) if since else None)  # type: ignore[type-var]
    assert nxt.trusted_streak == min(nxt.zone_faults[z].streak for z in layout.zones)
    reach: set[str] = set()
    for zone in faulted:
        reach.update(layout.reach[zone])
    fixed = set(cmd.diagnostics["fallback_channels"])
    assert fixed == reach, f"fallback channels {sorted(fixed)} != reach {sorted(reach)}"
    for ch in fixed:
        floor = min(max(prev[ch], cfg.pwm_min), cfg.pwm_max)
        assert cmd.pwm[ch] >= floor - TOL, (
            f"channel {ch!r} under fallback policy went below prev: {cmd.pwm[ch]} < {floor}"
        )
        assert ch not in nxt.integrator, f"channel {ch!r} under fallback kept an integrator entry"
