"""RC thermal plant for closed-loop tests (PROJECT.md sections 4.2, 4.6).

Two lumped temperatures, coolant and case air, with heat flowing::

    heat_w  -->  coolant  --(radiator fans)-->  air  --(intake fans)-->  ambient

    C_c dT_c/dt = heat_w - g_rad * (T_c - T_a)
    C_a dT_a/dt = g_rad * (T_c - T_a) - g_air * (T_a - T_ambient)

    g_rad = k_passive_w_per_k     + sum(radiator_fans[ch] * pwm_eff[ch])
    g_air = k_air_passive_w_per_k + sum(intake_fans[ch]   * pwm_eff[ch])

``pwm_eff`` is the commanded PWM after ``delay_ticks`` (actuator delay);
a stalled channel moves no air and reports ``rpm = 0`` whatever its PWM.
Fan RPM is ``rpm_max * pwm_eff``. Everything is explicit Euler with
``substeps`` sub-intervals per ``dt`` and is deterministic given ``seed``
(the only random element is optional Gaussian sensor noise on the
reported temperatures; the true state is never noisy).

The plant knows nothing about the controller. ``heat_w`` and ``stalled``
can be changed between ticks to inject disturbances. Model mismatch is
just a different :class:`PlantParams` than whatever the controller
assumes -- the PI controller assumes nothing, the MPC will carry its own
gains.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from aqua_bridge.model import MpcCommand, MpcConfig, MpcState, PlantObservation

__all__ = ["Plant", "PlantParams", "TickRecord", "run_closed_loop"]


@dataclass(frozen=True)
class PlantParams:
    """Physical parameters. Defaults give a small loop with tau_coolant of a few minutes."""

    dt: float = 2.0
    coolant: str = "coolant"
    air: str = "air"
    ambient_c: float = 22.0
    heat_w: float = 150.0
    c_coolant_j_per_k: float = 4000.0
    c_air_j_per_k: float = 600.0
    k_passive_w_per_k: float = 1.0
    k_air_passive_w_per_k: float = 4.0
    radiator_fans: dict[str, float] = field(default_factory=lambda: {"radiator": 25.0})
    intake_fans: dict[str, float] = field(default_factory=lambda: {"intake": 20.0})
    rpm_max: dict[str, float] = field(default_factory=dict)
    delay_ticks: int = 0
    stalled: frozenset[str] = frozenset()
    noise_sigma_c: float = 0.0
    substeps: int = 4

    def __post_init__(self) -> None:
        if self.dt <= 0:
            raise ValueError("dt must be > 0")
        if self.delay_ticks < 0:
            raise ValueError("delay_ticks must be >= 0")
        if self.substeps < 1:
            raise ValueError("substeps must be >= 1")
        if self.c_coolant_j_per_k <= 0 or self.c_air_j_per_k <= 0:
            raise ValueError("thermal capacities must be > 0")
        if self.noise_sigma_c < 0:
            raise ValueError("noise_sigma_c must be >= 0")
        object.__setattr__(self, "stalled", frozenset(self.stalled))

    @property
    def channels(self) -> tuple[str, ...]:
        seen: dict[str, None] = {}
        for ch in (*self.radiator_fans, *self.intake_fans):
            seen.setdefault(ch, None)
        return tuple(seen)

    def rpm_max_for(self, channel: str) -> float:
        return float(self.rpm_max.get(channel, 2000.0))


class Plant:
    """Mutable simulation object; one instance per closed-loop run."""

    def __init__(
        self,
        params: PlantParams | None = None,
        *,
        t_coolant: float | None = None,
        t_air: float | None = None,
        initial_pwm: Mapping[str, float] | float = 0.5,
        ts0: float = 0.0,
        seed: int = 0,
    ) -> None:
        self.params = params or PlantParams()
        self.heat_w = float(self.params.heat_w)
        self.stalled: set[str] = set(self.params.stalled)
        self.ts = float(ts0)
        self._rng = np.random.default_rng(seed)
        if isinstance(initial_pwm, Mapping):
            start = {ch: float(initial_pwm.get(ch, 0.0)) for ch in self.params.channels}
        else:
            start = dict.fromkeys(self.params.channels, float(initial_pwm))
        self._history: list[dict[str, float]] = [start]
        eq_c, eq_a = self.equilibrium(start)
        self.t_coolant = float(eq_c if t_coolant is None else t_coolant)
        self.t_air = float(eq_a if t_air is None else t_air)

    # -- helpers ------------------------------------------------------------

    @property
    def channels(self) -> tuple[str, ...]:
        return self.params.channels

    def _conductances(self, pwm: Mapping[str, float]) -> tuple[float, float]:
        p = self.params
        g_rad = p.k_passive_w_per_k
        for ch, gain in p.radiator_fans.items():
            if ch not in self.stalled:
                g_rad += gain * pwm.get(ch, 0.0)
        g_air = p.k_air_passive_w_per_k
        for ch, gain in p.intake_fans.items():
            if ch not in self.stalled:
                g_air += gain * pwm.get(ch, 0.0)
        return g_rad, g_air

    def equilibrium(self, pwm: Mapping[str, float]) -> tuple[float, float]:
        """Steady-state ``(T_coolant, T_air)`` for a constant PWM and the current ``heat_w``."""
        g_rad, g_air = self._conductances(pwm)
        t_air = self.params.ambient_c + self.heat_w / g_air
        return t_air + self.heat_w / g_rad, t_air

    def effective_pwm(self) -> dict[str, float]:
        """PWM currently acting on the fans (commands appear after ``delay_ticks``)."""
        idx = max(0, len(self._history) - 1 - self.params.delay_ticks)
        return dict(self._history[idx])

    # -- simulation ---------------------------------------------------------

    def apply(self, pwm: Mapping[str, float]) -> None:
        """Queue a command; it acts after ``delay_ticks`` calls to :meth:`advance`."""
        clean = {}
        for ch in self.params.channels:
            v = float(pwm.get(ch, self._history[-1].get(ch, 0.0)))
            clean[ch] = min(1.0, max(0.0, v))
        self._history.append(clean)
        keep = self.params.delay_ticks + 2
        if len(self._history) > keep:
            del self._history[:-keep]

    def advance(self) -> None:
        """Integrate one ``dt`` with the effective PWM; ``ts += dt``."""
        p = self.params
        g_rad, g_air = self._conductances(self.effective_pwm())
        h = p.dt / p.substeps
        tc, ta = self.t_coolant, self.t_air
        for _ in range(p.substeps):
            q_rad = g_rad * (tc - ta)
            q_air = g_air * (ta - p.ambient_c)
            tc += h * (self.heat_w - q_rad) / p.c_coolant_j_per_k
            ta += h * (q_rad - q_air) / p.c_air_j_per_k
        self.t_coolant, self.t_air = tc, ta
        self.ts += p.dt

    def observe(self) -> PlantObservation:
        """Current sensor readings (noisy if ``noise_sigma_c > 0``)."""
        p = self.params
        eff = self.effective_pwm()
        tc, ta = self.t_coolant, self.t_air
        if p.noise_sigma_c > 0:
            noise = self._rng.normal(0.0, p.noise_sigma_c, size=2)
            tc += float(noise[0])
            ta += float(noise[1])
        rpm = {
            ch: 0.0 if ch in self.stalled else p.rpm_max_for(ch) * eff[ch] for ch in self.channels
        }
        return PlantObservation(
            temps={p.coolant: tc, p.air: ta}, rpm=rpm, pwm=dict(eff), ts=self.ts
        )

    def step(self, pwm: Mapping[str, float]) -> PlantObservation:
        """``apply`` + ``advance`` + ``observe``."""
        self.apply(pwm)
        self.advance()
        return self.observe()


@dataclass(frozen=True)
class TickRecord:
    obs: PlantObservation
    cmd: MpcCommand
    state: MpcState


Controller = Callable[[PlantObservation, MpcConfig, MpcState], tuple[MpcCommand, MpcState]]
ObsHook = Callable[[int, PlantObservation], PlantObservation]


def run_closed_loop(
    plant: Plant,
    cfg: MpcConfig,
    controller: Controller,
    ticks: int,
    *,
    state: MpcState | None = None,
    observe_hook: ObsHook | None = None,
    on_tick: Callable[[TickRecord], Any] | None = None,
    heat_schedule: Iterable[tuple[int, float]] = (),
) -> list[TickRecord]:
    """Drive ``controller`` against ``plant`` for ``ticks`` ticks.

    ``observe_hook(i, obs)`` may replace the observation (fault injection);
    ``heat_schedule`` is ``(tick_index, heat_w)`` pairs applied before that
    tick; ``on_tick`` sees every record (tests assert invariants there).
    The plant is ``apply``-ed the command and advanced after each step.
    """
    schedule = dict(heat_schedule)
    st = MpcState.cold() if state is None else state
    records: list[TickRecord] = []
    for i in range(ticks):
        if i in schedule:
            plant.heat_w = float(schedule[i])
        obs = plant.observe()
        if observe_hook is not None:
            obs = observe_hook(i, obs)
        cmd, st = controller(obs, cfg, st)
        rec = TickRecord(obs=obs, cmd=cmd, state=st)
        records.append(rec)
        if on_tick is not None:
            on_tick(rec)
        plant.apply(cmd.pwm)
        plant.advance()
    return records
