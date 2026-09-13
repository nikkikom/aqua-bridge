"""The glue loop: ``read -> step -> compose -> apply -> watchdog`` (sections 3, 4.3, 9).

:class:`Loop` knows nothing about USB, sysfs, HTTP or MQTT. It talks to a
:class:`Source` (``read() -> PlantObservation``) and a :class:`Sink`
(``apply(MpcCommand)``), runs :func:`aqua_bridge.control.mpc.step` with the
:class:`~aqua_bridge.control.supervisor.Supervisor`'s effective config, lets
the supervisor compose manual overrides in, applies the result and kicks the
systemd watchdog.

Failure policy (section 4.3 "Loop / glue")
------------------------------------------
* ``read()`` raises, or returns something that is not a
  :class:`~aqua_bridge.model.PlantObservation`: the tick is *not* skipped.
  A blank observation (no temps, no rpm, no pwm) is fed to ``step`` so the
  sensor gate rejects it and the normal hold / ramp-high fallback runs.
  That fallback command is not garbage -- it is rate limited against the
  last applied PWM -- and applying it is what turns a dead sensor path into
  the software watchdog of section 2 (fans ramp to ``fallback_pwm`` after
  ``fallback_hold_s``). Its ``ts`` continues the observation clock
  (``last obs.ts + dt``) so a source with its own time base is not
  confused by a foreign timestamp.
* ``apply()`` raises: logged, the tick still finishes, the next tick still
  runs. The new :class:`~aqua_bridge.model.MpcState` is committed (fault
  timers, gate window and integrator advance), but its ``last_cmd`` and the
  newest window sample carry the last command that was *actually applied*,
  so the next tick's rate limit (``|delta| <= d_pwm_max``) is measured from
  what is on the fans, not from a command that never reached them. A
  failed write is assumed not to have happened (the conventional, and
  conservative, reading: a partial write can be off by at most one
  ``d_pwm_max`` step).
* ``step`` / ``compose`` raise (a controller bug -- ``step`` promises not
  to): the state is left untouched, an *emergency* command ramps every
  channel from the last applied PWM up to ``cfg.fallback_pwm`` at
  ``d_pwm_max`` (never down: a fault must never reduce cooling) and the watchdog is
  **not** kicked, so systemd restarts the process after ``WatchdogSec``.
* ``sd_notify`` failures are ignored (they return ``False``).

Applied-command feedback
------------------------
The solver rate-limits against ``state.last_cmd``. The command the sink
receives can differ from the solver's (manual overrides, failed apply), so
after every tick the loop rewrites ``state.last_cmd`` -- and the newest
``WindowSample.cmd_pwm`` (stuck detection compares net *commanded* PWM) --
to the applied command. Channels the supervisor released from a manual
override have their integrator entry dropped for one tick; ``step`` then
re-initialises the solver bumplessly (its first output equals what is on
the fan).

Shutdown (section 9 "Stop path")
--------------------------------
:meth:`Loop.shutdown` writes ``cfg.fallback_pwm`` through the sink exactly
once (``mode=fallback``; this deliberately ignores ``d_pwm_max`` -- the
process is about to exit and nothing else will ramp the fans), then sends
``STOPPING=1``. It is the only stop path; there is no ``ExecStop=``.

Observers
---------
``on_tick`` (constructor argument or attribute) is called with every
:class:`TickResult` after the supervisor snapshot has been updated; the
publishers (MQTT state) hook in there. It runs on the loop thread, so it
must be quick; any exception is logged and dropped.

Timing
------
:meth:`Loop.run` ticks every ``cfg.dt`` seconds of ``clock`` time. When a
tick overruns, the schedule restarts from "now" instead of bursting to
catch up. ``READY=1`` is sent once, after the first tick whose command was
applied successfully.
"""

from __future__ import annotations

import dataclasses
import logging
import threading
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from aqua_bridge.control.mpc import resolve_prev, step
from aqua_bridge.control.supervisor import Supervisor, TickPlan
from aqua_bridge.model import Mode, MpcCommand, MpcConfig, MpcState, PlantObservation, WindowSample
from aqua_bridge.sdnotify import NullNotifier

__all__ = ["Loop", "Notifier", "Sink", "Source", "TickResult", "emergency_command"]

_LOG = logging.getLogger("aqua_bridge.loop")

Sleeper = Callable[[float, threading.Event], None]


@runtime_checkable
class Source(Protocol):
    def read(self) -> PlantObservation: ...


@runtime_checkable
class Sink(Protocol):
    def apply(self, cmd: MpcCommand) -> None: ...


@runtime_checkable
class Notifier(Protocol):
    def ready(self) -> bool: ...

    def watchdog(self) -> bool: ...

    def stopping(self) -> bool: ...


@dataclass(frozen=True)
class TickResult:
    """What one :meth:`Loop.tick` did."""

    index: int
    obs: PlantObservation
    mpc_cmd: MpcCommand | None
    cmd: MpcCommand
    state: MpcState
    applied: bool
    read_error: str | None = None
    apply_error: str | None = None
    controller_error: str | None = None
    watchdog_sent: bool = False

    @property
    def ok(self) -> bool:
        return self.read_error is None and self.apply_error is None and self.applied


def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def emergency_command(cfg: MpcConfig, prev_pwm: dict[str, float], error: str) -> MpcCommand:
    """Ramp from ``prev_pwm`` up to ``cfg.fallback_pwm`` at ``d_pwm_max`` (controller bug path).

    A channel already above ``fallback_pwm`` is held there: a fault must
    never reduce cooling (same rule as ``mpc.step`` step 6).
    """
    pwm: dict[str, float] = {}
    for ch in cfg.channels:
        prev = float(prev_pwm.get(ch, cfg.fallback_pwm[ch]))
        want = max(prev, cfg.fallback_pwm[ch])
        moved = _clamp(want, prev - cfg.d_pwm_max, prev + cfg.d_pwm_max)
        pwm[ch] = _clamp(moved, cfg.pwm_min, cfg.pwm_max)
    return MpcCommand(
        pwm=pwm,
        mode=Mode.FALLBACK,
        diagnostics={"policy": "emergency", "controller_error": error[:500]},
    )


def _default_sleep(seconds: float, stop: threading.Event) -> None:
    stop.wait(seconds)


class Loop:
    """One controller instance driving one source/sink pair."""

    def __init__(
        self,
        source: Source,
        sink: Sink,
        cfg: MpcConfig,
        supervisor: Supervisor,
        *,
        clock: Callable[[], float] = time.monotonic,
        notifier: Notifier | None = None,
        state: MpcState | None = None,
        sleep: Sleeper | None = None,
        on_tick: Callable[[TickResult], None] | None = None,
    ) -> None:
        if not isinstance(cfg, MpcConfig):
            raise TypeError(f"cfg must be an MpcConfig, got {type(cfg).__name__}")
        self.source = source
        self.sink = sink
        self.cfg = cfg
        self.supervisor = supervisor
        self.clock = clock
        self.notifier: Notifier = notifier if notifier is not None else NullNotifier()
        self.state: MpcState = MpcState.cold() if state is None else state
        self._sleep: Sleeper = sleep if sleep is not None else _default_sleep
        self.applied_cmd: MpcCommand | None = None
        self.last_obs: PlantObservation | None = None
        self.tick_count = 0
        self.ready_sent = False
        self.shutdown_done = False
        self.last_result: TickResult | None = None
        #: Called with every TickResult after the supervisor has been updated
        #: (publishers hook in here). Exceptions are logged and dropped: a
        #: publisher must never touch read -> step -> apply.
        self.on_tick = on_tick

    # -- helpers ----------------------------------------------------------

    def _blank_obs(self) -> PlantObservation:
        """Observation the gate is guaranteed to reject (no temperatures)."""
        ts = self.last_obs.ts + self.cfg.dt if self.last_obs is not None else self.clock()
        return PlantObservation(temps={}, rpm={}, pwm={}, ts=ts)

    def _read(self) -> tuple[PlantObservation, str | None]:
        try:
            obs = self.source.read()
        except Exception as exc:  # any source failure is a fallback tick, never a crash
            return self._blank_obs(), f"{type(exc).__name__}: {exc}"
        if not isinstance(obs, PlantObservation):
            return self._blank_obs(), f"source returned {type(obs).__name__}, not PlantObservation"
        return obs, None

    def _state_for_step(self, plan: TickPlan) -> MpcState:
        if not plan.released or not self.state.integrator:
            return self.state
        integrator = {ch: v for ch, v in self.state.integrator.items() if ch not in plan.released}
        return dataclasses.replace(self.state, integrator=integrator)

    @staticmethod
    def _with_applied(state: MpcState, applied: MpcCommand | None) -> MpcState:
        """``state`` whose ``last_cmd`` / newest window sample mirror what is on the fans."""
        window = state.window
        if applied is not None and window:
            newest = window[-1]
            window = (*window[:-1], WindowSample(newest.raw_temps, dict(applied.pwm)))
        return dataclasses.replace(state, last_cmd=applied, window=window)

    def _prev_pwm(self, obs: PlantObservation, cfg: MpcConfig) -> dict[str, float]:
        if self.applied_cmd is not None:
            return {ch: float(self.applied_cmd.pwm[ch]) for ch in cfg.channels}
        return resolve_prev(self.state, obs, cfg)[0]

    # -- one tick ---------------------------------------------------------

    def tick(self) -> TickResult:
        index = self.tick_count
        self.tick_count += 1
        obs, read_error = self._read()
        if read_error is not None:
            _LOG.warning("tick %d: read failed (%s); running the fallback path", index, read_error)
        self.last_obs = obs

        plan = self.supervisor.plan_tick()
        cfg = plan.cfg
        mpc_cmd: MpcCommand | None = None
        controller_error: str | None = None
        try:
            prev = self._prev_pwm(obs, cfg)
            mpc_cmd, new_state = step(obs, cfg, self._state_for_step(plan))
            cmd = self.supervisor.compose(mpc_cmd, plan, prev)
        except Exception as exc:  # controller bug: safe command, state untouched, no watchdog
            controller_error = f"{type(exc).__name__}: {exc}"
            _LOG.error("tick %d: controller failed: %s\n%s", index, exc, traceback.format_exc())
            prev = self._prev_pwm(obs, cfg) if self.applied_cmd is not None else cfg.fallback_pwm
            cmd = emergency_command(cfg, dict(prev), controller_error)
            new_state = self.state

        applied, apply_error = self._apply(cmd)
        if apply_error is not None:
            _LOG.warning("tick %d: apply failed (%s); command not committed", index, apply_error)
        if applied:
            self.applied_cmd = cmd
        if controller_error is None:
            self.state = self._with_applied(new_state, self.applied_cmd)

        watchdog_sent = False
        if controller_error is None:
            watchdog_sent = bool(self.notifier.watchdog())
            if applied and not self.ready_sent:
                self.notifier.ready()
                self.ready_sent = True

        result = TickResult(
            index=index,
            obs=obs,
            mpc_cmd=mpc_cmd,
            cmd=cmd,
            state=self.state,
            applied=applied,
            read_error=read_error,
            apply_error=apply_error,
            controller_error=controller_error,
            watchdog_sent=watchdog_sent,
        )
        self.last_result = result
        self.supervisor.record_tick(
            obs=obs if read_error is None else None,
            mpc_cmd=mpc_cmd,
            cmd=cmd,
            state=self.state,
            applied=applied,
            usb_present=read_error is None and applied,
            extra={
                "tick": index,
                "applied": applied,
                "read_error": read_error,
                "apply_error": apply_error,
                "controller_error": controller_error,
            },
        )
        if self.on_tick is not None:
            try:
                self.on_tick(result)
            except Exception:  # publishers are observers; they never fail the tick
                _LOG.exception("tick %d: on_tick hook failed", index)
        return result

    def _apply(self, cmd: MpcCommand) -> tuple[bool, str | None]:
        try:
            self.sink.apply(cmd)
        except Exception as exc:  # sink failure: log, keep running
            return False, f"{type(exc).__name__}: {exc}"
        return True, None

    # -- run / shutdown ---------------------------------------------------

    def run(self, stop_event: threading.Event, *, max_ticks: int | None = None) -> int:
        """Tick every ``cfg.dt`` until ``stop_event`` is set or ``max_ticks`` done.

        Returns the number of ticks executed in this call. Does **not** call
        :meth:`shutdown`; the caller decides (the SIGTERM path does).
        """
        done = 0
        next_at = self.clock()
        while not stop_event.is_set() and (max_ticks is None or done < max_ticks):
            self.tick()
            done += 1
            if max_ticks is not None and done >= max_ticks:
                break
            next_at += self.cfg.dt
            now = self.clock()
            delay = next_at - now
            if delay <= 0:
                if delay < -self.cfg.dt:
                    _LOG.warning("loop overran by %.2f s; resetting schedule", -delay)
                next_at = now
                continue
            if stop_event.is_set():
                break
            self._sleep(delay, stop_event)
        return done

    def shutdown(self) -> bool:
        """Write ``fallback_pwm`` once, then ``STOPPING=1``. ``True`` if the write succeeded.

        The write is deliberately not rate-limited: a hot loop running above
        ``fallback_pwm`` drops to it in one step. This is an accepted decision,
        see PROJECT.md section 9 "Stop write ignores d_pwm_max".
        """
        if self.shutdown_done:
            return False
        self.shutdown_done = True
        cmd = MpcCommand(
            pwm=dict(self.cfg.fallback_pwm),
            mode=Mode.FALLBACK,
            diagnostics={"policy": "shutdown"},
        )
        applied, error = self._apply(cmd)
        if applied:
            self.applied_cmd = cmd
            _LOG.info("shutdown: wrote fallback_pwm %s", cmd.pwm)
        else:
            _LOG.error("shutdown: writing fallback_pwm failed: %s", error)
        self.supervisor.record_tick(
            obs=None,
            mpc_cmd=None,
            cmd=cmd,
            state=None,
            applied=applied,
            usb_present=applied,
            extra={"shutdown": True, "apply_error": error},
        )
        try:
            self.notifier.stopping()
        except Exception:  # notifier must never block the stop path
            _LOG.exception("shutdown: notifier failed")
        return applied

    # -- introspection ----------------------------------------------------

    def status(self) -> dict[str, Any]:
        r = self.last_result
        return {
            "ticks": self.tick_count,
            "ready_sent": self.ready_sent,
            "applied_pwm": None if self.applied_cmd is None else dict(self.applied_cmd.pwm),
            "last_ok": None if r is None else r.ok,
            "in_fault": self.state.in_fault,
        }
