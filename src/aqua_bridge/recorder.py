"""Record every tick to JSONL for offline identification (plan sections 5, 10, 11 milestone 8).

:class:`Recorder` is a :class:`~aqua_bridge.control.loop.Loop` ``on_tick`` observer
exactly like the MQTT state publisher: it is called on the loop thread after the
supervisor snapshot has been updated, with the tick's :class:`~aqua_bridge.control.
loop.TickResult`, and it must never raise and never do anything that could hang --
``Loop.tick`` already wraps the call in ``try/except`` (an exception here is logged
and dropped, the loop keeps ticking), and this module wraps its own body a second
time so a broken record or a disk error never surfaces past ``on_tick`` at all. The
only I/O is a local, append-only file write with rotation, so nothing here can grow
without bound or stall on the network -- the two ways an observer could turn into a
tick-blocking hazard.

Wiring
------
``__main__.py`` builds one :class:`Recorder` per run from ``--record PATH`` (highest
priority) or the top-level config key ``record_path`` (kept in
:attr:`~aqua_bridge.config.AppConfig.extra`, the same place the interim ``sim.das``
section lives before it has a typed home -- a plain top-level scalar needs no change
to ``config.py``/``model.py``, and a config without it behaves exactly as before:
no recorder is built). ``record_max_bytes``/``record_backup_count`` size the
rotation the same way. Recording composes with the MQTT state publisher's own
``on_tick``: both are called every tick (:func:`aqua_bridge.recorder.chain_on_tick`).

Record shape (one JSON object per line, schema :data:`RECORD_VERSION`)
------------------------------------------------------------------------
Every field a record needs to *replay* the tick through
:func:`aqua_bridge.control.thermal.update` (:func:`thermal_inputs`) is taken
directly from the fields ``mpc.step`` already computed for that tick --
``diagnostics["prev_pwm"]`` (the command the thermal model treats as ``u`` in
effect over the interval ending at this tick), ``diagnostics["gate"]``
(``filtered``/``per_temp``) restricted to ``time.status in (ok, first)`` for
``trusted_temps``, ``diagnostics["zones"]`` (``trusted`` and not ``fault``) for
``zones_ok``, and ``diagnostics["bays"]`` (occupancy, class, calibration, serial)
verbatim -- the exact shape ``control.mpc._thermal_shadow`` reads from
``EstimatorUpdate.bays`` live. A tick whose controller raised (``TickResult.
controller_error`` set) carries none of the DAS diagnostics keys (the emergency
command's diagnostics are just ``{policy, controller_error}``); its record still
gets written, with empty ``trusted_temps``/``zones_ok``/``bays``, so a reader can
see the gap and treat it as a break in the sequence the same way a dropped
interval already does.

Fields: ``v`` (schema version), ``i`` (tick index), ``ts``, ``das`` (whether
``cfg.is_das``), ``mode``, ``applied``, ``temps``/``rpm``/``pwm`` (raw, as read),
``prev`` (the command in effect this tick), ``cmd`` (the command computed this
tick, which becomes next tick's ``prev``), ``trusted_temps``, ``zones_ok``,
``bays``, ``read_error``, ``controller_error``. Legacy-mode recordings (``das:
false``) still carry ``temps``/``rpm``/``pwm``/``prev``/``cmd`` -- enough for
``tools/fit_fans.py``, which needs no topology.
"""

from __future__ import annotations

import json
import logging
import math
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

from aqua_bridge.control.loop import TickResult
from aqua_bridge.model import MpcConfig

__all__ = [
    "DEFAULT_BACKUP_COUNT",
    "DEFAULT_MAX_BYTES",
    "RECORD_VERSION",
    "Recorder",
    "chain_on_tick",
    "iter_records",
    "record_from_tick",
    "thermal_inputs",
]

_LOG = logging.getLogger("aqua_bridge.recorder")

RECORD_VERSION = 1
#: Rotate at ~20 MB (a season of dt=5s DAS ticks at ~1 KB/line is a few days per file).
DEFAULT_MAX_BYTES = 20_000_000
DEFAULT_BACKUP_COUNT = 5


def _finite(value: object) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _sanitize_map(values: Mapping[str, Any] | None) -> dict[str, float | None]:
    """A ``str -> float`` mapping with ``None``/NaN/Inf entries as JSON ``null``."""
    out: dict[str, float | None] = {}
    for key, value in (values or {}).items():
        out[str(key)] = float(value) if _finite(value) else None
    return out


def _mapping(value: object) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


# ---------------------------------------------------------------------------
# record <-> tick
# ---------------------------------------------------------------------------


def record_from_tick(result: TickResult, cfg: MpcConfig) -> dict[str, Any]:
    """The JSON-able record for one :class:`TickResult` (module docstring). Pure:
    reads only ``result`` and ``cfg``, raises only on a structurally broken
    ``result`` (not a :class:`TickResult`, or a non-mapping ``diagnostics``)."""
    diagnostics = result.cmd.diagnostics if isinstance(result.cmd.diagnostics, Mapping) else {}
    das = bool(cfg.is_das)
    time_info = diagnostics.get("time")
    time_status = time_info.get("status") if isinstance(time_info, Mapping) else None
    gate = diagnostics.get("gate")
    filtered = _mapping(gate.get("filtered")) if isinstance(gate, Mapping) else {}
    per_temp = _mapping(gate.get("per_temp")) if isinstance(gate, Mapping) else {}
    trusted_temps: dict[str, float] = {}
    if das and time_status in ("ok", "first"):
        for name in cfg.temps:
            if per_temp.get(name) and _finite(filtered.get(name)):
                trusted_temps[name] = float(filtered[name])
    zones_ok: list[str] = []
    zones_diag = diagnostics.get("zones")
    if das and isinstance(zones_diag, Mapping):
        zones_ok = sorted(
            z
            for z, info in zones_diag.items()
            if isinstance(info, Mapping) and info.get("trusted") and not info.get("fault")
        )
    bays_diag = diagnostics.get("bays")
    bays = _mapping(bays_diag) if das else {}
    prev = _mapping(diagnostics.get("prev_pwm"))
    return {
        "v": RECORD_VERSION,
        "i": int(result.index),
        "ts": float(result.obs.ts),
        "das": das,
        "mode": result.cmd.mode.value,
        "applied": bool(result.applied),
        "temps": _sanitize_map(result.obs.temps),
        "rpm": _sanitize_map(result.obs.rpm),
        "pwm": _sanitize_map(result.obs.pwm),
        "prev": _sanitize_map(prev),
        "cmd": _sanitize_map(result.cmd.pwm),
        "trusted_temps": {k: float(v) for k, v in trusted_temps.items()},
        "zones_ok": zones_ok,
        "bays": bays,
        "read_error": result.read_error,
        "controller_error": result.controller_error,
    }


def thermal_inputs(cfg: MpcConfig, record: Mapping[str, Any]) -> dict[str, Any]:
    """``kwargs`` for :func:`aqua_bridge.control.thermal.update` from one decoded record.

    Extracts ``occupancy``/``classes``/``maps`` from ``record["bays"]`` exactly the
    way ``control.mpc._thermal_shadow`` extracts them live from ``EstimatorUpdate.
    bays`` (module docstring); a malformed per-bay entry is skipped rather than
    raising, since a recording is untrusted input once it has left the daemon.
    """
    occupancy: dict[str, str] = {}
    classes: dict[str, str] = {}
    maps: dict[str, tuple[float, float]] = {}
    for bay, info in _mapping(record.get("bays")).items():
        if not isinstance(info, Mapping):
            continue
        if "occupancy" in info:
            occupancy[str(bay)] = str(info["occupancy"])
        if "class" in info:
            classes[str(bay)] = str(info["class"])
        cal = info.get("calibration")
        if info.get("serial") is not None and isinstance(cal, Mapping) and cal.get("accepted_once"):
            slope, offset = cal.get("slope"), cal.get("offset_c")
            if _finite(slope) and _finite(offset):
                maps[str(bay)] = (float(slope), float(offset))
    return {
        "temps": {k: float(v) for k, v in _mapping(record.get("trusted_temps")).items()},
        "u": {k: float(v) for k, v in _mapping(record.get("prev")).items() if _finite(v)},
        "ts": float(record["ts"]),
        "zones_ok": [str(z) for z in record.get("zones_ok") or ()],
        "occupancy": occupancy or None,
        "maps": maps or None,
        "classes": classes or None,
        "rpm": {k: v for k, v in _mapping(record.get("rpm")).items()},
    }


def iter_records(path: str | Path) -> list[dict[str, Any]]:
    """Every well-formed JSON object line of a recording (or several rotated files
    concatenated by the caller); a malformed line is skipped with a warning, never
    raised -- a recording is a log, not a contract, and one bad line must not sink
    the rest of it."""
    out: list[dict[str, Any]] = []
    path = Path(path)
    with path.open("r", encoding="utf-8") as fh:
        for lineno, line in enumerate(fh, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:
                _LOG.warning("%s:%d: not valid JSON (%s), skipped", path, lineno, exc)
                continue
            if not isinstance(obj, Mapping):
                _LOG.warning("%s:%d: not a JSON object, skipped", path, lineno)
                continue
            out.append(dict(obj))
    return out


# ---------------------------------------------------------------------------
# the observer
# ---------------------------------------------------------------------------


class Recorder:
    """Appends one JSON line per tick to ``path``, rotating like ``logging``'s
    ``RotatingFileHandler`` (``path``, ``path.1``, ``path.2``, ... oldest dropped past
    ``backup_count``). ``on_tick`` matches ``Loop``'s ``on_tick`` contract exactly and
    never raises: a broken record or a disk error is logged and the tick continues
    (``Loop.tick`` already wraps the call too, so this is defence in depth, not the
    only line of it). Not thread-safe: use one ``Recorder`` per ``Loop`` instance, as
    ``on_tick`` already implies (the loop calls it from one thread only).
    """

    def __init__(
        self,
        cfg: MpcConfig,
        path: str | Path,
        *,
        max_bytes: int = DEFAULT_MAX_BYTES,
        backup_count: int = DEFAULT_BACKUP_COUNT,
    ) -> None:
        self.cfg = cfg
        self.path = Path(path)
        self.max_bytes = int(max_bytes)
        self.backup_count = int(backup_count)
        self._fh: Any = None
        self._closed = False

    def on_tick(self, result: TickResult) -> None:
        if self._closed:
            return
        try:
            line = json.dumps(
                record_from_tick(result, self.cfg), allow_nan=False, separators=(",", ":")
            )
        except Exception:  # a broken record must never stop the tick
            index = getattr(result, "index", "?")
            _LOG.exception("recorder: could not build a record for tick %s", index)
            return
        try:
            self._write_line(line)
        except OSError:
            _LOG.exception("recorder: write to %s failed", self.path)

    def _write_line(self, line: str) -> None:
        if self._fh is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._fh = self.path.open("a", encoding="utf-8")
        self._fh.write(line)
        self._fh.write("\n")
        self._fh.flush()
        if self.max_bytes > 0 and self._fh.tell() >= self.max_bytes:
            self._rotate()

    def _rotate(self) -> None:
        self._fh.close()
        self._fh = None
        if self.backup_count > 0:
            oldest = self.path.with_name(f"{self.path.name}.{self.backup_count}")
            if oldest.exists():
                oldest.unlink()
            for i in range(self.backup_count - 1, 0, -1):
                src = self.path.with_name(f"{self.path.name}.{i}")
                if src.exists():
                    src.rename(self.path.with_name(f"{self.path.name}.{i + 1}"))
            self.path.rename(self.path.with_name(f"{self.path.name}.1"))
        else:
            self.path.unlink(missing_ok=True)
        # Reopen a fresh empty file right away (rather than lazily on the next
        # write) so ``path`` always exists once recording has started, even if
        # the run stops immediately after a rotation.
        self._fh = self.path.open("a", encoding="utf-8")

    def close(self) -> None:
        """Flush and close the open file (idempotent); further ``on_tick`` calls no-op."""
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        self._closed = True


def chain_on_tick(*hooks: Callable[[TickResult], None] | None) -> Callable[[TickResult], None]:
    """One ``on_tick`` that calls every non-``None`` hook, each isolated by its own
    ``try/except`` -- so, for example, the MQTT publisher and a :class:`Recorder` run
    side by side and one's exception never stops the other (``Loop.tick`` already
    isolates the combined call from the tick itself; this isolates the hooks from
    each other)."""
    active = [h for h in hooks if h is not None]

    def _run(result: TickResult) -> None:
        for hook in active:
            try:
                hook(result)
            except Exception:
                _LOG.exception("on_tick hook %r failed", hook)

    return _run
