"""The model store: ``$STATE_DIRECTORY/model.json`` (plan section 6 *Persistence*, milestone 10).

Two halves, split so the controller stays pure:

* this module does the I/O. :func:`load` reads the file once at start (schema, config
  fingerprint, finiteness, age) and :func:`initial_state` hands what it found to the
  controller as ``MpcState.solver_memory[STORE_SEED_KEY]``; :class:`ModelPersister`, a
  :class:`~aqua_bridge.control.loop.Loop` ``on_tick`` observer like the recorder, writes
  the controller's current model back at most every ``model_store_interval_s`` and once
  at a clean shutdown (:meth:`ModelPersister.close`). It imports only
  :mod:`aqua_bridge.model` (the tick result is read by duck typing) and never raises out
  of ``on_tick``.
* :mod:`aqua_bridge.control.persist` (pure) applies the seed on the first zoned tick of
  ``mpc.step``: everything that needs the config's model structure (bounds of every
  coefficient, covariance checks) or the new clock happens there, with a warning and the
  prior for whatever fails. Warnings of both halves end up in ``diagnostics["store"]``
  and the persister logs them once.

Which file
----------
``--model-store PATH``, else ``$STATE_DIRECTORY/model.json`` (systemd sets
``STATE_DIRECTORY`` from ``StateDirectory=aqua-bridge`` in ``deploy/aqua-bridge.service``;
the first entry when it lists several), else no store (:func:`store_path`). Only a zoned
DAS config has a store: without ``topology`` the environment variable is ignored, so a
legacy config behaves exactly as before, and ``--model-store`` is a configuration error.

File schema (:data:`SCHEMA`, version :data:`SCHEMA_VERSION`; plain JSON, finite numbers)
---------------------------------------------------------------------------------------
::

    {"schema": "aqua-bridge-model-store", "v": 1,
     "fingerprint": sha256 of the config's structure (:func:`fingerprint`),
     "saved_wall": unix time of the snapshot,
     "thermal": solver_memory["thermal"] | null,
     "fan_curves": {fan model: {rpm_max, deadband, exponent}},
     "calibration": {bay: {serial: {"th", "P", "n", "fresh", "rms2", "used",
                                     ["inflate", "confirm",]
                                     "last_sample_wall": unix time | null,
                                     "expires_wall": unix time | null}}},
     "manual_calibration": {bay: {"th", "P", "n", "fresh", "rms2", "used",
                                     ["inflate", "confirm",]
                                     "last_sample_wall": unix time | null,
                                     "declared": {"occupied", "class", "serial"}}},
     "bays": {bay: {"occupancy", "class", "serial", "association"}},
     "ident_settle": {zone: seconds trusted and fault-free at the snapshot}}

``thermal`` is the thermal model's memory as the controller keeps it (coefficients,
covariances, zone statuses, a pending stale hold); its windows and samples are dropped
on load. ``calibration`` holds the estimator's SMART calibrations per bay and serial;
their times are converted from the controller's clock (``obs.ts``, monotonic on the
hardware) to wall time at the snapshot, so they survive a reboot:
``last_sample_wall = saved_wall - (ts - entry.ts)`` and ``expires_wall = last_sample_wall
+ calibration_max_age_days``. ``manual_calibration`` holds the estimator's *manual*
calibrations (``POST /api/calibrate``, plan section 8 items 23 and 104), one entry per
bay rather than per serial -- the operator names the bay, so there is no serial to key
them by -- with the same wall-time conversion and, instead of an ``expires_wall`` of
their own, the bay's declaration (``occupied`` / ``class`` / ``serial``) as it stood at
the snapshot. Their window is the *loading* config's
``estimator.manual_calibration_max_age_days``, and the whole rule for what comes back
lives in one place,
:func:`aqua_bridge.control.estimator.restore_manual_calibration`: an entry that is too
old, whose age is unknown, or whose bay is now declared differently is dropped, and what
survives is always restored provisional. ``bays`` is the last view per bay from the
diagnostics.
``fan_curves`` is ``solver_memory["fan_curves"]``: the PWM -> RPM curve per fan model
that ``mpc.fan_curve_online`` fits online (:mod:`aqua_bridge.control.fancurve`), empty
without the switch.
``ident_settle`` is the experiments' settle timers
(:func:`aqua_bridge.control.ident.settle_snapshot`, through the ``ident_settle``
callable :class:`ModelPersister` is constructed with -- without it the section stays
empty and a restart starts every settle timer over) as **seconds already settled**, not
times, so they need no clock conversion;
:func:`aqua_bridge.control.persist.apply_seed` subtracts the outage from them and
drops them altogether when it was too long.

The fingerprint covers ``dt``, ``channels``, ``temps``, the zones (channels, coupling,
inlet), the bay-to-zone map, the sensors' placement (role, zone, bay, redundant) and the
fans (model, count, group): what the identified coefficients mean. It leaves out policy
that does not change them (drive classes and limits, declared occupancy and serials,
gains, penalties), so tightening a limit keeps the model.

Load (:func:`load`, never raises)
---------------------------------
A missing file, an unreadable, truncated or corrupt one (not JSON, ``NaN`` /
``Infinity``, larger than :data:`MAX_FILE_BYTES`), another schema or version, another
fingerprint, or no finite ``saved_wall`` give ``source: prior``: nothing is loaded and
the reason is a warning. Otherwise the file is ``fresh`` when its age
``now_wall - saved_wall`` is at most ``model_store_max_age_days``, and ``stale`` when it
is older **or negative** (a wall clock behind the file, e.g. a Pi without RTC before NTP
has synchronised: the age is unknown, conservative). A section of the wrong shape is
dropped with a warning; each calibration entry gets ``age_s = now_wall -
last_sample_wall`` (``None`` when unknown or negative) and ``expired`` when ``now_wall``
is past its stored ``expires_wall``. The seed then goes through
:func:`aqua_bridge.control.persist.apply_seed`, which is where the owner's rule acts:
``fresh`` loads the converged zones ``frozen`` and the DAS MPC acts on them at once (its
prediction-error guard on); ``stale`` puts the thermal model in shadow with a hold that
only ``model_reconfirm_s`` of convergence with the prediction error in bounds releases,
the PI-like DAS form acts meanwhile, and the calibrations load with ``sigma_cal`` x 2
until SMART confirms them.

A fitted model (:func:`document_from_fit`, plan section 8 item 15)
------------------------------------------------------------------
``tools/fit_model.py`` writes a *report* (``kind: aqua_bridge.thermal_model``) whose
``memory`` is already the thermal memory in the shape above, but whose envelope is not a
store document. :func:`document_from_fit` converts one: the fitted memory becomes
``thermal``, the other sections are empty (a fit of a recording knows nothing about the
SMART calibrations or the bay view of the machine that loads it) and ``saved_wall`` is
the report's ``generated_at``, so a fit from last year loads ``stale`` exactly like a
store file from last year. ``tools/fit_model.py --store-out PATH`` writes the converted
document next to the report, and :func:`load` also accepts a *report* in the store's own
place, so copying the tool's ``model.json`` to ``$STATE_DIRECTORY`` works. The report
carries the store fingerprint of the config it was fitted against
(``store_fingerprint``), so a report of another machine's structure is refused on exactly
the rule a store file is refused on; a report from a tool older than that field is
refused too, and re-running the fit gives one. The one difference from a file the daemon
wrote, warned about on load: the persister replaces the file with a store document at its
first save -- keep the tool's output elsewhere.

Save (:class:`ModelPersister`)
------------------------------
Each tick the persister keeps a reference to the new state's ``solver_memory`` (plain
JSON that ``step`` never mutates afterwards), its ``last_ts`` and the wall time; nothing
is serialised until a write is due, so a tick costs a few attribute reads. A write is
due ``model_store_interval_s`` after the previous one (or after start) and at
:meth:`~ModelPersister.close`. A state whose seed has not been applied yet (the
controller raised on every tick so far) is never written, so it cannot replace a good
file. The write is atomic: a temporary file in the same directory, ``flush`` + ``fsync``,
``os.replace`` onto ``model.json``, then ``fsync`` of the directory; a failure leaves the
previous file untouched, removes the temporary file and is logged (``errors``,
``last_error``). The write runs on the loop thread (a 10-100 KB file, a few ms to tens
of ms on an SD card, every ``model_store_interval_s``; well inside ``dt = 5 s``).
The age of a model and a pending stale hold are not laundered by a save: the hold is
part of the thermal memory and the inflated calibrations carry their own counter.
"""

from __future__ import annotations

import contextlib
import hashlib
import json
import logging
import math
import os
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from aqua_bridge.model import STORE_KEY, STORE_SEED_KEY, ConfigError, MpcConfig, MpcState

__all__ = [
    "FILENAME",
    "FIT_KIND",
    "FIT_VERSION",
    "MAX_FILE_BYTES",
    "SCHEMA",
    "SCHEMA_VERSION",
    "LoadResult",
    "ModelPersister",
    "build_document",
    "document_from_fit",
    "fingerprint",
    "initial_state",
    "load",
    "store_path",
    "write_atomic",
]

_LOG = logging.getLogger("aqua_bridge.modelstore")

SCHEMA = "aqua-bridge-model-store"
SCHEMA_VERSION = 1
FILENAME = "model.json"
#: ``kind`` and ``v`` of a ``tools/fit_model.py`` report (:func:`document_from_fit`).
FIT_KIND = "aqua_bridge.thermal_model"
FIT_VERSION = 1
#: A larger file is not read (corrupt or not ours).
MAX_FILE_BYTES = 16_000_000

_DAY_S = 86400.0
_CAL_FIELDS = ("th", "P", "n", "fresh", "rms2", "used", "inflate", "confirm")


def _finite(value: object) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


# ---------------------------------------------------------------------------
# path and fingerprint
# ---------------------------------------------------------------------------


def store_path(
    cfg: MpcConfig, cli_path: str | None, env: Mapping[str, str] | None = None
) -> Path | None:
    """The store file (module docstring, *Which file*). Raises :class:`ConfigError` for
    ``--model-store`` with a legacy config."""
    env = os.environ if env is None else env
    if cli_path is not None:
        if not cfg.is_das:
            raise ConfigError("--model-store needs a zoned DAS config (mpc.topology)")
        return Path(cli_path)
    state_dir = env.get("STATE_DIRECTORY")
    if not state_dir or not cfg.is_das:
        return None
    first = state_dir.split(":")[0]
    return Path(first) / FILENAME if first else None


def fingerprint(cfg: MpcConfig) -> str:
    """sha256 over the structure of a zoned config (module docstring)."""
    topo = cfg.topology
    if topo is None:
        raise ValueError("a model store needs a zoned config (mpc.topology)")
    data = {
        "dt": cfg.dt,
        "channels": list(cfg.channels),
        "temps": list(cfg.temps),
        "zones": {
            z: [list(spec.channels), list(spec.coupled_to), spec.inlet]
            for z, spec in topo.zones.items()
        },
        "bays": {b: bay.zone for b, bay in topo.bays.items()},
        "sensors": {
            name: [spec.role, spec.zone, spec.bay, spec.redundant]
            for name, spec in cfg.sensors.items()
        },
        "fans": {ch: [spec.model, spec.count, spec.group] for ch, spec in cfg.fans.items()},
    }
    text = json.dumps(data, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode()).hexdigest()


# ---------------------------------------------------------------------------
# load
# ---------------------------------------------------------------------------


@dataclass
class LoadResult:
    """What :func:`load` found: ``source`` ``fresh`` | ``stale`` | ``prior``, the seed for
    :func:`initial_state`, the warnings and the file's age (``None`` when unknown)."""

    path: str
    source: str = "prior"
    seed: dict[str, Any] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    age_s: float | None = None
    missing: bool = False


def _reject_constant(name: str) -> float:
    raise ValueError(f"non-finite number {name}")


def _prior(result: LoadResult, warning: str | None) -> LoadResult:
    if warning is not None:
        result.warnings.append(warning)
    result.source = "prior"
    result.seed = {
        "source": "prior",
        "path": result.path,
        "age_s": result.age_s,
        "warnings": list(result.warnings),
    }
    return result


def _calibration_seed(
    raw: object, now_wall: float, warnings: list[str]
) -> dict[str, dict[str, Any]] | None:
    if not isinstance(raw, Mapping):
        warnings.append("calibration: not a mapping, section dropped")
        return None
    out: dict[str, dict[str, Any]] = {}
    for bay, per_serial in raw.items():
        if not isinstance(per_serial, Mapping):
            warnings.append(f"calibration: bay {bay!r} is not a mapping, dropped")
            continue
        entries: dict[str, Any] = {}
        for serial, entry in per_serial.items():
            if not isinstance(entry, Mapping):
                warnings.append(f"calibration: {bay}/{serial} is not a mapping, dropped")
                continue
            seeded = {k: entry[k] for k in _CAL_FIELDS if k in entry}
            last = entry.get("last_sample_wall")
            expires = entry.get("expires_wall")
            age = now_wall - float(last) if _finite(last) else None
            seeded["age_s"] = age if age is not None and age >= 0 else None
            seeded["expired"] = bool(_finite(expires) and now_wall > float(expires))
            entries[str(serial)] = seeded
        out[str(bay)] = entries
    return out


def _manual_calibration_seed(
    raw: object, now_wall: float, warnings: list[str]
) -> dict[str, dict[str, Any]] | None:
    """The ``manual_calibration`` section as
    :func:`aqua_bridge.control.estimator.restore_manual_calibration` wants it: the entry's
    fields, its ``age_s`` at load time and the bay declaration it was saved under.

    No ``expired`` flag: the window of a manual calibration is the *loading* config's
    ``estimator.manual_calibration_max_age_days``, applied in the estimator where that
    config is at hand, so the rule has exactly one home (module docstring).
    """
    if not isinstance(raw, Mapping):
        warnings.append("manual_calibration: not a mapping, section dropped")
        return None
    out: dict[str, dict[str, Any]] = {}
    for bay, entry in raw.items():
        if not isinstance(entry, Mapping):
            warnings.append(f"manual_calibration: bay {bay!r} is not a mapping, dropped")
            continue
        seeded = {k: entry[k] for k in _CAL_FIELDS if k in entry}
        last = entry.get("last_sample_wall")
        age = now_wall - float(last) if _finite(last) else None
        seeded["age_s"] = age if age is not None and age >= 0 else None
        declared = entry.get("declared")
        seeded["declared"] = dict(declared) if isinstance(declared, Mapping) else None
        out[str(bay)] = seeded
    return out


def load(path: str | os.PathLike[str], cfg: MpcConfig, *, now_wall: float) -> LoadResult:
    """Read the store file (module docstring, *Load*). Never raises."""
    result = LoadResult(path=str(path))
    try:
        p = Path(path)
        if not p.exists():
            result.missing = True
            return _prior(result, None)
        size = p.stat().st_size
        if size > MAX_FILE_BYTES:
            return _prior(result, f"store: {p} is {size} bytes, not read")
        try:
            doc = json.loads(p.read_bytes().decode("utf-8"), parse_constant=_reject_constant)
        except (OSError, UnicodeDecodeError, ValueError) as exc:
            return _prior(result, f"store: {p} is unreadable or corrupt ({exc})"[:300])
        if not isinstance(doc, dict):
            return _prior(result, "store: the file is not a JSON object")
        if doc.get("kind") == FIT_KIND:  # a tools/fit_model.py report (module docstring)
            try:
                doc = document_from_fit(cfg, doc)
            except ValueError as exc:
                return _prior(result, f"store: {p} is a fit report but {exc}")
            result.warnings.append(
                f"store: {p} is a tools/fit_model.py report; its fitted model is loaded "
                "and the other sections start at their prior. The daemon replaces the "
                "file with a store document at its first save."
            )
        if doc.get("schema") != SCHEMA or doc.get("v") != SCHEMA_VERSION:
            return _prior(result, f"store: unknown schema {doc.get('schema')!r} v{doc.get('v')!r}")
        if doc.get("fingerprint") != fingerprint(cfg):
            return _prior(
                result, "store: the file was written for another config structure, not loaded"
            )
        saved = doc.get("saved_wall")
        if not _finite(saved):
            return _prior(result, "store: saved_wall is missing or not a finite number")
        age = now_wall - float(saved)
        if age < 0:
            result.warnings.append(
                f"store: the file is {-age:.0f} s newer than the wall clock (clock not set?); "
                "its age is unknown, loading it stale"
            )
            result.source = "stale"
        elif age > cfg.model_store_max_age_days * _DAY_S:
            result.source = "stale"
            result.age_s = age
        else:
            result.source = "fresh"
            result.age_s = age
        seed: dict[str, Any] = {
            "source": result.source,
            "path": result.path,
            "age_s": result.age_s,
        }
        thermal = doc.get("thermal")
        if thermal is not None and not isinstance(thermal, dict):
            result.warnings.append("thermal: not a mapping, section dropped")
            thermal = None
        seed["thermal"] = thermal
        if doc.get("calibration") is not None:
            seed["calibration"] = _calibration_seed(doc["calibration"], now_wall, result.warnings)
        if doc.get("manual_calibration") is not None:
            seed["manual_calibration"] = _manual_calibration_seed(
                doc["manual_calibration"], now_wall, result.warnings
            )
        for name in ("fan_curves", "bays", "ident_settle"):
            value = doc.get(name)
            if value is None:
                continue
            if isinstance(value, dict):
                seed[name] = value
            else:
                result.warnings.append(f"{name}: not a mapping, section dropped")
        seed["warnings"] = list(result.warnings)
        result.seed = seed
        return result
    except Exception as exc:  # never raises: the daemon starts on the prior
        result.warnings = []
        return _prior(result, f"store: not loaded ({type(exc).__name__}: {exc})"[:300])


def initial_state(result: LoadResult | None) -> MpcState:
    """The controller's first state: cold, carrying the seed for ``mpc.step``."""
    if result is None or not result.seed:
        return MpcState.cold()
    return MpcState(solver_memory={STORE_SEED_KEY: result.seed})


# ---------------------------------------------------------------------------
# save
# ---------------------------------------------------------------------------


def build_document(
    cfg: MpcConfig,
    memory: Mapping[str, Any],
    *,
    ts: float | None,
    wall: float,
    bays: Mapping[str, Any] | None = None,
    ident_settle: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """The file content for a snapshot of ``solver_memory`` taken at controller time
    ``ts`` and wall time ``wall`` (module docstring, *File schema*)."""
    thermal = memory.get("thermal")
    est = memory.get("estimator")
    max_age = cfg.estimator.calibration_max_age_days * _DAY_S if cfg.estimator is not None else None
    calibration: dict[str, dict[str, Any]] = {}
    cal = est.get("cal") if isinstance(est, Mapping) else None
    if isinstance(cal, Mapping):
        for bay, per_serial in cal.items():
            if not isinstance(per_serial, Mapping):
                continue
            out: dict[str, Any] = {}
            for serial, entry in per_serial.items():
                if not isinstance(entry, Mapping):
                    continue
                saved = {k: entry[k] for k in _CAL_FIELDS if k in entry}
                entry_ts = entry.get("ts")
                last = None
                if _finite(entry_ts) and _finite(ts) and float(ts) >= float(entry_ts):  # type: ignore[arg-type]
                    last = wall - (float(ts) - float(entry_ts))  # type: ignore[arg-type]
                saved["last_sample_wall"] = last
                saved["expires_wall"] = None if last is None or max_age is None else last + max_age
                out[str(serial)] = saved
            if out:
                calibration[str(bay)] = out
    manual_calibration: dict[str, Any] = {}
    manual = est.get("manual") if isinstance(est, Mapping) else None
    if isinstance(manual, Mapping) and cfg.topology is not None:
        for bay, known in manual.items():
            if not isinstance(known, Mapping):
                continue
            entry = known.get("cal")
            if not isinstance(entry, Mapping) or str(bay) not in cfg.topology.bays:
                continue  # a bay with a pending reading but no map yet carries nothing
            saved = {k: entry[k] for k in _CAL_FIELDS if k in entry}
            entry_ts = entry.get("ts")
            last = None
            if _finite(entry_ts) and _finite(ts) and float(ts) >= float(entry_ts):  # type: ignore[arg-type]
                last = wall - (float(ts) - float(entry_ts))  # type: ignore[arg-type]
            saved["last_sample_wall"] = last
            saved["declared"] = cfg.bay_declaration(str(bay))
            manual_calibration[str(bay)] = saved
    bays_out: dict[str, Any] = {}
    for bay, info in (bays or {}).items():
        if isinstance(info, Mapping):
            bays_out[str(bay)] = {
                k: info.get(k) for k in ("occupancy", "class", "serial", "association")
            }
    settle_out = {
        str(zone): float(value)
        for zone, value in (ident_settle or {}).items()
        if _finite(value) and float(value) >= 0.0
    }
    curves = memory.get("fan_curves")
    return {
        "schema": SCHEMA,
        "v": SCHEMA_VERSION,
        "fingerprint": fingerprint(cfg),
        "saved_wall": float(wall),
        "thermal": dict(thermal) if isinstance(thermal, Mapping) else None,
        "fan_curves": dict(curves) if isinstance(curves, Mapping) else {},
        "calibration": calibration,
        "manual_calibration": manual_calibration,
        "bays": bays_out,
        "ident_settle": settle_out,
    }


def document_from_fit(
    cfg: MpcConfig, payload: Mapping[str, Any], *, wall: float | None = None
) -> dict[str, Any]:
    """A store document from a ``tools/fit_model.py`` report (module docstring, *A fitted
    model*).

    ``payload`` is the report as the tool writes it (``kind``/``v``, ``memory`` -- which
    is already in exactly the shape ``solver_memory["thermal"]`` uses -- and
    ``store_fingerprint``, :func:`fingerprint` of the config the fit ran against). The
    thermal section is that memory; ``fan_curves``, ``calibration``,
    ``manual_calibration`` and ``bays`` are empty, because a fit of a recording knows
    nothing about the calibrations or the bay view of the machine that will load it.
    ``wall`` defaults to the report's
    ``generated_at``, so the file's age -- and with it the ``fresh`` / ``stale`` rule --
    is the age of the *fit*, not of the conversion.

    The document's ``fingerprint`` is the report's ``store_fingerprint``, never
    :func:`fingerprint` of the config doing the conversion: that is what makes
    :func:`load`'s fingerprint check bite on a report as it does on a store file, so a
    fit of another machine's structure is refused rather than loaded. A report without
    it (written before it was recorded) raises ``ValueError`` -- re-run the fit.

    Raises ``ValueError`` for anything that is not such a report.
    """
    if not isinstance(payload, Mapping) or payload.get("kind") != FIT_KIND:
        raise ValueError(f"not an {FIT_KIND} report")
    if payload.get("v") != FIT_VERSION:
        raise ValueError(f"its version is {payload.get('v')!r}, not {FIT_VERSION}")
    memory = payload.get("memory")
    if not isinstance(memory, Mapping):
        raise ValueError("it carries no thermal memory")
    fp = payload.get("store_fingerprint")
    if not isinstance(fp, str) or not fp:
        raise ValueError(
            "it carries no store_fingerprint (a report from an older tools/fit_model.py; "
            "re-run the fit to get one)"
        )
    if fp != fingerprint(cfg):
        raise ValueError("it was fitted against another config structure")
    if wall is None:
        generated = payload.get("generated_at")
        if not _finite(generated):
            raise ValueError("it has no finite generated_at")
        wall = float(generated)  # type: ignore[arg-type]
    return {
        "schema": SCHEMA,
        "v": SCHEMA_VERSION,
        "fingerprint": fp,
        "saved_wall": float(wall),
        "thermal": dict(memory),
        "fan_curves": {},
        "calibration": {},
        "manual_calibration": {},
        "bays": {},
    }


def write_atomic(path: str | os.PathLike[str], data: bytes) -> None:
    """Write ``data`` to ``path`` atomically: temporary file in the same directory,
    ``fsync``, ``os.replace``, ``fsync`` of the directory. On failure the previous file is
    untouched, the temporary file is removed and the exception propagates."""
    target = Path(path)
    tmp = target.with_name(f".{target.name}.{os.getpid()}.tmp")
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
    try:
        try:
            view = memoryview(data)
            while view:
                written = os.write(fd, view)
                view = view[written:]
            os.fsync(fd)
        finally:
            os.close(fd)
        os.replace(tmp, target)
    except BaseException:
        with contextlib.suppress(OSError):
            tmp.unlink()
        raise
    try:
        dir_fd = os.open(target.parent, os.O_RDONLY)
    except OSError:  # a platform without directory handles: the replace itself is atomic
        return
    try:
        os.fsync(dir_fd)
    except OSError:
        pass
    finally:
        os.close(dir_fd)


class ModelPersister:
    """``Loop.on_tick`` observer that keeps ``model.json`` current (module docstring, *Save*)."""

    def __init__(
        self,
        cfg: MpcConfig,
        path: str | os.PathLike[str],
        *,
        clock: Callable[[], float] = time.monotonic,
        wall: Callable[[], float] = time.time,
        interval_s: float | None = None,
        ident_settle: Callable[[], Mapping[str, Any] | None] | None = None,
    ) -> None:
        if not cfg.is_das:
            raise ValueError("a model store needs a zoned config (mpc.topology)")
        self.cfg = cfg
        self.path = Path(path)
        self.clock = clock
        self.wall = wall
        self.interval_s = float(cfg.model_store_interval_s if interval_s is None else interval_s)
        #: Called on every captured tick for the experiments' settle timers (module
        #: docstring); ``None`` leaves that section empty.
        self.ident_settle = ident_settle
        self.writes = 0
        self.errors = 0
        self.last_error: str | None = None
        self._last_write = clock()
        self._snapshot: tuple[Mapping[str, Any], float | None, float] | None = None
        self._bays: Mapping[str, Any] | None = None
        self._settle: Mapping[str, Any] | None = None
        self._dirty = False
        self._reported = False

    def on_tick(self, result: Any) -> None:
        """Capture the tick's state; write when due. Never raises."""
        try:
            self._capture(result)
            if self._dirty and self.clock() - self._last_write >= self.interval_s:
                self.save()
        except Exception:  # an observer never fails the tick
            _LOG.exception("model store: on_tick failed")

    def _capture(self, result: Any) -> None:
        state = getattr(result, "state", None)
        memory = getattr(state, "solver_memory", None)
        if not isinstance(memory, Mapping) or STORE_SEED_KEY in memory:
            return  # nothing applied yet: never replace the file with an empty model
        self._report(memory.get(STORE_KEY))
        cmd = getattr(result, "mpc_cmd", None)
        diagnostics = getattr(cmd, "diagnostics", None)
        if isinstance(diagnostics, Mapping) and isinstance(diagnostics.get("bays"), Mapping):
            self._bays = diagnostics["bays"]
        if self.ident_settle is not None:
            settle = self.ident_settle()
            self._settle = settle if isinstance(settle, Mapping) else None
        ts = memory.get("last_ts")
        self._snapshot = (memory, float(ts) if _finite(ts) else None, float(self.wall()))
        self._dirty = True

    def _report(self, store: object) -> None:
        if self._reported or not isinstance(store, Mapping):
            return
        self._reported = True
        _LOG.info(
            "model store: %s loaded as %s (sections %s)",
            store.get("path"),
            store.get("source"),
            store.get("sections"),
        )
        for warning in store.get("warnings") or ():
            _LOG.warning("model store: %s", warning)

    def save(self) -> bool:
        """Write the latest snapshot now (atomic). ``True`` when written. Never raises."""
        if self._snapshot is None:
            return False
        memory, ts, wall = self._snapshot
        self._last_write = self.clock()
        try:
            doc = build_document(
                self.cfg, memory, ts=ts, wall=wall, bays=self._bays, ident_settle=self._settle
            )
            data = json.dumps(doc, allow_nan=False, separators=(",", ":")).encode()
            write_atomic(self.path, data)
        except Exception as exc:  # a disk error must not touch the loop
            self.errors += 1
            self.last_error = f"{type(exc).__name__}: {exc}"[:300]
            _LOG.warning("model store: writing %s failed: %s", self.path, self.last_error)
            return False
        self.writes += 1
        self._dirty = False
        return True

    def close(self) -> bool:
        """Clean shutdown: write the latest snapshot if it changed since the last write."""
        if not self._dirty:
            return False
        return self.save()
