"""Apply a model loaded from the store to the controller's memory (plan section 6, *Persistence*).

Pure: :func:`apply_seed` is a function of its arguments, reads no clock and does no
I/O. :mod:`aqua_bridge.modelstore` reads ``model.json`` (file schema, fingerprint, age,
finiteness) and puts what it found into the initial state as
``solver_memory[STORE_SEED_KEY]``; ``mpc.step`` calls :func:`apply_seed` on its first
zoned tick, before the estimator, with that tick's ``obs.ts``. Everything that needs the
running config's structure or the new clock happens here, so it is deterministic and
testable without files:

* ``thermal`` -- :func:`aqua_bridge.control.thermal.restore` (structure, bounds,
  finiteness, covariance; zone statuses ``frozen`` for a fresh file, a stale hold for a
  stale one). Installed only with ``model_shadow`` (without it the thermal memory does
  not exist and the section is ignored with a warning). A section that fails its checks
  is dropped with a warning: the model starts at its prior.
* ``calibration`` -- :func:`aqua_bridge.control.estimator.restore_calibration` (per bay
  and serial; each bad entry is dropped with a warning; the time of each entry is
  re-based on ``ts``; a stale file inflates ``sigma_cal``).
* ``fan_curves`` -- per fan model ``{rpm_max, deadband, exponent}`` inside the bounds of
  ``fan_models`` validation, kept in ``solver_memory["fan_curves"]``. With
  ``mpc.fan_curve_online`` the online fit (:mod:`aqua_bridge.control.fancurve`) keeps
  that section current and the thermal model and the DAS MPC plan on it in place of the
  ``fan_models`` entry, so a fitted curve survives a restart through the store. Without
  the switch the section is still loaded and saved, but nothing reads it: the configured
  curves are used.
* ``bays`` -- the last occupancy, class, serial and association per bay, reported only.
  Conservative choice: occupancy restarts ``unknown`` (a drive may have been inserted
  while the daemon was down) and associations by correlation are formed again (drives
  may have been swapped); a calibration comes back when its serial is associated with
  its bay again.

The result, ``solver_memory[STORE_KEY]`` and ``diagnostics["store"]``::

    {"source": "fresh" | "stale" | "prior", "path": str | None, "age_s": float | None,
     "loaded_ts": ts, "sections": {"thermal": "loaded" | "dropped" | "ignored" | "absent",
     "calibration": int entries, "fan_curves": int, "bays": int},
     "warnings": [str, ...], "bays": {bay: {...}}}

``source`` is what the file was; a ``fresh`` file whose thermal section was dropped
still reports ``fresh`` with that section ``dropped``. A missing or unusable file comes as
a seed with ``source: prior`` and the reasons as warnings, so they show in the
diagnostics too. A malformed seed never raises: it reports ``prior`` with a warning and
installs nothing.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from aqua_bridge.control import estimator, thermal
from aqua_bridge.model import STORE_KEY, MpcConfig

__all__ = ["MAX_WARNINGS", "SOURCES", "apply_seed"]

#: What a store file was: younger than ``model_store_max_age_days``, older, or unusable.
SOURCES: tuple[str, ...] = ("fresh", "stale", "prior")
#: Warnings kept in the summary (the rest are counted).
MAX_WARNINGS = 20

_BAY_FIELDS = ("occupancy", "class", "serial", "association")


def _finite(value: object) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _fan_curves(raw: object, cfg: MpcConfig, warnings: list[str]) -> dict[str, dict[str, float]]:
    out: dict[str, dict[str, float]] = {}
    if not isinstance(raw, Mapping):
        warnings.append("fan_curves: not a mapping, section dropped")
        return out
    for name, curve in raw.items():
        if name not in cfg.fan_models:
            warnings.append(f"fan_curves: fan model {name!r} is not configured, dropped")
            continue
        if not isinstance(curve, Mapping):
            warnings.append(f"fan_curves: {name!r} is not a mapping, dropped")
            continue
        rpm_max, deadband, exponent = (curve.get(k) for k in ("rpm_max", "deadband", "exponent"))
        if not (_finite(rpm_max) and _finite(deadband) and _finite(exponent)):
            warnings.append(f"fan_curves: {name!r} has a missing or non-finite value, dropped")
            continue
        if not (
            float(rpm_max) > 0  # type: ignore[arg-type]
            and 0.0 <= float(deadband) < 0.5  # type: ignore[arg-type]
            and 0.5 <= float(exponent) <= 1.5  # type: ignore[arg-type]
        ):
            warnings.append(f"fan_curves: {name!r} is out of bounds, dropped")
            continue
        out[str(name)] = {
            "rpm_max": float(rpm_max),  # type: ignore[arg-type]
            "deadband": float(deadband),  # type: ignore[arg-type]
            "exponent": float(exponent),  # type: ignore[arg-type]
        }
    return out


def _bays(raw: object, cfg: MpcConfig, warnings: list[str]) -> dict[str, dict[str, Any]]:
    out: dict[str, dict[str, Any]] = {}
    if not isinstance(raw, Mapping):
        warnings.append("bays: not a mapping, section dropped")
        return out
    assert cfg.topology is not None
    for bay, info in raw.items():
        if bay not in cfg.topology.bays or not isinstance(info, Mapping):
            warnings.append(f"bays: {bay!r} is not a bay of the topology, dropped")
            continue
        entry = {}
        for key in _BAY_FIELDS:
            value = info.get(key)
            entry[key] = value if value is None or isinstance(value, str) else None
        out[str(bay)] = entry
    return out


def apply_seed(mem: dict[str, Any], cfg: MpcConfig, seed: object, ts: float) -> dict[str, Any]:
    """Install a store seed into ``mem`` (``solver_memory``, modified in place) and return
    the store summary, also kept as ``mem[STORE_KEY]`` (module docstring). Never raises."""
    warnings: list[str] = []
    sections: dict[str, Any] = {"thermal": "absent", "calibration": 0, "fan_curves": 0, "bays": 0}
    summary: dict[str, Any] = {
        "source": "prior",
        "path": None,
        "age_s": None,
        "loaded_ts": float(ts),
        "sections": sections,
        "warnings": warnings,
        "bays": {},
    }
    staged: dict[str, Any] = {}
    try:
        if cfg.topology is None:
            raise ValueError("a model store needs a zoned config")
        if not isinstance(seed, Mapping):
            raise ValueError("the loaded model is not a mapping")
        source = seed.get("source")
        if source not in SOURCES:
            raise ValueError(f"unknown source {source!r}")
        stale = source == "stale"
        path = seed.get("path")
        summary["path"] = path if isinstance(path, str) else None
        age = seed.get("age_s")
        summary["age_s"] = float(age) if _finite(age) else None
        for w in seed.get("warnings") or ():
            warnings.append(str(w))
        summary["source"] = source
        if source == "prior":  # no usable file: only its warnings are reported
            seed = {}

        stored = seed.get("thermal")
        if stored is not None:
            if not cfg.model_shadow:
                sections["thermal"] = "ignored"
                warnings.append("thermal: not loaded, model_shadow is off")
            else:
                try:
                    staged["thermal"] = thermal.restore(stored, cfg, stale=stale)
                    sections["thermal"] = "loaded"
                except ValueError as exc:
                    sections["thermal"] = "dropped"
                    warnings.append(f"{exc}; the thermal model starts at its prior")

        cal = seed.get("calibration")
        if cal is not None:
            est_mem, cal_warnings = estimator.restore_calibration(
                mem.get("estimator"), cal, cfg, ts=ts, stale=stale
            )
            staged["estimator"] = est_mem
            warnings.extend(cal_warnings)
            sections["calibration"] = sum(len(v) for v in est_mem["cal"].values())

        curves = seed.get("fan_curves")
        if curves is not None:
            staged["fan_curves"] = _fan_curves(curves, cfg, warnings)
            sections["fan_curves"] = len(staged["fan_curves"])

        bays = seed.get("bays")
        if bays is not None:
            summary["bays"] = _bays(bays, cfg, warnings)
            sections["bays"] = len(summary["bays"])
        mem.update(staged)
    except Exception as exc:  # a malformed seed never raises out of step (nothing installed)
        summary["source"] = "prior"
        summary["bays"] = {}
        sections.update(thermal="absent", calibration=0, fan_curves=0, bays=0)
        warnings.append(f"store: not applied ({type(exc).__name__}: {exc})"[:300])
    if len(warnings) > MAX_WARNINGS:
        extra = len(warnings) - MAX_WARNINGS
        del warnings[MAX_WARNINGS:]
        warnings.append(f"... and {extra} more")
    summary["warnings"] = [w[:300] for w in warnings]
    mem[STORE_KEY] = summary
    return summary
