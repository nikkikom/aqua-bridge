#!/usr/bin/env python3
"""Fit the zoned thermal model offline from a recording (plan sections 5, 10, 11 milestone 8).

::

    python tools/fit_model.py --config config.example-das.yaml --topology \\
        --out model.json rec1.jsonl [rec2.jsonl ...]

Three passes, in order:

1. **Windowed least squares per zone.** Every DAS record of the training split is
   replayed through :func:`aqua_bridge.control.thermal.update` (``learn=True``), the
   exact online identification ``mpc.step`` runs live -- offline, this is a batch
   windowed-RLS fit over the whole recording rather than one tick at a time. Its
   result is ``theta`` (every key of ``thermal.PARAMETERS``) plus, per zone/bay
   block, the relative standard error :func:`aqua_bridge.control.thermal.summary`
   already reports.
2. **Output-error refinement**, zone-air block only. The windowed pass is an
   *equation-error* fit: each window regresses an integral of the ODE, never a free
   run of the model. This pass instead free-runs the zone-air state ``T_a`` alone
   (the only node the plan's sensor list actually measures) for short horizons,
   seeded from measurement at each horizon's start, exogenous inlet/neighbour-air/
   proximal-heat inputs taken from the recording at every tick, and adjusts
   ``E``/``leak``/``kappa``/``p_air`` by bounded coordinate descent to reduce the
   *simulated* trajectory's error against the *measured* one -- the quantity that
   actually matters for the MPC milestone's prediction quality. The per-bay
   ``g0``/``k``/``q_s`` stay at their windowed-LS values: refining them by output
   error would need free-running the *latent, unmeasured* drive node, which is
   exactly the "a static mixing map applied to a lagging sensor makes a fan
   increase look like drive heating" hazard the plan's own prototype flagged
   (deviation, documented in the module docstring below and the PR).
3. **Hold-out validation.** The last ``--holdout-frac`` of the recording (default
   20%, chronological, never touched by passes 1-2) is scored two ways with the
   refined ``theta`` held frozen: the same zone-air rollout (``air_rmse_c``), and
   ``thermal.update(..., learn=False)`` chained through the hold-out records, whose
   returned ``pred_err_c`` is a genuine a-priori equation-error prediction from
   parameters the hold-out data played no part in fitting.

The report also ranks bays by smallest observed margin and fastest fitted dynamics
(``tau_d = C_d / (g0 + k)``) -- plan section 1: "the spare thermistor inputs move
there" -- and states which parameters the data actually pinned down (a relative
standard error under ``mpc.model_converged_rel_se``) versus which are still at
their prior.

Output ``model.json`` carries the thermal memory in exactly the shape
``solver_memory["thermal"]``/``GET /api/model`` use (:mod:`aqua_bridge.control.
thermal` module docstring) under ``"memory"`` -- see :func:`aqua_bridge.control.
thermal.theta_from_memory`.

``--store-out PATH`` additionally writes that fit as a **model store** document
(:func:`aqua_bridge.modelstore.document_from_fit`), which the daemon loads from
``--model-store PATH`` or ``$STATE_DIRECTORY/model.json``: the fitted memory becomes
the store's ``thermal`` section and its ``saved_wall`` is this run's clock, so the
usual ``fresh`` / ``stale`` rule applies to the age of the fit. The daemon also
accepts the *report* itself in the store's place -- it carries ``store_fingerprint``,
the store's own structure fingerprint of this config, so a report meant for another
machine is refused there exactly as a store file would be -- but it replaces that file
with a store document at its first save, so write the store copy separately and keep
the report.
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any

from aqua_bridge import modelstore
from aqua_bridge.config import AppConfig, ConfigError, load_config
from aqua_bridge.control import estimator, thermal
from aqua_bridge.model import MpcConfig
from aqua_bridge.recorder import iter_records, thermal_inputs

__all__ = [
    "build_parser",
    "fit",
    "main",
    "parameters_pinned",
    "rank_bays",
    "refine_air",
]

_LOG = logging.getLogger("aqua_bridge.fit_model")

DEFAULT_HOLDOUT_FRAC = 0.2
DEFAULT_HORIZON_TICKS = 12
DEFAULT_MAX_REFINE_STEP = 0.15
DEFAULT_REFINE_SWEEPS = 3
DEFAULT_MAX_CHUNKS = 150
MIN_RECORDS = 20


def _param_kind(key: str) -> str:
    return key.split(".", 1)[0]


def _mean_trusted(temps: dict[str, float], names: tuple[str, ...]) -> float | None:
    values = [temps[n] for n in names if n in temps]
    return sum(values) / len(values) if values else None


# ---------------------------------------------------------------------------
# per-tick samples for the output-error rollout
# ---------------------------------------------------------------------------


def _zone_samples(
    cfg: MpcConfig, st: thermal.Structure, zone: Any, records: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """One rollout sample per record for ``zone``: exogenous inputs plus whether the
    tick is usable (the zone trusted, every needed mean defined)."""
    samples = []
    for rec in records:
        ti = thermal_inputs(cfg, rec)
        temps = ti["temps"]
        zones_ok = set(ti["zones_ok"])
        params = thermal.model_params(
            cfg, None, st=st, occupancy=ti["occupancy"], maps=ti["maps"], classes=ti["classes"]
        )
        ta = _mean_trusted(temps, zone.air)
        tin = _mean_trusted(temps, zone.inlet)
        nb = {o: _mean_trusted(temps, st.zones[o].air) for o in zone.coupled}
        ok = zone.name in zones_ok and ta is not None and tin is not None
        ok = ok and all(v is not None for v in nb.values())
        s_by_bay: dict[str, float] = {}
        if ok:
            for b in zone.bays:
                if not params.occupied[b]:
                    continue
                sv = _mean_trusted(temps, st.bays[b].sensors)
                if sv is None:
                    ok = False
                    break
                s_by_bay[b] = sv
        samples.append(
            {
                "ts": ti["ts"],
                "u": ti["u"],
                "ta": ta,
                "tin": tin,
                "nb": nb,
                "s_by_bay": s_by_bay,
                "params": params,
                "ok": ok,
            }
        )
    return samples


def _chunks(
    samples: list[dict[str, Any]], horizon_ticks: int, gap_ticks: float, dt: float
) -> list[list[dict[str, Any]]]:
    """Non-overlapping runs of ``horizon_ticks + 1`` consecutive usable samples, the
    gap between consecutive ticks within ``gap_ticks * dt`` (module docstring's
    "seeded from measurement at each horizon's start")."""
    chunks: list[list[dict[str, Any]]] = []
    run: list[dict[str, Any]] = []
    for s in samples:
        if s["ok"] and (not run or 0.0 < s["ts"] - run[-1]["ts"] <= gap_ticks * dt):
            run.append(s)
        else:
            run = [s] if s["ok"] else []
        if len(run) == horizon_ticks + 1:
            chunks.append(list(run))
            run = []
    return chunks


def _zone_flow(
    zone: Any, params: thermal.ThermalParams, theta: dict[str, float], u: dict[str, float]
):
    total = 0.0
    flow = 0.0
    for gr in zone.groups:
        e = theta[gr.key]
        total += e
        if gr.prior <= 0:
            continue
        gphi = sum(w * thermal.phi(u.get(ch, 0.0), *params.fan[ch]) for ch, w in gr.weights.items())
        flow += e * gphi / gr.prior
    qn = flow / total if total > 1e-9 else 0.0
    return flow, qn


def _rollout_error(
    zone: Any, theta: dict[str, float], chunk: list[dict[str, Any]]
) -> tuple[float, int]:
    """Sum of squared ``T_a`` tracking error (degC^2) of one free-running chunk, and
    the number of scored steps."""
    ta = chunk[0]["ta"]
    err2 = 0.0
    n = 0
    for k in range(1, len(chunk)):
        prev_s, cur_s = chunk[k - 1], chunk[k]
        h = cur_s["ts"] - prev_s["ts"]
        flow, qn = _zone_flow(zone, prev_s["params"], theta, prev_s["u"])
        leak = theta[f"leak.{zone.name}"]
        k_total = flow + leak
        m_total = k_total * prev_s["tin"]
        for other in zone.coupled:
            kap = theta[f"kappa.{zone.name}.{other}"]
            k_total += kap
            m_total += kap * prev_s["nb"][other]
        m_total += theta[f"p_air.{zone.name}"]
        for b in zone.bays:
            if b not in prev_s["s_by_bay"]:
                continue
            g = theta[f"g0.{b}"] + theta[f"k.{b}"] * qn
            s_map, b_map = prev_s["params"].s[b], prev_s["params"].b[b]
            k_total += g
            m_total += g * (prev_s["s_by_bay"][b] - b_map) / s_map
        c_air = estimator.C_AIR_J_PER_K
        ta = (ta + h * m_total / c_air) / (1.0 + h * k_total / c_air)
        err2 += (ta - cur_s["ta"]) ** 2
        n += 1
    return err2, n


def _subsample(chunks: list[Any], limit: int) -> list[Any]:
    if len(chunks) <= limit or limit <= 0:
        return chunks
    step = len(chunks) / limit
    return [chunks[int(i * step)] for i in range(limit)]


def refine_air(
    cfg: MpcConfig,
    st: thermal.Structure,
    theta0: dict[str, float],
    records: list[dict[str, Any]],
    *,
    horizon_ticks: int = DEFAULT_HORIZON_TICKS,
    max_frac: float = DEFAULT_MAX_REFINE_STEP,
    sweeps: int = DEFAULT_REFINE_SWEEPS,
    max_chunks: int = DEFAULT_MAX_CHUNKS,
) -> tuple[dict[str, float], dict[str, Any]]:
    """Output-error refinement of the zone-air block (module docstring, pass 2). Pure
    given ``records``; the per-bay ``g0``/``k``/``q_s`` keys of ``theta0`` pass
    through unchanged."""
    theta = dict(theta0)
    report: dict[str, Any] = {}
    fracs = tuple(f for f in (-max_frac, -max_frac / 2, max_frac / 2, max_frac) if f != 0.0)
    for z, zone in st.zones.items():
        samples = _zone_samples(cfg, st, zone, records)
        chunks = _subsample(_chunks(samples, horizon_ticks, thermal.GAP_TICKS, cfg.dt), max_chunks)
        if not chunks:
            report[z] = {"status": "skipped", "reason": "insufficient contiguous trusted data"}
            continue

        def total_error(
            th: dict[str, float], chunks: list[Any] = chunks, zone: Any = zone
        ) -> float:
            return sum(_rollout_error(zone, th, c)[0] for c in chunks)

        keys = list(zone.air_keys)
        error_before = total_error(theta)
        for _ in range(sweeps):
            improved = False
            for key in keys:
                base = theta[key]
                spec = thermal.PARAMETERS[_param_kind(key)]
                best_val, best_err = base, total_error(theta)
                for frac in fracs:
                    delta = base * frac if abs(base) > 1e-9 else frac * spec.scale
                    cand = min(spec.hi, max(spec.lo, base + delta))
                    trial = dict(theta)
                    trial[key] = cand
                    err = total_error(trial)
                    if err < best_err:
                        best_err, best_val = err, cand
                if best_val != base:
                    theta[key] = best_val
                    improved = True
            if not improved:
                break
        report[z] = {
            "status": "ok",
            "chunks": len(chunks),
            "horizon_ticks": horizon_ticks,
            "keys_refined": keys,
            "rmse_c_before": math.sqrt(error_before / max(1, len(chunks) * horizon_ticks)),
            "rmse_c_after": math.sqrt(total_error(theta) / max(1, len(chunks) * horizon_ticks)),
        }
    return theta, report


def _air_holdout_rmse(
    cfg: MpcConfig,
    st: thermal.Structure,
    theta: dict[str, float],
    records: list[dict[str, Any]],
    horizon_ticks: int,
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for z, zone in st.zones.items():
        samples = _zone_samples(cfg, st, zone, records)
        chunks = _chunks(samples, horizon_ticks, thermal.GAP_TICKS, cfg.dt)
        if not chunks:
            out[z] = {
                "status": "skipped",
                "reason": "insufficient contiguous trusted hold-out data",
            }
            continue
        totals = [_rollout_error(zone, theta, c) for c in chunks]
        err2 = sum(e for e, _ in totals)
        n = sum(k for _, k in totals)
        out[z] = {
            "status": "ok",
            "chunks": len(chunks),
            "rmse_c": math.sqrt(err2 / n) if n else None,
        }
    return out


def _write_theta_into_memory(
    memory: dict[str, Any], st: thermal.Structure, theta: dict[str, float]
) -> dict[str, Any]:
    out = copy.deepcopy(memory)
    for z, zone in st.zones.items():
        out["zones"][z]["air"]["theta"] = [theta[k] for k in zone.air_keys]
    for b, bay in st.bays.items():
        out["bays"][b]["theta"] = [theta[k] for k in bay.keys]
    return out


def parameters_pinned(cfg: MpcConfig, summary: dict[str, Any]) -> dict[str, Any]:
    """Every identified key: its value, relative standard error and whether the data
    *pinned* it (``rel_se < mpc.model_converged_rel_se``); ``None`` rel_se means the
    key never saw an excited window and is still effectively at its prior."""
    threshold = cfg.model_converged_rel_se
    out: dict[str, Any] = {}
    for block in (*summary["zones"].values(), *summary["bays"].values()):
        for key, value in block["theta"].items():
            rel = block["rel_se"].get(key)
            out[key] = {
                "value": value,
                "rel_se": rel,
                "pinned": rel is not None and rel < threshold,
                "windows": block["windows"],
                "excited_windows": block["excited_windows"],
            }
    return out


def rank_bays(
    cfg: MpcConfig, st: thermal.Structure, theta: dict[str, float], records: list[dict[str, Any]]
) -> list[dict[str, Any]]:
    """Bays ranked by smallest observed margin and fastest fitted dynamics (module
    docstring; plan section 1: spare thermistor inputs go where this points)."""
    rows: list[dict[str, Any]] = []
    for b, bay in st.bays.items():
        cls = cfg.bay_class(b)
        limit = cfg.bay_limit(b)
        c_drive = estimator.drive_capacity(cfg, cls)
        g0, k = theta[f"g0.{b}"], theta[f"k.{b}"]
        tau_d_s = c_drive / max(g0 + k, 1e-9)
        observed_max_c = None
        for rec in records:
            temps = rec.get("trusted_temps") or {}
            for name in bay.sensors:
                v = temps.get(name)
                if isinstance(v, int | float) and (observed_max_c is None or v > observed_max_c):
                    observed_max_c = float(v)
        margin_c = None if observed_max_c is None else limit - observed_max_c
        rows.append(
            {
                "bay": b,
                "zone": bay.zone,
                "class": cls,
                "limit_c": limit,
                "observed_max_c": observed_max_c,
                "margin_c": margin_c,
                "tau_d_s": tau_d_s,
                "g0_w_per_k": g0,
                "k_w_per_k": k,
            }
        )
    with_margin = sorted(
        (r for r in rows if r["margin_c"] is not None), key=lambda r: r["margin_c"]
    )
    for rank, r in enumerate(with_margin):
        r["rank_margin"] = rank
    for r in rows:
        r.setdefault("rank_margin", len(with_margin))  # unobserved: least urgent, not "safe"
    for rank, r in enumerate(sorted(rows, key=lambda r: r["tau_d_s"])):
        r["rank_dynamics"] = rank
    for r in rows:
        r["rank_combined"] = r["rank_margin"] + r["rank_dynamics"]
    rows.sort(key=lambda r: r["rank_combined"])
    return rows


# ---------------------------------------------------------------------------
# the fit itself
# ---------------------------------------------------------------------------


def fit(
    cfg: MpcConfig,
    records: list[dict[str, Any]],
    *,
    holdout_frac: float = DEFAULT_HOLDOUT_FRAC,
    horizon_ticks: int = DEFAULT_HORIZON_TICKS,
    max_refine_step: float = DEFAULT_MAX_REFINE_STEP,
    refine_sweeps: int = DEFAULT_REFINE_SWEEPS,
    max_chunks: int = DEFAULT_MAX_CHUNKS,
) -> dict[str, Any]:
    """The three passes (module docstring) over ``records`` (chronological, DAS
    records only). Raises ``ValueError`` if ``cfg`` is not zoned or there are too
    few records; every other failure mode of the passes below is a per-block
    ``"status": "skipped"`` entry in the report, never an exception."""
    if not cfg.is_das:
        raise ValueError("fit_model needs a zoned (DAS) config (mpc.topology)")
    if len(records) < MIN_RECORDS:
        raise ValueError(
            f"not enough DAS records to fit (need >= {MIN_RECORDS}, got {len(records)})"
        )

    st = thermal.structure(cfg)
    if holdout_frac <= 0.0:
        n_train = len(records)  # holdout_frac 0 means exactly that: no hold-out split
    else:
        n_train = max(1, round(len(records) * (1.0 - holdout_frac)))
        # a small fraction on a small recording must not round away to 0 hold-out
        n_train = min(n_train, len(records) - 1) if len(records) > 1 else len(records)
    train, holdout = records[:n_train], records[n_train:]

    # -- pass 1: windowed least squares per zone -----------------------------------
    mem: Any = None
    result = None
    for rec in train:
        result = thermal.update(mem, cfg, learn=True, **thermal_inputs(cfg, rec))
        mem = result.memory
    if mem is None:
        mem = thermal.fresh_memory(cfg)
    theta_wls = thermal.theta_from_memory(cfg, mem, st=st)

    # -- pass 2: output-error refinement (zone-air only) ----------------------------
    theta_refined, refine_report = refine_air(
        cfg,
        st,
        theta_wls,
        train,
        horizon_ticks=horizon_ticks,
        max_frac=max_refine_step,
        sweeps=refine_sweeps,
        max_chunks=max_chunks,
    )
    mem_refined = _write_theta_into_memory(mem, st, theta_refined)
    final_summary = thermal.summary(mem_refined, cfg, st=st)

    # -- pass 3: hold-out validation --------------------------------------------------
    holdout_report: dict[str, Any] = {"n_records": len(holdout)}
    if holdout:
        holdout_report["air_rmse_c"] = _air_holdout_rmse(
            cfg, st, theta_refined, holdout, horizon_ticks
        )
        holdout_mem = copy.deepcopy(mem_refined)
        eq_result = None
        for rec in holdout:
            eq_result = thermal.update(holdout_mem, cfg, learn=False, **thermal_inputs(cfg, rec))
            holdout_mem = eq_result.memory
        holdout_report["equation_error"] = None if eq_result is None else eq_result.summary
    else:
        holdout_report["air_rmse_c"] = {}
        holdout_report["equation_error"] = None

    return {
        "v": 1,
        "kind": "aqua_bridge.thermal_model",
        "fingerprint": st.fingerprint,
        # the model store's own structure fingerprint, so a report copied into
        # $STATE_DIRECTORY is refused for a config it was not fitted against
        "store_fingerprint": modelstore.fingerprint(cfg),
        "n_records": len(records),
        "n_train": len(train),
        "n_holdout": len(holdout),
        "memory": mem_refined,
        "summary": final_summary,
        "refinement": refine_report,
        "holdout": holdout_report,
        "parameters_pinned": parameters_pinned(cfg, final_summary),
        "bay_ranking": rank_bays(cfg, st, theta_refined, records),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("recordings", nargs="+", metavar="RECORDING.jsonl", help="oldest first")
    p.add_argument("--config", required=True)
    p.add_argument(
        "--topology",
        action="store_true",
        help="fit the zoned thermal model (required: the only mode this tool supports)",
    )
    p.add_argument("--out", default="model.json", metavar="PATH")
    p.add_argument(
        "--store-out",
        metavar="PATH",
        help="also write the fit as a model store document the daemon can load "
        "(--model-store PATH or $STATE_DIRECTORY/model.json); keep it separate from --out, "
        "the daemon overwrites the store file",
    )
    p.add_argument("--holdout-frac", type=float, default=DEFAULT_HOLDOUT_FRAC)
    p.add_argument("--horizon-ticks", type=int, default=DEFAULT_HORIZON_TICKS)
    p.add_argument("--max-refine-step", type=float, default=DEFAULT_MAX_REFINE_STEP)
    p.add_argument("--refine-sweeps", type=int, default=DEFAULT_REFINE_SWEEPS)
    p.add_argument("--max-chunks", type=int, default=DEFAULT_MAX_CHUNKS)
    p.add_argument("--log-level", default="INFO")
    return p


def _print_report(cfg: MpcConfig, payload: dict[str, Any]) -> None:
    n_train, n_holdout = payload["n_train"], payload["n_holdout"]
    print(f"records: {payload['n_records']} (train {n_train}, holdout {n_holdout})")
    summary = payload["summary"]
    print(f"status: {summary['status']}  pred_err_c: {summary['pred_err_c']}")
    for z, info in summary["zones"].items():
        w, ew = info["windows"], info["excited_windows"]
        print(f"  zone {z}: {info['status']}  windows={w} excited={ew}")
    pinned = [k for k, v in payload["parameters_pinned"].items() if v["pinned"]]
    print(f"parameters pinned by the data ({len(pinned)}): {', '.join(sorted(pinned)) or '(none)'}")
    print("bay ranking (smallest margin, fastest dynamics first):")
    for row in payload["bay_ranking"]:
        margin = "?" if row["margin_c"] is None else f"{row['margin_c']:.1f}"
        tau = row["tau_d_s"]
        print(f"  {row['bay']:>8}  zone={row['zone']:<6} margin_c={margin:>6} tau_d_s={tau:.0f}")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))
    if not args.topology:
        parser.error("--topology is required (the only fit mode this tool supports)")

    try:
        app: AppConfig = load_config(args.config)
    except ConfigError as exc:
        print(f"config: {exc}", file=sys.stderr)
        return 2
    cfg = app.mpc

    records: list[dict[str, Any]] = []
    for path in args.recordings:
        records.extend(iter_records(path))
    records = [r for r in records if r.get("das")]
    records.sort(key=lambda r: r.get("ts", 0.0))

    try:
        payload = fit(
            cfg,
            records,
            holdout_frac=args.holdout_frac,
            horizon_ticks=args.horizon_ticks,
            max_refine_step=args.max_refine_step,
            refine_sweeps=args.refine_sweeps,
            max_chunks=args.max_chunks,
        )
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    payload["generated_at"] = time.time()
    payload["config"] = str(args.config)
    payload["recordings"] = list(args.recordings)

    out_path = Path(args.out)
    if out_path.parent != Path(""):
        out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8"
    )
    _print_report(cfg, payload)
    print(f"wrote {out_path}")

    if args.store_out:
        store_out = Path(args.store_out)
        if store_out.resolve() == out_path.resolve():
            print("--store-out must differ from --out", file=sys.stderr)
            return 2
        if store_out.parent != Path(""):
            store_out.parent.mkdir(parents=True, exist_ok=True)
        try:
            document = modelstore.document_from_fit(cfg, payload)
        except ValueError as exc:  # unreachable for a payload this run built; never silent
            print(f"model store: {exc}", file=sys.stderr)
            return 2
        modelstore.write_atomic(
            store_out, json.dumps(document, allow_nan=False, separators=(",", ":")).encode()
        )
        print(f"wrote {store_out} (model store, {modelstore.SCHEMA} v{modelstore.SCHEMA_VERSION})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
