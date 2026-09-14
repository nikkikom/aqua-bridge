"""Static check of ``publishers/static/index.html``'s JavaScript against the real
JSON views (PROJECT.md section 6, section 8 items 22, 24 and 25).

The page is a thin, view-only client: every field its JavaScript reads must
exist in the JSON the server actually sends. This test never runs the page in
a browser; it extracts every ``.fieldName`` token the ``<script>`` block reads
(a fixed set of JS/DOM builtins aside, listed in ``_JS_BUILTINS`` below) and
checks each one names a real key *somewhere* in the union of every JSON
payload the page fetches (``/api/state``, ``/api/health``, ``/api/estimate``,
``/api/bays``, ``/api/zones``, ``/api/model``), collected from one live tick
of a real DAS config with a zone fault and ``model_shadow`` on, and one tick
of a legacy config, so both modes' fields are present. This is deliberately
approximate (it does not track which local variable holds which payload) --
it exists to catch a stale or misspelled field name, not to replace a
browser test."""

from __future__ import annotations

import asyncio
import dataclasses
import re
from pathlib import Path
from typing import Any

from aqua_bridge.config import load_config
from aqua_bridge.control.mpc import step
from aqua_bridge.control.supervisor import Supervisor
from aqua_bridge.model import MpcConfig, MpcState
from conftest import EXAMPLE_CONFIG, EXAMPLE_DAS_CONFIG
from das_fixtures import das_obs
from test_das_intents import _post_all, needs_socket

_INDEX_HTML = (
    Path(__file__).resolve().parent.parent
    / "src"
    / "aqua_bridge"
    / "publishers"
    / "static"
    / "index.html"
)

# Every non-JSON ".identifier" the page's <script> reads: DOM/JS builtins and
# array/promise/string methods, not a field of any server payload. Kept as an
# explicit list (not a guess) -- see the module docstring's counted dump.
_JS_BUILTINS = {
    "all",
    "className",
    "getElementById",
    "hidden",
    "innerHTML",
    "join",
    "json",
    "keys",
    "length",
    "map",
    "ok",
    "push",
    "querySelector",
    "sort",
    "stringify",
    "textContent",
    "toFixed",
}


def _script_field_tokens() -> set[str]:
    text = _INDEX_HTML.read_text(encoding="utf-8")
    script = text.split("<script>", 1)[1].split("</script>", 1)[0]
    tokens = set(re.findall(r"\.([A-Za-z_][A-Za-z0-9_]*)", script))
    return tokens - _JS_BUILTINS


def _collect_keys(value: Any, out: set[str]) -> None:
    """Every dict key anywhere in ``value``, however deeply nested."""
    if isinstance(value, dict):
        for k, v in value.items():
            out.add(k)
            _collect_keys(v, out)
    elif isinstance(value, list):
        for item in value:
            _collect_keys(item, out)


def _das_payload_keys() -> set[str]:
    cfg = dataclasses.replace(load_config(EXAMPLE_DAS_CONFIG).mpc, model_shadow=True)
    sup = Supervisor(cfg)
    state = MpcState.cold()
    obs = das_obs(cfg, 0.0, pwm=0.5)
    cmd, state = step(obs, sup.effective_config(), state)
    sup.record_tick(obs=obs, mpc_cmd=cmd, cmd=cmd, state=state, applied=True, usb_present=True)
    # a zone fault, so "fault"/"in_closure"/"channels_under_fallback" are populated
    obs2 = das_obs(cfg, cfg.dt, pwm=cmd.pwm, drop=("air_z3",))
    cmd2, state2 = step(obs2, sup.effective_config(), state)
    sup.record_tick(obs=obs2, mpc_cmd=cmd2, cmd=cmd2, state=state2, applied=True, usb_present=True)

    results = asyncio.run(
        _post_all(
            sup,
            [
                ("GET /api/state", None),
                ("GET /api/health", None),
                ("GET /api/estimate", None),
                ("GET /api/bays", None),
                ("GET /api/zones", None),
                ("GET /api/model", None),
            ],
        )
    )
    keys: set[str] = set()
    for status, body in results:
        assert status == 200
        _collect_keys(body, keys)
    return keys


def _legacy_payload_keys() -> set[str]:
    cfg: MpcConfig = load_config(EXAMPLE_CONFIG).mpc
    sup = Supervisor(cfg)
    results = asyncio.run(_post_all(sup, [("GET /api/state", None), ("GET /api/health", None)]))
    keys: set[str] = set()
    for status, body in results:
        assert status == 200
        _collect_keys(body, keys)
    return keys


@needs_socket
def test_index_html_field_tokens_are_real_json_keys() -> None:
    used = _script_field_tokens()
    real = _das_payload_keys() | _legacy_payload_keys()
    missing = sorted(used - real)
    assert not missing, (
        f"index.html reads {missing} but no /api/... payload in this test has that key "
        "-- stale field name, or the test's fixture needs to grow to cover it"
    )


def test_js_builtins_list_has_no_unused_entries() -> None:
    """Keeps ``_JS_BUILTINS`` honest: every excluded name must actually occur in the
    page, so a rename there is caught here instead of silently widening the exclusion."""
    text = _INDEX_HTML.read_text(encoding="utf-8")
    script = text.split("<script>", 1)[1].split("</script>", 1)[0]
    all_tokens = set(re.findall(r"\.([A-Za-z_][A-Za-z0-9_]*)", script))
    unused = _JS_BUILTINS - all_tokens
    assert not unused, f"_JS_BUILTINS lists names index.html no longer uses: {sorted(unused)}"
