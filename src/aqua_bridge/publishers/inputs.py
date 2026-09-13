"""SMART temperatures inbox (the DAS plan, section 1 "SMART path").

The PC-side agent (``tools/smart_agent.py``) publishes one retained MQTT
message per drive to ``{node_id}/in/smart/<serial>`` (payload
``{serial, model, temp_c, ts_wall}``); a non-MQTT setup can ``POST
/api/in/smart`` with the same JSON body instead (the "twin" the plan calls
for). :class:`SmartInbox` is the single store both transports feed: it
keeps the latest sample per serial, stamped with this process's own
monotonic receipt time (never the agent's ``ts_wall``, which is wall-clock
and can be skewed or wrong -- staleness is judged on receipt time only).
:meth:`snapshot` is what :class:`aqua_bridge.hw.sources.CompositeSource`
reads every tick into ``PlantObservation.inputs["smart"]``.

Deliberately *not* here (later milestones per the plan):

* serial -> bay association (``control/estimator.py``'s job, plan section 1
  point 4: correlation against ``T_s - T_a`` on the DAS truth/estimate) --
  this module only ever indexes by serial, never a bay;
* calibration, staleness-driven ``sigma_cal`` expiry -- the estimator's
  ``smart_max_age_s``/``calibration_max_age_days`` do not exist as config
  yet (the ``estimator`` section is a later milestone); :data:`DEFAULT_MAX_AGE_S`
  is this module's own constant (300 s, the plan's proposed default),
  overridable per instance, not read from ``MpcConfig``.

SMART is explicitly never a gate input (plan section 1, "Trust"): a
malformed or implausible payload only ever increments :attr:`SmartInbox.rejected`
and is dropped -- it can never raise into the MQTT network thread or fail
an HTTP request with a 5xx, and it can never fault a zone.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

__all__ = ["DEFAULT_MAX_AGE_S", "SmartInbox", "smart_topic_filter"]

_LOG = logging.getLogger("aqua_bridge.publishers.inputs")

#: The plan's proposed ``estimator.smart_max_age_s`` default (section 7),
#: kept here as a plain constant until the ``estimator`` config section
#: exists (a later milestone) to read it from.
DEFAULT_MAX_AGE_S = 300.0


def smart_topic_filter(node_id: str) -> str:
    """The MQTT topic filter every SMART reading is published under."""
    return f"{node_id}/in/smart/+"


@dataclass
class _Entry:
    temp_c: float
    model: str | None
    ts_wall: float | None
    received_at: float


def _validate_payload(payload: object) -> tuple[str, float, str | None, float | None] | None:
    """``(serial, temp_c, model, ts_wall)`` or ``None`` if ``payload`` is not
    a well-formed SMART sample (plan section 1: ``{serial, model, temp_c,
    ts_wall}``). ``model``/``ts_wall`` are optional; everything else about
    the shape is checked strictly so a garbage message never gets stored
    half-parsed.
    """
    if not isinstance(payload, Mapping):
        return None
    serial = payload.get("serial")
    if not isinstance(serial, str) or not serial:
        return None
    temp_c = payload.get("temp_c")
    if isinstance(temp_c, bool) or not isinstance(temp_c, int | float):
        return None
    temp_c = float(temp_c)
    if not math.isfinite(temp_c):
        return None
    model = payload.get("model")
    if model is not None and not isinstance(model, str):
        return None
    ts_wall = payload.get("ts_wall")
    if ts_wall is not None:
        if isinstance(ts_wall, bool) or not isinstance(ts_wall, int | float):
            return None
        ts_wall = float(ts_wall)
        if not math.isfinite(ts_wall):
            return None
    return serial, temp_c, model, ts_wall


class SmartInbox:
    """Latest SMART temperature per drive serial, fed by MQTT and/or HTTP.

    Parameters
    ----------
    max_age_s:
        :meth:`snapshot` omits a serial whose last accepted sample is older
        than this (receipt time, not ``ts_wall``).
    clock:
        Zero-argument monotonic seconds callable, injected so tests control
        time without sleeping (matches :class:`~aqua_bridge.hw.onewire.W1Source`).
    """

    def __init__(
        self, *, max_age_s: float = DEFAULT_MAX_AGE_S, clock: Callable[[], float] = time.monotonic
    ) -> None:
        if not max_age_s > 0:
            raise ValueError(f"max_age_s must be > 0, got {max_age_s}")
        self._max_age_s = float(max_age_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._entries: dict[str, _Entry] = {}
        self.rejected = 0
        self.accepted = 0

    # -- ingest (MQTT thread or the HTTP handler; never raises) -------------

    def record(self, payload: object) -> bool:
        """Stores one decoded SMART sample. ``True`` if it was well-formed
        and stored, ``False`` (and :attr:`rejected` incremented) otherwise.
        Never raises -- an unexpected payload shape (wrong types, extra or
        missing keys, NaN/inf) is just rejected, per the module docstring's
        "SMART can only add information" rule.
        """
        try:
            parsed = _validate_payload(payload)
        except Exception:  # defensive: a payload with a hostile __getattr__ etc.
            parsed = None
        if parsed is None:
            self.rejected += 1
            return False
        serial, temp_c, model, ts_wall = parsed
        with self._lock:
            self._entries[serial] = _Entry(
                temp_c=temp_c, model=model, ts_wall=ts_wall, received_at=self._clock()
            )
        self.accepted += 1
        return True

    def on_message(self, topic: str, payload: bytes | str) -> None:
        """MQTT ``on_message`` shape (``str, bytes -> None``): decodes JSON
        and calls :meth:`record`. Never raises -- see the module docstring;
        a garbage message on ``{node_id}/in/smart/+`` must not touch the
        paho network thread (the same contract
        :func:`aqua_bridge.publishers.mqtt_ha.MqttClient._on_message` gives
        the command topics).
        """
        try:
            text = (
                payload.decode("utf-8") if isinstance(payload, bytes | bytearray) else str(payload)
            )
            data = json.loads(text)
        except Exception:
            self.rejected += 1
            _LOG.info("smart: undecodable payload on %s", topic)
            return
        if not self.record(data):
            _LOG.info("smart: rejected payload on %s", topic)

    # -- read (loop thread, every tick) --------------------------------------

    def snapshot(self) -> dict[str, dict[str, Any]]:
        """``{serial: {temp_c, age_s, model}}`` for every serial whose last
        sample is within ``max_age_s`` (receipt time) -- what
        :class:`~aqua_bridge.hw.sources.CompositeSource` puts into
        ``PlantObservation.inputs["smart"]`` every tick. A serial that has
        gone stale simply stops appearing; it is not deleted (a late-but-
        still-recent sample keeps updating it).
        """
        now = self._clock()
        out: dict[str, dict[str, Any]] = {}
        with self._lock:
            for serial, entry in self._entries.items():
                age_s = now - entry.received_at
                if age_s <= self._max_age_s:
                    out[serial] = {"temp_c": entry.temp_c, "age_s": age_s, "model": entry.model}
        return out
