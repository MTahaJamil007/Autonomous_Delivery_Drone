"""UDP sensor intake: the writer for the timestamps the fail-closed checks read.

WHY THIS MODULE EXISTS (the P2.3 defect)
----------------------------------------
Three places already read sensor freshness:

    navigation.py     -> lidar_data["last_lidar_ts"]
    safety_supervisor -> lidar_data["last_lidar_ts"]
    safety_supervisor -> drone_state["last_telemetry_ts"]

NOTHING ANYWHERE WROTE EITHER KEY. The consequences were both wrong and
opposite:

* Navigation read `time.time() - 0`, an enormous number, concluded the LiDAR
  feed was stale on the very first tick, and held position forever. The drone
  took off and hovered. The fail-closed guard was permanently closed.
* The supervisor's heartbeat check computed the same difference and fired an
  emergency RTL at t=0, before any telemetry could possibly have arrived.

One reader fixes both, because both defects were the same missing writer.

DESIGN NOTES
------------
* DRAIN TO THE NEWEST DATAGRAM. Reading one datagram per tick would build a
  backlog under load and act on stale avoidance decisions -- dodging left
  because of a wall the drone has already passed. The loop reads until the
  socket is empty and keeps only the last message, which for a state feed
  (rather than an event stream) is always the right one.
* MONOTONIC TIMESTAMPS. Every consumer computes an elapsed time from these.
  Wall clock is subject to NTP steps; a backwards step in flight makes a
  timeout fire instantly or never.
* SEQUENCE-NUMBER GAP DETECTION distinguishes "the producer is slow" from "the
  network is dropping datagrams", which are different problems with different
  fixes.
* Malformed payloads are counted and logged, never fatal. A garbled datagram
  must not end a flight; the staleness check already handles a feed that has
  genuinely stopped saying anything useful.
"""

from __future__ import annotations

import asyncio
import json
import logging
import socket
import time
from typing import Any

import config

logger = logging.getLogger(__name__)

RECV_BUFFER_BYTES = 65535
POLL_INTERVAL_S = 0.02          # 50 Hz: comfortably faster than either 10 Hz feed


class UdpJsonReceiver:
    """Reads newline-free JSON datagrams into a shared state dict.

    The dict is the same object the navigation and landing loops read, so a
    fresh datagram is visible to them without any handoff. Single-threaded
    asyncio means no lock is needed: only this task writes.
    """

    def __init__(
        self,
        name: str,
        port: int,
        state: dict[str, Any],
        timestamp_key: str,
        host: str = "127.0.0.1",
    ):
        """
        Args:
            name: Label for logs, e.g. 'lidar' or 'vision'.
            port: UDP port to bind.
            state: Shared dict to update in place.
            timestamp_key: Key to stamp with time.monotonic() on every valid
                datagram. This is the key the fail-closed checks read; getting
                it wrong reintroduces the exact bug this class fixes.
            host: Bind address.
        """
        self._name = name
        self._port = port
        self._host = host
        self._state = state
        self._timestamp_key = timestamp_key

        self._sock: socket.socket | None = None
        self._last_seq: int | None = None

        self.messages = 0
        self.malformed = 0
        self.dropped = 0

    def bind(self) -> None:
        """Bind the socket. Separate from the loop so a port clash fails early.

        SO_REUSEADDR is set so a restart after an unclean exit does not hit
        "Address already in use" while the old socket lingers in TIME_WAIT --
        which would otherwise mean the fastest way to recover a crashed bridge
        is to wait a minute.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setblocking(False)
        try:
            sock.bind((self._host, self._port))
        except OSError as exc:
            sock.close()
            raise OSError(
                f"{self._name} receiver cannot bind {self._host}:{self._port}: {exc}. "
                f"Another instance is probably already running."
            ) from exc
        self._sock = sock
        logger.info("%s receiver listening on %s:%d", self._name, self._host, self._port)

    async def run(self) -> None:
        """Drain-and-stamp loop. Run as a task for the mission's lifetime."""
        if self._sock is None:
            self.bind()
        assert self._sock is not None

        loop = asyncio.get_running_loop()
        try:
            while True:
                newest: bytes | None = None

                # Drain: keep only the most recent datagram. Processing a
                # backlog would mean acting on decisions about obstacles the
                # drone has already flown past.
                while True:
                    try:
                        data, _addr = self._sock.recvfrom(RECV_BUFFER_BYTES)
                    except BlockingIOError:
                        break
                    except OSError as exc:
                        logger.warning("%s receiver socket error: %s", self._name, exc)
                        break
                    newest = data

                if newest is not None:
                    self._ingest(newest)

                await asyncio.sleep(POLL_INTERVAL_S)
        except asyncio.CancelledError:
            raise
        finally:
            self.close()
            # Deliberately does NOT clear the timestamp. On shutdown the reader
            # stops and the timestamp goes stale on its own, which is exactly
            # what the consumers should observe.
            loop_is_closing = loop.is_closed()
            logger.info(
                "%s receiver stopped: %d messages, %d malformed, %d dropped%s",
                self._name, self.messages, self.malformed, self.dropped,
                "" if not loop_is_closing else " (loop closing)",
            )

    def _ingest(self, data: bytes) -> None:
        """Parse one datagram and merge it into the shared state."""
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.malformed += 1
            if self.malformed <= 5 or self.malformed % 100 == 0:
                logger.warning(
                    "%s receiver: malformed datagram #%d (%s): %r",
                    self._name, self.malformed, exc, data[:80],
                )
            return

        if not isinstance(payload, dict):
            self.malformed += 1
            return

        seq = payload.get("seq")
        if isinstance(seq, int):
            if self._last_seq is not None and seq > self._last_seq + 1:
                gap = seq - self._last_seq - 1
                self.dropped += gap
                logger.debug("%s receiver: %d datagram(s) dropped", self._name, gap)
            self._last_seq = seq

        self._state.update(payload)

        # THE WRITE THIS MODULE EXISTS FOR. Stamped from our own monotonic clock
        # rather than trusting the producer's `t`: the producer may be a separate
        # process whose monotonic epoch differs, and what the consumer needs to
        # know is "how long since I last heard anything", which only the
        # receiver can answer.
        self._state[self._timestamp_key] = time.monotonic()
        self.messages += 1

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None


def is_stale(
    state: dict[str, Any],
    timestamp_key: str,
    timeout_s: float = config.STALE_SENSOR_TIMEOUT_S,
    now: float | None = None,
) -> bool:
    """True if the feed has not produced a valid datagram recently.

    A feed that has NEVER produced one (the key is absent, or zero) counts as
    stale. That is the correct answer for navigation -- never assume a path is
    clear on the strength of no information -- but it is the wrong answer for
    the supervisor's heartbeat check, which must not trigger an emergency RTL
    before the first telemetry sample has had a chance to arrive. Callers that
    need to distinguish "not yet started" from "stopped" use `has_ever_arrived`.
    """
    last = state.get(timestamp_key, 0.0)
    if not last:
        return True
    return (time.monotonic() if now is None else now) - last > timeout_s


def has_ever_arrived(state: dict[str, Any], timestamp_key: str) -> bool:
    """True once at least one valid datagram has been received."""
    return bool(state.get(timestamp_key, 0.0))


def age_s(state: dict[str, Any], timestamp_key: str, now: float | None = None) -> float:
    """Seconds since the last valid datagram, or inf if there has never been one."""
    last = state.get(timestamp_key, 0.0)
    if not last:
        return float("inf")
    return (time.monotonic() if now is None else now) - last


# Keys used across the flight stack. Named constants so a typo is an
# AttributeError here rather than a silently-never-written dict key there --
# which is the shape of the bug this module fixes.
LIDAR_TS_KEY = "last_lidar_ts"
VISION_TS_KEY = "last_vision_ts"
TELEMETRY_TS_KEY = "last_telemetry_ts"


def make_lidar_receiver(drone_id: str, lidar_data: dict[str, Any]) -> UdpJsonReceiver:
    """Receiver for one drone's avoidance actions."""
    return UdpJsonReceiver(
        name=f"lidar[{drone_id}]",
        port=config.lidar_port(drone_id),
        state=lidar_data,
        timestamp_key=LIDAR_TS_KEY,
    )


def make_vision_receiver(drone_id: str, vision_data: dict[str, Any]) -> UdpJsonReceiver:
    """Receiver for one drone's marker detections."""
    return UdpJsonReceiver(
        name=f"vision[{drone_id}]",
        port=config.vision_port(drone_id),
        state=vision_data,
        timestamp_key=VISION_TS_KEY,
    )
