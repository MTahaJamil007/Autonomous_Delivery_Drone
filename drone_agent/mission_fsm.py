"""Mission finite state machine.

The transition table below was already correct and unit-tested before this
remediation; what was missing was any caller. The mission orchestrator now
drives it, which is what turns the FSM from a described feature into an
observable one.

WHAT CHANGED
------------
* Added transitions for the paths a real mission needs and the table lacked:
  a pre-leg battery refusal (which happens in NEXT_LEG or IDLE, not ENROUTE),
  a mission-wide `abort` the safety supervisor can force from any state, and
  `leg_failed` so a failed landing routes somewhere explicit instead of
  falling through. Every transition the existing tests exercise is unchanged.

* on_enter callbacks that return coroutines now have their tasks referenced.
  `asyncio.create_task(result)` with the result discarded can be
  garbage-collected before it runs, and its exception vanishes with it.

* `history` records the transitions taken, so a mission result can say how far
  it got rather than only that it failed.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable
from enum import Enum

logger = logging.getLogger(__name__)


class MissionState(Enum):
    """States in a delivery mission's lifecycle."""

    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    ENROUTE = "ENROUTE"
    SEARCHING = "SEARCHING"
    APPROACH = "APPROACH"
    DESCEND = "DESCEND"
    PAYLOAD_OP = "PAYLOAD_OP"
    NEXT_LEG = "NEXT_LEG"
    HOVER_AND_ALERT = "HOVER_AND_ALERT"
    ABORT = "ABORT"
    DONE = "DONE"


TRANSITIONS: dict[MissionState, dict[str, MissionState]] = {
    MissionState.IDLE: {
        "start": MissionState.TAKEOFF,
        # Pre-leg battery gate for the FIRST leg. The gate runs before takeoff,
        # so it cannot use ENROUTE's battery_low.
        "battery_low": MissionState.ABORT,
        "abort": MissionState.ABORT,
    },
    MissionState.TAKEOFF: {
        "altitude_reached": MissionState.ENROUTE,
        "fail_x2": MissionState.ABORT,
        "abort": MissionState.ABORT,
    },
    MissionState.ENROUTE: {
        "arrived": MissionState.SEARCHING,
        "battery_low": MissionState.ABORT,
        # Reactive avoidance and detour planning both failed: hold for a human
        # rather than keep generating longer paths through clutter.
        "blocked": MissionState.HOVER_AND_ALERT,
        "abort": MissionState.ABORT,
    },
    MissionState.SEARCHING: {
        "marker_locked": MissionState.APPROACH,
        "search_exhausted": MissionState.HOVER_AND_ALERT,
        "abort": MissionState.ABORT,
    },
    MissionState.APPROACH: {
        "centered_stable": MissionState.DESCEND,
        "lock_lost": MissionState.SEARCHING,
        "touchdown": MissionState.PAYLOAD_OP,
        "search_exhausted": MissionState.HOVER_AND_ALERT,
        "abort": MissionState.ABORT,
    },
    MissionState.DESCEND: {
        "touchdown": MissionState.PAYLOAD_OP,
        "marker_drift": MissionState.APPROACH,
        "lock_lost": MissionState.SEARCHING,
        "search_exhausted": MissionState.HOVER_AND_ALERT,
        "abort": MissionState.ABORT,
    },
    MissionState.PAYLOAD_OP: {
        "op_confirmed": MissionState.NEXT_LEG,
        "op_failed_x2": MissionState.HOVER_AND_ALERT,
        "abort": MissionState.ABORT,
    },
    MissionState.NEXT_LEG: {
        "more_legs": MissionState.TAKEOFF,
        "mission_complete": MissionState.DONE,
        # Pre-leg battery gate for legs 2 and 3.
        "battery_low": MissionState.ABORT,
        "abort": MissionState.ABORT,
    },
    MissionState.HOVER_AND_ALERT: {
        "operator_override": MissionState.SEARCHING,
        "geofence_timeout": MissionState.ABORT,
        "abort": MissionState.ABORT,
    },
    # Terminal states accept nothing. Firing at them logs and returns False,
    # which is the correct response to a late event from a cancelled task.
    MissionState.ABORT: {},
    MissionState.DONE: {},
}


class MissionFSM:
    """Mission state with logged transitions and entry callbacks.

    Invalid events are logged and return False rather than raising. That is
    deliberate: an event arriving from a task that is being torn down should not
    turn a controlled shutdown into an exception on the flight path.
    """

    def __init__(self, initial_state: MissionState = MissionState.IDLE,
                 label: str = ""):
        self._state = initial_state
        self._label = label
        self._on_enter: dict[MissionState, list[Callable]] = {}
        self._tasks: set[asyncio.Task] = set()
        self.history: list[tuple[str, str, str]] = []
        logger.info("%sFSM initialised in %s", self._prefix, initial_state.value)

    @property
    def _prefix(self) -> str:
        return f"[{self._label}] " if self._label else ""

    @property
    def state(self) -> MissionState:
        return self._state

    def can_fire(self, event: str) -> bool:
        return event in TRANSITIONS.get(self._state, {})

    def fire(self, event: str) -> bool:
        """Attempt a transition. Returns True if it happened."""
        available = TRANSITIONS.get(self._state, {})
        next_state = available.get(event)

        if next_state is None:
            logger.warning(
                "%sevent '%s' is not valid in %s (valid: %s)",
                self._prefix, event, self._state.value,
                ", ".join(sorted(available)) or "none - terminal state",
            )
            return False

        logger.info(
            "%s%s --[%s]--> %s",
            self._prefix, self._state.value, event, next_state.value,
        )
        self.history.append((self._state.value, event, next_state.value))
        self._state = next_state
        self._run_on_enter(next_state)
        return True

    def force(self, state: MissionState, reason: str) -> None:
        """Jump to a state regardless of the transition table.

        For the safety supervisor only. A safety layer must not be blocked from
        recording an abort because the mission happened to be in a state whose
        row lacks that event -- the drone is coming down either way, and the FSM
        should say so.
        """
        logger.critical(
            "%sFORCED %s -> %s: %s",
            self._prefix, self._state.value, state.value, reason,
        )
        self.history.append((self._state.value, f"force:{reason}", state.value))
        self._state = state
        self._run_on_enter(state)

    def on_enter(self, state: MissionState, callback: Callable) -> None:
        """Register a callback invoked on entry to `state`."""
        self._on_enter.setdefault(state, []).append(callback)

    def _run_on_enter(self, state: MissionState) -> None:
        for callback in self._on_enter.get(state, []):
            try:
                result = callback()
            except Exception:  # noqa: BLE001 - a callback must not break a transition
                logger.error(
                    "%son_enter callback for %s raised",
                    self._prefix, state.value, exc_info=True,
                )
                continue

            if asyncio.iscoroutine(result):
                try:
                    task = asyncio.create_task(result)
                except RuntimeError:
                    # No running loop: a synchronous caller registered an async
                    # callback. Close the coroutine so it does not warn about
                    # never being awaited.
                    result.close()
                    logger.warning(
                        "%sasync on_enter callback for %s skipped: no event loop",
                        self._prefix, state.value,
                    )
                    continue
                # Hold the reference: an unreferenced task can be collected
                # before it runs, and its exception disappears with it.
                self._tasks.add(task)
                task.add_done_callback(self._tasks.discard)

    def reset(self) -> None:
        self._state = MissionState.IDLE
        self.history.clear()
