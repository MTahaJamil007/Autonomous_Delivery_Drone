"""
Mission Finite State Machine for autonomous delivery drone.

Provides state management and transition logic for the entire mission lifecycle,
from idle through multiple flight legs to completion or abort.
"""

from enum import Enum
from typing import Dict, Optional, Callable
import logging

logger = logging.getLogger(__name__)


class MissionState(Enum):
    """Mission states representing the lifecycle of a delivery mission."""
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


# Transition table: {current_state: {event: next_state}}
TRANSITIONS: Dict[MissionState, Dict[str, MissionState]] = {
    MissionState.IDLE: {
        "start": MissionState.TAKEOFF
    },
    MissionState.TAKEOFF: {
        "altitude_reached": MissionState.ENROUTE,
        "fail_x2": MissionState.ABORT
    },
    MissionState.ENROUTE: {
        "arrived": MissionState.SEARCHING,
        "battery_low": MissionState.ABORT
    },
    MissionState.SEARCHING: {
        "marker_locked": MissionState.APPROACH,
        "search_exhausted": MissionState.HOVER_AND_ALERT
    },
    MissionState.APPROACH: {
        "centered_stable": MissionState.DESCEND,
        "lock_lost": MissionState.SEARCHING
    },
    MissionState.DESCEND: {
        "touchdown": MissionState.PAYLOAD_OP,
        "marker_drift": MissionState.APPROACH
    },
    MissionState.PAYLOAD_OP: {
        "op_confirmed": MissionState.NEXT_LEG,
        "op_failed_x2": MissionState.HOVER_AND_ALERT
    },
    MissionState.NEXT_LEG: {
        "more_legs": MissionState.TAKEOFF,
        "mission_complete": MissionState.DONE
    },
    MissionState.HOVER_AND_ALERT: {
        "operator_override": MissionState.SEARCHING,
        "geofence_timeout": MissionState.ABORT
    },
}


class MissionFSM:
    """
    Finite State Machine for mission control.
    
    Manages state transitions and provides hooks for state entry callbacks.
    Invalid transitions are logged but do not raise exceptions, allowing
    graceful degradation.
    """
    
    def __init__(self, initial_state: MissionState = MissionState.IDLE):
        """Initialize FSM with the given initial state."""
        self._state = initial_state
        self._on_enter_callbacks: Dict[MissionState, list[Callable]] = {}
        logger.info(f"Mission FSM initialized in state: {self._state.value}")
    
    @property
    def state(self) -> MissionState:
        """Get current state."""
        return self._state
    
    def fire(self, event: str) -> bool:
        """
        Attempt to fire an event and transition to the next state.
        
        Args:
            event: Event name to fire
            
        Returns:
            True if transition succeeded, False if invalid event for current state
        """
        current_transitions = TRANSITIONS.get(self._state, {})
        next_state = current_transitions.get(event)
        
        if next_state is None:
            logger.warning(
                f"Invalid transition: event '{event}' not allowed in state "
                f"'{self._state.value}'. Valid events: {list(current_transitions.keys())}"
            )
            return False
        
        logger.info(
            f"State transition: {self._state.value} --[{event}]--> {next_state.value}"
        )
        
        old_state = self._state
        self._state = next_state
        
        # Trigger on_enter callbacks for the new state
        self._trigger_on_enter(next_state)
        
        return True
    
    def on_enter(self, state: MissionState, callback: Callable) -> None:
        """
        Register a callback to be invoked when entering a specific state.
        
        Args:
            state: The state to watch for entry
            callback: Function to call when entering that state (no args)
        """
        if state not in self._on_enter_callbacks:
            self._on_enter_callbacks[state] = []
        self._on_enter_callbacks[state].append(callback)
        logger.debug(f"Registered on_enter callback for state: {state.value}")
    
    def _trigger_on_enter(self, state: MissionState) -> None:
        """Trigger all on_enter callbacks for the given state."""
        import asyncio  # R2.6: Import asyncio for coroutine check
        callbacks = self._on_enter_callbacks.get(state, [])
        for callback in callbacks:
            try:
                result = callback()
                # R2.6: Support async callbacks
                if asyncio.iscoroutine(result):
                    asyncio.create_task(result)
            except Exception as e:
                logger.error(
                    f"Error in on_enter callback for state {state.value}: {e}",
                    exc_info=True
                )
    
    def reset(self) -> None:
        """Reset FSM to IDLE state."""
        logger.info("Resetting FSM to IDLE")
        self._state = MissionState.IDLE
