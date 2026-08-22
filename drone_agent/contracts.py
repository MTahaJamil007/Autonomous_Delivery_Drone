"""Shared data contracts between the dispatcher and the mission layer.

WHY DATACLASSES AND NOT POSITIONAL ARGUMENTS
--------------------------------------------
The dispatcher called

    execute_delivery(pickup_lat, pickup_lon, drop_lat, drop_lon, drone_id)

with five positional arguments, against an implementation that accepted four.
Python raised TypeError at call time, the traceback was swallowed by a broad
`except Exception`, and the mission was recorded as COMPLETED. Four floats of
the same type in a row is a signature that cannot be checked by eye or by
tooling: swap pickup and drop-off and everything still runs, just to the wrong
place.

A dataclass ends that whole class of bug. Adding a field is a compile-time-ish
error at every construction site instead of a silent shift of every argument
after it.

WHY MissionResult AND NOT AN EXCEPTION OR A BOOL
------------------------------------------------
The dispatcher has to record three distinguishable things: what happened, why,
and how far the mission got. A bool cannot carry a reason, and an exception
cannot carry partial progress. Before this, every outcome including a crash was
written to the database as COMPLETED, so the fleet view was a list of successes
regardless of what the drones actually did.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any


def utc_now_iso() -> str:
    """Current UTC time as an ISO-8601 string with an explicit offset.

    Timezone-aware on purpose. `datetime.utcnow()` -- used at four sites in
    fleet_dispatch/db.py -- returns a NAIVE datetime holding UTC, so comparing
    it against anything timezone-aware raises TypeError, and it is deprecated
    in 3.12+. The confidence-decay maths in the obstacle service parses these
    strings back with fromisoformat(), so a naive/aware mismatch there would
    make every stored obstacle either ageless or infinitely old.
    """
    return datetime.now(timezone.utc).isoformat()


class JobStatus(str, Enum):
    """Lifecycle of a delivery job.

    QUEUED -> ASSIGNED -> IN_PROGRESS -> COMPLETED | FAILED | ABORTED

    Terminal states are deliberately three, not one:

    * COMPLETED - all legs flown, payload delivered, drone home.
    * FAILED    - could not fly, or a leg failed unrecoverably. The drone is
                  believed to be safe (it never armed, or it landed/RTL'd).
    * ABORTED   - the mission was cut short in flight, by the safety supervisor
                  or by cancellation. Needs an operator's eye on the airframe.

    Collapsing FAILED and ABORTED loses exactly the distinction an operator
    needs at 2 a.m.: is there a drone sitting in a field somewhere?
    """

    QUEUED = "QUEUED"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"

    @property
    def is_terminal(self) -> bool:
        return self in (JobStatus.COMPLETED, JobStatus.FAILED, JobStatus.ABORTED)


class DroneStatus(str, Enum):
    """Availability of a drone in the registry."""

    AVAILABLE = "AVAILABLE"
    BUSY = "BUSY"
    OFFLINE = "OFFLINE"
    ERROR = "ERROR"


@dataclass(frozen=True)
class DeliveryJob:
    """One delivery request: fly to pickup, then to drop-off, then home.

    Frozen so a mission cannot mutate the job it was handed. Home is not a
    field: it is wherever the drone actually is when it connects, read from
    telemetry, because a stored home that disagrees with the airframe's is how
    a return-to-home leg ends up somewhere else.
    """

    job_id: str
    pickup_lat: float
    pickup_lon: float
    drop_lat: float
    drop_lon: float

    def __post_init__(self) -> None:
        for name, value, limit in (
            ("pickup_lat", self.pickup_lat, 90.0),
            ("drop_lat", self.drop_lat, 90.0),
            ("pickup_lon", self.pickup_lon, 180.0),
            ("drop_lon", self.drop_lon, 180.0),
        ):
            if not -limit <= value <= limit:
                raise ValueError(f"{name}={value} is not a valid coordinate")

    @property
    def pickup(self) -> tuple[float, float]:
        return self.pickup_lat, self.pickup_lon

    @property
    def drop(self) -> tuple[float, float]:
        return self.drop_lat, self.drop_lon


@dataclass
class LegOutcome:
    """What happened on one leg. Kept so a partial mission is still legible."""

    role: str                     # 'pickup_pad' | 'drop_pad' | 'home_pad'
    marker_id: int
    arrived: bool = False
    landed: bool = False
    payload_op_ok: bool | None = None   # None if the leg has no payload action
    detours: int = 0
    detail: str = ""


@dataclass
class MissionResult:
    """Outcome of a whole delivery mission.

    `status` is what the dispatcher writes to the database; `detail` is what an
    operator reads in the browser. Both are mandatory on a non-success path:
    "FAILED" with no reason is barely better than the old silent COMPLETED.
    """

    status: JobStatus
    detail: str = ""
    legs: list[LegOutcome] = field(default_factory=list)
    final_state: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def completed(cls, legs: list[LegOutcome], state: dict[str, Any] | None = None) -> MissionResult:
        return cls(JobStatus.COMPLETED, "All legs complete.", legs, state or {})

    @classmethod
    def failed(cls, detail: str, legs: list[LegOutcome] | None = None,
               state: dict[str, Any] | None = None) -> MissionResult:
        return cls(JobStatus.FAILED, detail, legs or [], state or {})

    @classmethod
    def aborted(cls, detail: str, legs: list[LegOutcome] | None = None,
                state: dict[str, Any] | None = None) -> MissionResult:
        return cls(JobStatus.ABORTED, detail, legs or [], state or {})

    @property
    def ok(self) -> bool:
        return self.status is JobStatus.COMPLETED
