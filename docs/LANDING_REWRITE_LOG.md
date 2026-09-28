# Precision-landing rewrite — work log

Running log for the ArUco precision-landing audit and rewrite that started from
this operator report:

> "I have run a simulation of delivery but the drone was not able to land. The
> program in which the drone detects the ArUco marker and then lands hovers
> around it and moves down in spiral shape is no good actually. And I have
> witnessed it to fail and it's not working."

The requested work was: audit the landing program, find the issues, research the
best production-proven approach, write that code, and test it.

---

## Session 1 — 5 Sep 2026 — **cancelled mid-execution by the operator**

Cancelled during the final live-Gazebo validation step. Nothing was lost: every
code change was already written to the working tree, and `pytest` was green at
186 passed before the cancellation.

### Completed before the cancellation

| | Item | State at cancellation |
| --- | --- | --- |
| 1 | Audit of the shipped landing stack | Complete — root cause found and proven empirically |
| 2 | Root-cause proof against the real detector | Complete — the shipped pad's texture is undecodable below ~2.5 m |
| 3 | `world/pad_layout.py` — nested multi-scale pad geometry | Written |
| 4 | `perception/pad_estimator.py` — marker→pad-centre estimator | Written, validated to 3–8 mm against ground truth |
| 5 | `drone_agent/landing.py` — controller rewrite | Written |
| 6 | `perception/vision_bridge.py` — per-pad observations | Written |
| 7 | `drone_agent/mission.py` — attitude telemetry for tilt correction | Written |
| 8 | `world/build_pads.py` — pad texture/model generator | Written, pads regenerated |
| 9 | `scripts/preflight.py` — bench gate that would have caught the defect | Written, verified to reject the original defect |
| 10 | Test suite | 186 passed, `make check` green |
| 11 | Live-Gazebo validation of the **new** pad | Complete — lock held across the full descent |

### In flight at the moment of cancellation

- **Live-Gazebo validation of the *old* pad**, run as the controlled contrast to
  item 11 in the same renderer. The comparison run had been set up (legacy pad
  spawned in place of `pad_0`) but its result was never captured.
- **A background research workflow** on production-proven ArUco landing
  techniques and PX4 integration. Four research agents had finished; the
  synthesis stage was still running and its output was never read.

### Cleanup owed by the cancellation

- The Gazebo world had `pad_0` removed and a temporary `pad_legacy` spawned in
  its place. That Gazebo instance is **gone** (no simulator process survived the
  cancellation), and the temporary models lived only in the session scratchpad,
  which has since been cleared. **No repository state was affected** — verified
  by `git status`: the only changes on disk are the intended ones.

---

## Session 2 — 6 Sep 2026 — resumed

Picks up the three open threads above: the legacy-pad contrast, the unread
research synthesis, and the end-to-end delivery flight that the operator's
original report was about.

### Live-Gazebo pad validation — the interrupted contrast, completed

Method: a static probe model with optics identical to `x500_delivery`'s
`downward_camera` (320x240, hfov 1.047, 10 Hz, pitched +90 deg to look straight
down) is teleported to each altitude over a pad at the world origin. Frames come
from Gazebo's own renderer, go through the real `cv2.aruco` detector and the
real `perception.pad_estimator`. Between the two pads *nothing changes but the
texture*, which is what makes it a controlled experiment rather than a flight.

**Decode band, measured (altitudes 0.40 m to 8.00 m):**

| Condition | OLD single-marker pad | NEW nested pad |
| --- | --- | --- |
| Perfectly centred | **2.00 – 8.00 m** | **0.50 – 8.00 m** |
| 0.30 m off centre (realistic) | **2.76 – 8.00 m** | **1.20 – 8.00 m** |

The off-centre figure of 2.76 m matches the analytic prediction
`s * (1 + 2*quiet) / (2*tan(v))` = 2.76 m for the old 1.592 m marker exactly.
The theory and the renderer agree.

**This is the root cause, measured rather than argued.** A drone descending on
the old pad lost the marker at 2.0–2.8 m every time, and the shipped controller
answered that loss by climbing back to `SEARCH_ALT_M`. That is the reported
hover, and no control-law tuning could have fixed it.

`COMMIT_ALT_M` is 1.20 m, derived independently in `config.py` from the pad
geometry. The measured off-centre vision floor is also 1.20 m — the open-loop
commit begins exactly where the measurement stops existing.

**Estimator accuracy against ground truth, new pad, over the whole decode band:**

- centred: worst centre error **26 mm**
- 0.30 m off centre: worst centre error **39 mm** (recovered `v = +0.293 … +0.339` for a true +0.300)
- `range_m` below 3 m: within **±64 mm**

`range_m` is biased high at altitude (+0.73 m at 8 m, where a marker spans only
19 px), because ArUco's corner fit sits inside the black border and that bias is
a fixed fraction of the marker's apparent size. It is harmless here: every gate
that consumes the height — the commit, the descent-rate schedule, `NO_CLIMB_ALT_M`
— acts below 3 m, where the same measurement is good to ±64 mm.
`working_height_m`'s factor-of-two cross-check against the EKF still covers the
case where it is not merely biased but wrong.

Independently confirmed by the same runs: image **+v** corresponds to world
**−X**, i.e. a pad behind the drone — the "image +v → body AFT" mapping in
docs/CALIBRATION.md that the whole control loop's sign convention rests on.

### End-to-end delivery flight #1 — a real defect the bench could not see

The first full delivery dispatch against live PX4 SITL. The landing itself was
everything the rewrite was for:

```
14:56:20  pad 0 locked at 6.97 m via markers [10, 11, 12, 13]
14:56:20  descending: 1.659 m off at 6.97 m (gate 2.591 m)
14:56:27  committing to the landing from 1.17 m, 0.042 m off centre
14:56:39  commit did not reach the ground within 12 s (still -0.03 m up)
```

Acquired on arrival, closed 1.66 m of offset, committed **42 mm** off centre,
and Gazebo ground truth put the airframe stopped on the pad **0.12 m** from its
centre. Then it reported `stalled`, the leg failed, and the mission was recorded
`FAILED`.

**The defect.** `is_on_ground` let an explicit `IN_AIR` from PX4's LandedState
veto the height check. The height was −0.03 m — far inside `TOUCHDOWN_ALT_M`
(0.15 m) — so the veto was the only thing standing between a successful landing
and a failed mission.

**Why `IN_AIR` was reported, and why the veto was exactly backwards.** PX4's
land detector requires sustained low thrust and near-zero velocity. The commit
phase streams a 0.2 m/s *downward velocity setpoint in OFFBOARD* all the way to
the ground. The detector will not declare a landing while it is being commanded
to fly, so under an offboard descent LandedState is not merely late to report
`ON_GROUND` — it never reports it at all. The veto therefore fired precisely in
the one situation where the height was the only signal available, and never in
the situation it was written for (a single noisy sample). It was a guaranteed
deadlock dressed as a safeguard.

Confirmation that it is the offboard stream and not a slow detector: the moment
the loop gave up and released offboard, PX4's own log shows
`Landing detected` / `Disarmed by landing` — the detector latched immediately
once it stopped being commanded.

**The fixes.** Two, each independently sufficient — defence in depth on the one
step that decides whether a mission succeeds:

1. `is_on_ground` — `ON_GROUND` remains authoritative when present, but a low
   height is now sufficient on its own. Noise rejection moves to where it can do
   its job without deadlocking: both callers already require the condition to
   hold for `TOUCHDOWN_PUSH_S` and then confirm via `_confirm_landed`, which
   commands a land and waits for a real disarm.
2. A commit that overruns `COMMIT_TIMEOUT_S` now hands the vehicle to the
   autopilot's own land mode and only reports `stalled` if *that* also fails.
   Leaving offboard is itself what lets the land detector settle. This mirrors
   PX4 PrecLand's and ArduPilot PrecLand's fallback-to-normal-land behaviour.

**Why the test suite missed it.** Every existing test drove a fake autopilot
that latched `ON_GROUND` at 0.05 m. That fake was *kinder than the hardware*,
and a fake more generous than the real thing cannot fail the way the real thing
fails. `World` now takes `land_detector_latches`, and
`test_it_lands_even_when_the_autopilot_never_reports_on_ground` runs the real
coroutine against a plant that never reports it. Verified to be a real
regression test: against the code as it flew it fails with `stalled`, and it
passes with either fix in place.

### SITL battery endurance — a simulator artifact that blocked the test

The same flight tripped `battery warning (fast)` → failsafe → RTL. Cause:
PX4's `SIM_BAT_DRAIN` defaults to **60 seconds** to go from full to
`SIM_BAT_MIN_PCT`, which is shorter than a single leg of a three-leg delivery.
Left alone, every multi-leg mission test becomes a test of the low-battery
failsafe instead of the thing under test.

`sim/set_sim_params.py` now sets it to 3000 s (50 min), a realistic delivery
endurance, and `world/spawn_fleet.sh` calls it after the instances come up. The
battery *gate* in `drone_agent/battery.py` is untouched and still refuses a leg
it cannot afford — this only stops the simulated cell from emptying faster than
the aircraft can fly.

### End-to-end delivery flight #2 — leg 1 lands, and exposes the next defect

With the touchdown fixes in, leg 1 flew the way the rewrite intended:

```
15:08:16  pad 0 locked at 7.10 m via markers [10, 11, 12, 13]
15:08:16  descending: 1.644 m off at 7.10 m (gate 2.635 m)
15:08:22  committing to the landing from 1.14 m, 0.026 m off centre
15:08:28  ground contact at 0.14 m
15:08:33  disarmed - landing confirmed
15:08:33  APPROACH --[touchdown]--> PAYLOAD_OP ; cargo secured
```

**Gazebo ground-truth touchdown error: 0.147 m** from the pad centre (drone
settled at x=24.873, y=+0.014; pad centre at x=25.019, y=+0.033).

Then leg 2 armed, climbed to 9.6 m, announced it was navigating — and hovered
over the pickup pad, motionless, for three minutes:

```
15:08:53  at 9.6 m ... navigating to (30.031293, 72.314083)
15:11:53  still at x=24.85 y=-0.02 z=9.76
```

**The defect: a mission could only ever fly one leg.** Confirming a touchdown
means commanding `action.land()` and waiting for a real disarm, and that takes
PX4 out of OFFBOARD by design — PX4's own log shows
`Landing at current position` → `Disarmed by landing`. The next leg's takeoff
leaves the vehicle in Takeoff/Hold. Nothing ever put it back into offboard.

`SetpointPublisher.start()` returned early whenever its publishing task was
alive. That check was true and beside the point: setpoints *were* streaming at
20 Hz, into a mode that was not listening. **"The publisher is running" and "the
autopilot is following it" are two different facts, and only the second one
matters.**

**The fix.** `SetpointPublisher._ensure_offboard()` asks
`offboard.is_active()` and re-enters only when the autopilot has actually
dropped out. This is not the mode thrashing `stop()` warns against — that
warning is about *voluntarily* leaving offboard between legs; this is recovering
from a mode change we ourselves requested by landing. The stream is already
flowing, which is the handshake PX4 requires, so no re-priming is needed.

Pinned by `test_publisher_re_enters_offboard_after_the_autopilot_leaves_it`
(fails without the fix) and `test_publisher_does_not_thrash_offboard_when_it_is_already_active`
(guards the other direction).

**Why neither defect was reachable before.** Both live only past a *successful*
touchdown. Until the landing itself worked, leg 1 always failed and no flight
ever reached the code that follows it. Fixing the landing did not create these
bugs; it made them reachable for the first time.

### End-to-end delivery flight #3 — the mission flies on, the state machine does not

With offboard re-entry in place, leg 1 landed and **leg 2 started** — the first
time any flight had got that far. It exposed a third defect, this time in
reporting rather than in flight:

```
15:16:47  pad 0 locked at 6.73 m ; SEARCHING --[marker_locked]--> APPROACH
15:16:55  no lock at 1.45 m but already below the commit altitude -
          committing on the last known trim
15:17:13  disarmed - landing confirmed
15:17:13  event 'touchdown' is not valid in SEARCHING
15:17:13  event 'op_confirmed' is not valid in SEARCHING
15:17:16  event 'more_legs' is not valid in SEARCHING
15:17:46  event 'altitude_reached' is not valid in SEARCHING
```

The drone landed and secured its cargo. The state machine that reports on the
mission had been driven back to `SEARCHING` on the way down and then rejected
every subsequent event, so mission state stopped describing the mission.

**The defect.** The loop fired `lock_lost` on *every* lock loss, which takes the
FSM `APPROACH -> SEARCHING`. But below `NO_CLIMB_ALT_M` a lock loss is not a
setback — it is the field of view doing exactly what it must as the drone closes
on the pad — and the loop answers it by holding and committing, never by
climbing away to look again. Announcing "searching again" while committing to a
landing contradicts the controller's own design, and `SEARCHING` has no
`touchdown` transition.

**The fix.** `lock_lost` is now fired only when searching is something this loop
would actually do — that is, only above `NO_CLIMB_ALT_M`. The FSM's transition
table was right; the event was wrong, so the event is what changed.

**Why the suite missed it.** `FakeFSM` records events and returns `True` for all
of them. Only the real transition table can tell a legal event from an illegal
one. `test_a_low_lock_loss_leaves_the_real_fsm_able_to_accept_a_touchdown` now
drives the real `MissionFSM`, and without the fix it fails with the exact log
line from the flight.

### A pattern worth naming

All three defects share one shape: **a test double that was more forgiving than
the real system.**

| | The double said | The real system said |
| --- | --- | --- |
| Touchdown | land detector latches `ON_GROUND` at 0.05 m | never latches while offboard streams a descent |
| Offboard | publisher running ⇒ autopilot following | the autopilot had silently left the mode |
| FSM | every event is accepted | `SEARCHING` rejects `touchdown` |

Each fake was written to model the *intent* of its collaborator rather than its
behaviour, and each one therefore could not fail the way the real thing fails.
The three regression tests added here all work by making the double *stricter*,
and each was verified to fail against the code as it flew.

None of these were reachable before the pad and controller were fixed: they all
live past a successful touchdown, and until this work no flight had ever had one.

### Flight #4 — the offboard fix was right about the cause and wrong about the test

The first offboard-re-entry fix asked `offboard.is_active()` before re-entering.
Leg 2 hung again, in exactly the same way, and this time the log said why by
saying nothing at all:

```
15:22:50  OFFBOARD engaged, streaming at 20 Hz      (leg 1)
15:24:16  at 9.6 m ... navigating to (30.031427, 72.314290)
15:27:14  still parked over the pickup pad at (18.22, -10.01, 9.93)
          -- and not one "re-entered OFFBOARD" line in between
```

**`offboard.is_active()` does not mean what its docstring says.** The docstring
reads "True means that the vehicle is in offboard mode"; MAVSDK implements it as
`_mode != Mode::NotActive` over the **plugin's own local state**, set by
`start()`, cleared by `stop()`, and never touched when the autopilot changes
mode by itself. After a land command it still returns True, so the guard
concluded everything was fine and skipped the re-entry.

A second trap sits directly behind it: MAVSDK's `offboard.start()` returns
`Success` **without sending anything** when it already believes offboard is
active. Even calling `start()` unconditionally would not have helped.

**The corrected fix** asks `telemetry.flight_mode()`, which comes from the
vehicle's HEARTBEAT and reports what PX4 is actually doing, and re-enters with
`stop()` then `start()` so that MAVSDK's stale belief is cleared and the mode
command actually goes out.

`FakeOffboard` now carries two separate facts — `plugin_thinks_active` and
`vehicle_mode` — because a double that collapses them into one boolean cannot
reproduce this. The test asserts the trap explicitly: it checks that
`is_active()` still returns True at the moment the vehicle has left offboard,
then requires the publisher to re-enter anyway.

This is the same lesson as the other three, one level deeper: the first fix
replaced a bad assumption about PX4 with a bad assumption about the MAVSDK
client. What settled it was reading MAVSDK's implementation rather than its
documentation.

### Flight #5 — the re-entry detected the problem and then lost a 35 ms race

Reading the vehicle's real flight mode worked exactly as intended:

```
15:33:31,548  the autopilot is in HOLD, not OFFBOARD - re-entering
15:33:31,563  could not re-enter OFFBOARD: No Setpoint Set
```

The first line is the diagnosis confirmed in flight: after leg 1's land and the
next leg's takeoff, PX4 really was sitting in **HOLD** while MAVSDK's
`is_active()` had been reporting True.

The second line is a third MAVSDK subtlety. `OffboardImpl::start()` returns
`NoSetpointSet` whenever its `_mode` is `NotActive`, and **only the `set_*`
methods clear that — `start()` never does.** So the `stop()` that clears
MAVSDK's stale belief also disarms the handshake, and re-entry has to re-arm it.

The publisher was streaming at 20 Hz, so a setpoint was never more than 50 ms
away — but `stop()` and `start()` ran 15 ms apart and lost the race.

**The fix.** The zero-velocity priming loop that `start()` always did is now a
`_prime()` helper, and re-entry calls it between `stop()` and `start()`. Zero
velocity deliberately: a mode change is not the moment to also begin moving, and
the publishing task resumes the real setpoint on its next tick.

`FakeOffboard` now models `NoSetpointSet` — `start()` raises unless a setpoint
was pushed since the last `stop()` — so
`test_re_entry_primes_setpoints_so_start_is_not_rejected` fails without the
priming call.

**Three MAVSDK behaviours, none of them in the docstrings**, all found by
reading the implementation after a flight contradicted the documentation:

| Call | Docstring says | Actually does |
| --- | --- | --- |
| `offboard.is_active()` | "the vehicle is in offboard mode" | reports the plugin's own flag, not the vehicle's mode |
| `offboard.start()` when it thinks it is active | starts offboard | returns Success, sends nothing |
| `offboard.start()` after `stop()` | starts offboard | `NoSetpointSet` until a `set_*` call |

### Flight #6 — the first complete delivery

```
MISSION 1/7   pickup 20.8 m @ 119 deg   drop 20.5 m @ 104 deg
  leg 1 touchdown on drone-0_pickup_pad: 0.240 m
  leg 2 touchdown on drone-0_drop_pad:   0.466 m
  leg 3 touchdown on drone-0_home_pad:   0.195 m
  -> mission COMPLETED, 3 touchdown(s)
```

Three legs, three markers (0, 1, 2), three touchdowns, cargo picked up and
released, `COMPLETED`. This is the scenario the operator reported as broken,
working end to end for the first time.

All errors are Gazebo ground truth — the drone's own EKF is what the controller
steers on, so it cannot also be the judge.

---

## Session 3 — 7 Sep 2026

### Two rig defects found before any flight

**DDS multicast filled a log and blocked startup.** The rig was relaunched on a
machine whose `enp0s31f6` and `wwan0` interfaces were DOWN. `ROS_LOCALHOST_ONLY`
was `0`, so the ros_gz bridge kept trying multicast discovery and wrote

```
Exception sending a multicast message:Network is unreachable
```

in a tight loop until `ros_gz_bridge_0.log` reached **354 MB**. The rest of
`run_system.sh` never came up behind it, so the dispatcher was absent and the
trial script polled a service that did not exist.

Every process here talks over 127.0.0.1, so `sim/env.sh` now exports
`ROS_LOCALHOST_ONLY=1` — which removes the multicast attempt rather than the
symptom, and is simply the truth about this deployment.

### Corner refinement was requested and never happening

The research sweep turned up an OpenCV detail the earlier tuning study had
missed. OpenCV sizes the sub-pixel search window as
`relativeCornerRefinmentWinSize * (average marker side)`, clamped by
`cornerRefinementWinSize`. **At the default 0.3, a marker must span roughly
32 px before that product reaches even one pixel** — so `CORNER_REFINE_SUBPIX`,
which this project has always set, was silently skipped for every smaller
marker. A descent spends its long-range frames exactly there: a 0.58 m outer
marker spans 19–25 px at 7–8 m.

The earlier study concluded "tuning changes nothing" and it was right about what
it measured — *detection rate* is saturated at 72/72 on the nested pad, because
the pad design already solved visibility. It never measured *corner accuracy*.

Measured on this project's own texture, 72 frames from 1–8 m with two offsets,
two blur levels, noise and tilt, scored through the real estimator against
ground truth:

| config | median | p90 | phantom ids |
| --- | --- | --- | --- |
| default | 8.6 mm | 23.2 mm | 0 |
| **`relativeCornerRefinmentWinSize = 1.0`** | **6.9 mm** | **16.3 mm** | **0** |
| full published "small marker" set | 7.3 mm | 17.6 mm | 1 |

One assignment cuts the p90 by 30%. The rest of the published set was measured
and deliberately **not** adopted: it moved nothing here and its
`errorCorrectionRate = 1.0` bought a phantom id — a bad trade against a
detection rate that is already 72/72. Pinned by
`test_corner_refinement_is_actually_enabled_for_small_markers`.

### The audit found a wedge band the flights never happened to hit

An adversarial audit probe dropped the loop in at 2.5 m with the pad out of
frame and watched it hold for the **full 200 s timeout**, in phase `hold`,
issuing nothing but hold commands.

Between `COMMIT_ALT_M + 0.3` (1.5 m) and `NO_CLIMB_ALT_M` (3.0 m) the loop has
no move left. It is too **low** to climb away and search — `may_climb` is False
by design, because climbing on a lost pad is the exact behaviour that caused the
reported hover — and too **high** for the commit-on-last-trim fallback.

Two things had to be wrong at once for that to survive:

1. The stall watchdog only asked whether a **held** pad was failing to descend.
   With no lock at all it never looked.
2. The hold branch ended in `continue`, which stepped straight over the watchdog
   at the bottom of the loop — so even a widened check would not have run.

Both are fixed: the watchdog now also covers "no lock, and too low to search",
and the hold branch falls through to it. The search branch stays exempt, because
a spiral holds altitude on purpose and charging it with failing to descend would
abort every search.

This is a milder relative of the original bug — it terminates rather than
looping forever — but it spends the battery the return leg needs, doing nothing.
Pinned by `test_losing_the_pad_between_the_commit_and_no_climb_does_not_hover`,
which fails against the code as flown.

### Closing the commit loop: the single largest accuracy win

The baseline sweep separated the two error sources for the first time:

| | mean | max |
| --- | --- | --- |
| offset at commit (what vision achieves) | 0.042 m | 0.065 m |
| **drift during the open-loop commit** | **0.170 m** | **0.211 m** |
| touchdown error | 0.209 m | 0.466 m |

**The controller was four times more accurate than the landing it produced**,
and the whole gap was drift over the last 1.2 m that nothing opposed.

The cause was a wrong instinct expressed correctly. The commit handed the last
stretch to PX4 "because PX4 holds a spot better than this loop can
extrapolate" — true, but it did so by commanding **zero velocity**, and a zero
velocity setpoint asks PX4 to *stop moving*, not to *stay put*. Drift already
accumulated was never taken back.

The commit is open loop with respect to **vision**, because the pad has left the
frame. It never had a reason to be open loop with respect to **position**. It
now holds the spot it committed from, against the drone's own estimate, which is
entirely trustworthy over the four to six seconds involved. This is what PX4's
PrecLand does in its own final descent.

### Measured effect of closing the commit loop

Same seven routes, same rig, same everything but the last 1.2 m:

| | landings | mean | median | max | within 0.5 m |
| --- | --- | --- | --- | --- | --- |
| commit commands zero velocity | 9 | 0.209 m | 0.175 m | 0.466 m | 9/9 |
| **commit holds position** | 11 | **0.082 m** | 0.086 m | **0.116 m** | 11/11 |

**2.5x better on the mean and 4x better on the worst case.** The old worst case
sat at 0.466 m against a 0.5 m criterion — passing with almost no margin, and
one gust from failing. The new worst case is 0.116 m, four times inside it.

The vision-guided phase was never the limit: it centres to 0.042 m mean either
way. The whole difference is that the final metre stopped being unopposed.

### A rig limit worth writing down

The formal twenty-one-landing gate needs the machine mostly to itself. Two
separate runs were spoiled by CPU contention while audit agents were working,
and each time the failure looked like a flight-software bug until it was traced:

* **LiDAR feed staleness.** `obstacle feed silent for 1.1s - holding position
  instead of flying blind`, repeatedly. Navigation was behaving correctly; the
  bridge simply could not sustain 10 Hz.
* **`Preflight Fail: Battery unhealthy`** blocking the next leg's arming, while
  MAVSDK reported the battery at 100% and 16.2 V. PX4's `battery_status`
  publication was gapping under lockstep starvation. It cleared the moment the
  sweep stopped.

Neither is a defect in this code, and both are covered by the fail-closed
behaviour working as designed. But they do mean **flight numbers taken under
load are not trustworthy**, and the four CPUs here are shared with anything else
running. Run the gate on a quiet machine, and check `uptime` and
`gz topic -e -t /world/delivery/stats` before believing a bad result.

The second of those also exposed a message that named the wrong cause:
`arm_and_takeoff` reported "EKF/GPS did not converge" for what the PX4 console
called a battery problem. It now reports which of the three health conditions
actually failed, and says explicitly when GPS and the EKF were fine — the same
class of fix as every other misleading message in this log.

