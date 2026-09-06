"""Nested multi-scale ArUco landing pads: geometry, IDs and texture generation.

WHY A NESTED PAD AND NOT ONE BIG MARKER
=======================================
The single 1.592 m marker on the old 2.0 m pad is *undetectable below 2.5 m*,
and that one fact is why precision landing never worked.

A marker of side `s`, centred under a camera of vertical half-FOV `v`, is fully
inside the frame -- corners AND quiet zone, both of which ArUco's contour stage
requires -- only while

    altitude >= s * (1 + 2 * QUIET_ZONE_FRACTION) / (2 * tan(v))

For the shipped camera (320x240, hfov 1.047 rad, so tan(v) = 0.4330) and the old
1.592 m marker that floor is 2.76 m. Measured against the real detector and the
project's own texture, lock was first lost at 2.52 m centred and at 3.42 m with
8 degrees of airframe tilt, and was impossible below 1.8 m in every condition.

The landing loop reacted to that loss by falling into its GPS-spiral search
branch, which commands a *climb* back to config.SEARCH_ALT_M. So the drone
descended, went blind at ~2.5 m, climbed, re-acquired, descended, went blind --
the limit cycle the operator saw as "hovers around it and moves down in a
spiral, never lands". A closed-loop simulation of the shipped controller lands
in 0 of 12 runs and parks at 2.56 m; the same controller with the marker visible
all the way down lands in 12 of 12.

THE FIX: FIVE MARKERS, TWO SCALES, ONE PAD
------------------------------------------
The constraint that makes this interesting is that low altitude shrinks the
footprint *around the pad centre*, so the close-range marker has to BE at the
centre -- exactly where a long-range marker would also want to be. Resolving it
needs two scales at different radii:

    id      side     centre        in frame at      decodes up to
    ----    ----     ----------    -------------    -------------
    C       0.40 m   ( 0.00, 0.00)   >= 0.69 m         4.44 m
    E/W     0.58 m   (+-0.65, 0.00)  >= 1.88 m         6.43 m
    N/S     0.58 m   ( 0.00,+-0.65)  >= 2.51 m         6.43 m

The union covers 0.69 m to 6.4 m continuously, with a 1.88-4.44 m overlap where
both scales are readable. Measured against the real detector: at least one
marker decodes from 0.50 m to beyond 8 m. Below 0.69 m the controller stops
needing vision at all and commits open-loop, which is what PX4's PrecLand
(PLD_FAPPR_ALT) and ArduPilot's equivalent both do.

The E/W pair sits on the image's *wide* axis, so it stays in frame 0.63 m lower
than the N/S pair. That asymmetry is deliberate: it buys overlap for free.

ANY ONE MARKER IS ENOUGH
------------------------
Every marker's position on the pad is known, so a single detected marker fixes
the pad centre: its four image corners and its four known pad-frame corners
determine a homography, and the pad centre is that homography applied to (0, 0).
The same marker's apparent side length gives the scale in pixels per metre, so
the offset comes out in *metres without needing an altitude estimate at all*.
See perception.pad_estimator.

IDS
---
Centre markers keep ids 0, 1 and 2, so world.marker_models.ROLE_TO_ID and every
caller that says "land on marker 1" are unchanged. The outer ring of pad p takes
ids 10p+10 .. 10p+13, which keeps the three pads mutually distinguishable inside
the single DICT_4X4_50 dictionary the whole system uses.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# ─────────────────────────────────────────────────────────────────────────────
#  Geometry
# ─────────────────────────────────────────────────────────────────────────────

PAD_SIZE_M = 2.0
"""Side of the square pad plane. Must match sim/models/pad_N/model.sdf."""

CENTRE_MARKER_M = 0.40
"""Side of the close-range centre marker.

Sized from the two competing limits. Larger keeps it decodable higher up
(h <= fx * s / MIN_DECODE_PX) but pushes up the altitude at which it overflows
the frame (h >= s * 1.5 / (2 tan v)); 0.40 m puts those at 4.44 m and 0.69 m,
which brackets the outer ring's 1.88 m floor with room to spare.
"""

OUTER_MARKER_M = 0.58
"""Side of each of the four long-range markers.

The largest that fits: centred at 0.65 m with a quiet zone of a quarter of its
side, a marker of side s spans 0.65 + 0.75 s from the pad centre, which must
stay inside the 1.0 m pad half-width. s = 0.58 leaves 0.915 m, an 85 mm margin.
"""

OUTER_RADIUS_M = 0.65
"""Distance from pad centre to each outer marker's centre."""

QUIET_ZONE_FRACTION = 0.25
"""White margin around each marker, as a fraction of its side, on every side.

ArUco's contour stage cannot segment a marker flush against another feature, so
this is not cosmetic. A quarter of the side is one and a half cells of a 4x4
marker's 6x6 grid -- comfortably above the one cell the format requires.
"""

MIN_DECODE_PX = 25.0
"""Apparent side below which a 4x4 marker is treated as unreadable.

Conservative: the real detector manages rather less. Used for *predicting*
coverage, never for discarding a marker the detector actually decoded.
"""

ARUCO_DICTIONARY = "DICT_4X4_50"
"""The one dictionary the whole system uses. 4x4 carries the fewest bits and so
survives the smallest apparent size, which is what matters on a 320x240 frame.
Do not widen it: the textures also decode under DICT_ARUCO_MIP_36h12, so a
multi-dictionary detector reports phantom markers."""


@dataclass(frozen=True)
class Marker:
    """One marker on a pad.

    Coordinates are metres in the PAD FRAME: origin at the pad centre, +x to the
    right of the texture as drawn, +y up it. The frame is only ever used
    self-consistently -- to turn detected corners into a pad centre -- so it
    needs no relationship to the world frame, and deliberately does not have one.
    """

    marker_id: int
    side_m: float
    x_m: float
    y_m: float

    def corners_pad_frame(self) -> list[tuple[float, float]]:
        """The four corners in ArUco's own order: TL, TR, BR, BL.

        cv2.aruco.detectMarkers returns image corners in this order for the
        marker as drawn, and the textures draw every marker upright and
        axis-aligned, so corner i here corresponds to detected corner i. That
        correspondence is what makes the homography in
        perception.pad_estimator a four-point exact solve rather than a fit.
        """
        h = self.side_m / 2.0
        return [
            (self.x_m - h, self.y_m + h),  # top-left
            (self.x_m + h, self.y_m + h),  # top-right
            (self.x_m + h, self.y_m - h),  # bottom-right
            (self.x_m - h, self.y_m - h),  # bottom-left
        ]

    def altitude_band_m(self, fx_px: float, tan_h: float, tan_v: float) -> tuple[float, float]:
        """(lowest, highest) altitude at which this marker is usable.

        Lowest: the altitude below which the marker plus its quiet zone no
        longer fits the frame, with the drone over the PAD CENTRE -- which is
        where the controller holds it, and which is why an off-centre marker's
        floor is set by its radius, not just its size.

        Highest: the altitude above which it shrinks past MIN_DECODE_PX.
        """
        reach = self.side_m * (1.0 + 2.0 * QUIET_ZONE_FRACTION) / 2.0
        lowest = max((abs(self.x_m) + reach) / tan_h, (abs(self.y_m) + reach) / tan_v)
        return lowest, fx_px * self.side_m / MIN_DECODE_PX


def outer_ids(pad_index: int) -> tuple[int, int, int, int]:
    """The four outer-ring ids for a pad, as (west, east, south, north)."""
    base = 10 * (pad_index + 1)
    return base + 0, base + 1, base + 2, base + 3


def pad_markers(pad_index: int) -> list[Marker]:
    """Every marker on pad `pad_index`, centre first.

    Centre first is load-bearing for readability only; pad_estimator weights by
    apparent size rather than by position in this list.
    """
    west, east, south, north = outer_ids(pad_index)
    r = OUTER_RADIUS_M
    return [
        Marker(pad_index, CENTRE_MARKER_M, 0.0, 0.0),
        Marker(west, OUTER_MARKER_M, -r, 0.0),
        Marker(east, OUTER_MARKER_M, +r, 0.0),
        Marker(south, OUTER_MARKER_M, 0.0, -r),
        Marker(north, OUTER_MARKER_M, 0.0, +r),
    ]


def marker_index() -> dict[int, tuple[int, Marker]]:
    """Every marker of every pad, keyed by ArUco id -> (pad_index, Marker).

    One flat lookup is what lets the detector answer "which pad is this, and
    where on it am I looking?" from a bare id, without the caller knowing the
    layout.
    """
    index: dict[int, tuple[int, Marker]] = {}
    for pad_index in range(PAD_COUNT):
        for marker in pad_markers(pad_index):
            if marker.marker_id in index:
                raise ValueError(
                    f"marker id {marker.marker_id} is claimed by pad "
                    f"{index[marker.marker_id][0]} and pad {pad_index}; ids must "
                    f"be unique across all pads or select_target cannot "
                    f"disambiguate them"
                )
            index[marker.marker_id] = (pad_index, marker)
    return index


PAD_COUNT = 3
"""Pickup, drop-off and home. Matches world.marker_models.LEG_ORDER."""


def visibility_floor_m(
    fx_px: float,
    tan_h: float,
    tan_v: float,
    offset_m: float = 0.0,
    pad_index: int = 0,
) -> float:
    """Lowest altitude at which SOME marker is still fully in frame.

    `offset_m` is how far the pad centre may be from the optical axis. It
    matters more than it looks: the frame shrinks linearly with altitude, so
    tolerating a 0.15 m offset near the ground costs 0.35 m of altitude. A
    commit gate written as "descend below H and be within R" is unsatisfiable
    unless H is derived from R this way -- the drone would go blind at exactly
    the moment it was asked to prove it was centred.

    Worst case over the axes: an offset can be in any direction, and the
    vertical field is the narrower one.
    """
    best = float("inf")
    for marker in pad_markers(pad_index):
        reach = marker.side_m * (1.0 + 2.0 * QUIET_ZONE_FRACTION) / 2.0
        needed = max(
            (abs(marker.x_m) + offset_m + reach) / tan_h,
            (abs(marker.y_m) + offset_m + reach) / tan_v,
        )
        best = min(best, needed)
    return best


def coverage_m(fx_px: float, tan_h: float, tan_v: float, pad_index: int = 0) -> tuple[float, float]:
    """(lowest, highest) altitude at which AT LEAST ONE marker is usable.

    The lowest is the best any single marker manages, because one marker is
    enough. Raises if the individual bands leave a hole, since a gap in the
    middle of the descent is exactly the defect this layout exists to remove.
    """
    bands = [m.altitude_band_m(fx_px, tan_h, tan_v) for m in pad_markers(pad_index)]
    bands.sort()

    reach = bands[0][1]
    for low, high in bands[1:]:
        if low > reach:
            raise ValueError(
                f"pad {pad_index} has a coverage gap between {reach:.2f} m and "
                f"{low:.2f} m: no marker is both in frame and decodable there, "
                f"so the descent would go blind mid-way. Grow the centre marker "
                f"or shrink OUTER_RADIUS_M."
            )
        reach = max(reach, high)

    return min(b[0] for b in bands), reach


# ─────────────────────────────────────────────────────────────────────────────
#  Texture generation
# ─────────────────────────────────────────────────────────────────────────────

TEXTURE_PX = 1000
"""Texture resolution for the 2.0 m pad: 500 px/m.

Generous, because the marker is resampled twice -- once by Gazebo onto the plane
and once by the camera -- and each resampling softens the cell edges that the
decoder thresholds.
"""


def texture_pixels_per_m() -> float:
    return TEXTURE_PX / PAD_SIZE_M


def render_pad_texture(pad_index: int):
    """The pad's texture as a white image with its five markers drawn on it.

    Imported lazily so this module stays importable -- and the geometry above
    stays unit-testable -- on a machine with no OpenCV.
    """
    import cv2
    import numpy as np

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICTIONARY))
    image = np.full((TEXTURE_PX, TEXTURE_PX), 255, np.uint8)
    ppm = texture_pixels_per_m()

    for marker in pad_markers(pad_index):
        side_px = int(round(marker.side_m * ppm))
        drawn = cv2.aruco.generateImageMarker(dictionary, marker.marker_id, side_px)

        # Pad frame is +y up; image rows run downwards, so y flips here. This is
        # the ONLY place the two conventions meet, which is why
        # Marker.corners_pad_frame can stay in pad-frame terms throughout.
        left = int(round((marker.x_m + PAD_SIZE_M / 2.0) * ppm - side_px / 2.0))
        top = int(round((PAD_SIZE_M / 2.0 - marker.y_m) * ppm - side_px / 2.0))

        if left < 0 or top < 0 or left + side_px > TEXTURE_PX or top + side_px > TEXTURE_PX:
            raise ValueError(
                f"marker {marker.marker_id} ({marker.side_m} m at "
                f"({marker.x_m}, {marker.y_m})) does not fit the "
                f"{PAD_SIZE_M} m pad. Reduce OUTER_MARKER_M or OUTER_RADIUS_M."
            )

        image[top : top + side_px, left : left + side_px] = drawn

    return image


def describe(fx_px: float, tan_h: float, tan_v: float) -> str:
    """A human-readable coverage table. Printed by the texture build script and
    by scripts/preflight.py, so the altitude budget is visible at the moment
    someone changes it rather than discovered in flight."""
    lines = [
        f"pad {PAD_SIZE_M} m | centre {CENTRE_MARKER_M} m | "
        f"outer {OUTER_MARKER_M} m at r={OUTER_RADIUS_M} m | fx={fx_px:.1f} px",
        f"  {'id':>4} {'side':>6} {'position':>16} {'in frame >=':>13} {'decodes <=':>12}",
    ]
    for marker in pad_markers(0):
        low, high = marker.altitude_band_m(fx_px, tan_h, tan_v)
        lines.append(
            f"  {marker.marker_id:>4} {marker.side_m:6.2f} "
            f"({marker.x_m:+5.2f},{marker.y_m:+5.2f}) m {low:11.2f} m {high:10.2f} m"
        )
    low, high = coverage_m(fx_px, tan_h, tan_v)
    lines.append(f"  union: at least one marker usable from {low:.2f} m to {high:.2f} m")
    return "\n".join(lines)


def _tan_half_fov(width_px: int, height_px: int, hfov_rad: float) -> tuple[float, float, float]:
    """(fx, tan of horizontal half-FOV, tan of vertical half-FOV)."""
    fx = (width_px / 2.0) / math.tan(hfov_rad / 2.0)
    return fx, math.tan(hfov_rad / 2.0), (height_px / 2.0) / fx
