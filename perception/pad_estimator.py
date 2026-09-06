"""Turn detected ArUco markers into one estimate of where the PAD CENTRE is.

THE IDEA
========
The landing controller does not care where a marker is. It cares where the pad
centre is, and it needs that in metres. A nested pad (world.pad_layout) gives
both from ANY ONE of its five markers, because every marker's position on the
pad is known:

    four image corners  +  four known pad-frame corners  ->  homography H

H maps the pad plane onto the image, so the pad centre is H applied to (0, 0)
whether or not the centre marker itself is visible. That is what keeps the
estimate alive from 8 m -- where only the big outer ring is readable -- down to
0.5 m, where only the small centre marker still fits the frame.

METRES WITHOUT AN ALTITUDE
--------------------------
The old code converted pixels to metres with `err_px * altitude / fx`, which
made every measurement only as good as the EKF's altitude estimate and coupled
the landing loop to a number that drifts. H already carries the scale: its
Jacobian at the pad centre is the local pixels-per-metre, so

    offset_m = offset_px / px_per_m

needs no altitude at all. Altitude comes back out as a *result*
(`range_m = fx / px_per_m`), which is a genuinely independent cross-check on
the autopilot's own estimate rather than a consumer of it.

Using the Jacobian rather than a marker's apparent side length matters once the
pad is off to one side of the frame or the airframe is tilted, because then a
marker's apparent side is foreshortened by an amount that varies across the pad;
the Jacobian is evaluated exactly where the answer is wanted.

DISAGREEMENT IS INFORMATION
---------------------------
With two or more markers in view, each yields an INDEPENDENT estimate of the
same pad centre. Their spread measures the quality of this frame directly -- no
tuning, no model. A confident frame has all estimates within a few centimetres;
a frame with a misdecoded id or a badly refined corner does not, and
`residual_m` reports it so the controller can distrust that sample rather than
steer on it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import world.pad_layout as pad_layout


@dataclass
class PadObservation:
    """Where one pad's centre is, in this frame."""

    pad_index: int
    """Which pad. Equals the centre marker's ArUco id, so it is also the
    marker id every existing caller already asks for."""

    centre_px: tuple[float, float]
    """Pad centre in image coordinates: +u right, +v down, origin top-left."""

    err_px: tuple[float, float]
    """Pad centre relative to the IMAGE centre. The quantity the controller
    steers on, kept in the same convention the old per-marker contract used so
    the body-frame sign mapping in docs/CALIBRATION.md is unchanged."""

    px_per_m: float
    """Local image scale at the pad centre, from the homography Jacobian."""

    range_m: float
    """Distance to the pad plane along the optical axis, fx / px_per_m.
    Derived from the marker geometry alone -- independent of the autopilot."""

    marker_ids: list[int]
    """Which markers contributed. Length 1 is normal and sufficient."""

    residual_m: float
    """Spread of the independent per-marker estimates of the pad centre, in
    metres. 0.0 when only one marker was visible (no cross-check available)."""

    largest_side_px: float
    """Apparent side of the biggest contributing marker. A decode this small is
    the first thing to degrade, so it is worth surfacing."""

    corners_in_frame: bool = True
    """False when a contributing marker touches the image border, which makes
    its corners -- and so the whole estimate -- unreliable."""

    contributions: list[tuple[int, float, float]] = field(default_factory=list)
    """(marker_id, centre_u, centre_v) per marker, for debugging a bad frame."""

    @property
    def offset_m(self) -> tuple[float, float]:
        """(right_m, down_m) of the pad centre in the IMAGE plane, in metres.

        Still image-frame: +x with image u, +y with image v. Rotating this into
        the body frame is the caller's job, because only the caller knows the
        camera mount.
        """
        return (self.err_px[0] / self.px_per_m, self.err_px[1] / self.px_per_m)

    @property
    def offset_norm_m(self) -> float:
        return math.hypot(*self.offset_m)


def _homography(marker: pad_layout.Marker, image_corners: np.ndarray) -> np.ndarray | None:
    """Exact 4-point homography from the pad plane (metres) to the image (px).

    cv2.getPerspectiveTransform rather than findHomography: four exact
    correspondences have a closed-form solution, and a least-squares fit over
    exactly four points is the same answer with more ways to fail.
    """
    import cv2

    source = np.array(marker.corners_pad_frame(), dtype=np.float32)
    destination = np.asarray(image_corners, dtype=np.float32).reshape(4, 2)
    try:
        return cv2.getPerspectiveTransform(source, destination)
    except cv2.error:
        # Degenerate corners (a marker seen edge-on, or a bad refinement) make
        # the transform singular. One unusable marker must not cost us the
        # others, so this is a skip rather than a failure.
        return None


def _project(homography: np.ndarray, x: float, y: float) -> tuple[float, float]:
    """Map a pad-frame point to image pixels."""
    u, v, w = homography @ np.array([x, y, 1.0])
    if abs(w) < 1e-12:
        raise ValueError("homography maps the pad centre to infinity")
    return float(u / w), float(v / w)


def _scale_px_per_m(homography: np.ndarray, x: float, y: float) -> float:
    """Local pixels-per-metre at a pad-frame point, from the homography Jacobian.

    For H mapping (x, y, 1) -> (U, V, W) with u = U/W and v = V/W, the exact
    derivatives are

        du/dx = (h00 - u * h20) / W        dv/dx = (h10 - v * h20) / W
        du/dy = (h01 - u * h21) / W        dv/dy = (h11 - v * h21) / W

    and the local *area* scale is |det J|, so the linear scale is its square
    root. This is the isotropic scale; under a tilt the true scale differs
    slightly between axes, and the geometric mean is the right single number to
    divide a radial offset by.
    """
    h = homography
    _, _, w = h @ np.array([x, y, 1.0])
    if abs(w) < 1e-12:
        return 0.0
    u, v = _project(h, x, y)

    jacobian = np.array(
        [
            [(h[0, 0] - u * h[2, 0]) / w, (h[0, 1] - u * h[2, 1]) / w],
            [(h[1, 0] - v * h[2, 0]) / w, (h[1, 1] - v * h[2, 1]) / w],
        ]
    )
    determinant = abs(float(np.linalg.det(jacobian)))
    return math.sqrt(determinant)


def _mean_side_px(image_corners: np.ndarray) -> float:
    points = np.asarray(image_corners, dtype=np.float64).reshape(4, 2)
    return float(np.mean([np.linalg.norm(points[i] - points[(i + 1) % 4]) for i in range(4)]))


def _touches_border(image_corners: np.ndarray, width: int, height: int, margin_px: float) -> bool:
    """True if any corner sits within `margin_px` of the image edge.

    A marker clipped by the frame still sometimes decodes, but its clipped
    corner is wherever the crop happened to fall rather than where the marker
    actually is -- so its homography is confidently wrong. Excluding it is the
    difference between degrading and lying.
    """
    points = np.asarray(image_corners, dtype=np.float64).reshape(4, 2)
    return bool(
        np.any(points[:, 0] < margin_px)
        or np.any(points[:, 1] < margin_px)
        or np.any(points[:, 0] > width - 1 - margin_px)
        or np.any(points[:, 1] > height - 1 - margin_px)
    )


def estimate_pads(
    detections: list[tuple[int, np.ndarray]],
    width: int,
    height: int,
    fx_px: float,
    *,
    border_margin_px: float = 1.5,
    index: dict[int, tuple[int, pad_layout.Marker]] | None = None,
) -> list[PadObservation]:
    """Collapse per-marker detections into one observation per pad.

    Args:
        detections: (aruco_id, 4x2 image corners) as cv2.aruco.detectMarkers
            returns them, in its TL/TR/BR/BL order.
        width, height: frame size, for the image centre and the border test.
        fx_px: focal length, used only to report range_m.
        border_margin_px: how close to the edge a corner may sit before its
            marker is discarded.
        index: marker id -> (pad, Marker). Defaults to the shipped layout;
            injectable so tests can build a pad without touching the world.

    Returns:
        One PadObservation per pad that contributed at least one usable marker,
        best first (largest apparent marker).
    """
    if index is None:
        index = pad_layout.marker_index()

    centre_u = width / 2.0
    centre_v = height / 2.0

    by_pad: dict[int, list[tuple[int, np.ndarray, pad_layout.Marker]]] = {}
    for aruco_id, corners in detections:
        entry = index.get(int(aruco_id))
        if entry is None:
            # A marker from the dictionary that is not part of any pad. Ignoring
            # it here rather than upstream keeps the detector generic.
            continue
        pad_index, marker = entry
        by_pad.setdefault(pad_index, []).append((int(aruco_id), corners, marker))

    observations: list[PadObservation] = []

    for pad_index, members in by_pad.items():
        estimates: list[tuple[int, float, float, float, float]] = []
        clipped = False

        for aruco_id, corners, marker in members:
            if _touches_border(corners, width, height, border_margin_px):
                clipped = True
                continue

            homography = _homography(marker, corners)
            if homography is None:
                continue

            try:
                pad_u, pad_v = _project(homography, 0.0, 0.0)
            except ValueError:
                continue

            scale = _scale_px_per_m(homography, 0.0, 0.0)
            if scale <= 0.0:
                continue

            estimates.append((aruco_id, pad_u, pad_v, scale, _mean_side_px(corners)))

        if not estimates:
            continue

        # Weight by apparent AREA. A marker twice the side is four times the
        # pixels over which its corners are localised, so its corner noise in
        # metres is a quarter -- which is what an area weight encodes.
        weights = np.array([side**2 for *_, side in estimates], dtype=np.float64)
        weights /= weights.sum()

        pad_u = float(np.sum(weights * np.array([e[1] for e in estimates])))
        pad_v = float(np.sum(weights * np.array([e[2] for e in estimates])))
        px_per_m = float(np.sum(weights * np.array([e[3] for e in estimates])))

        if len(estimates) > 1:
            spread_px = math.sqrt(
                float(
                    np.sum(
                        weights
                        * np.array([(e[1] - pad_u) ** 2 + (e[2] - pad_v) ** 2 for e in estimates])
                    )
                )
            )
            residual_m = spread_px / px_per_m if px_per_m > 0 else 0.0
        else:
            residual_m = 0.0

        observations.append(
            PadObservation(
                pad_index=pad_index,
                centre_px=(pad_u, pad_v),
                err_px=(pad_u - centre_u, pad_v - centre_v),
                px_per_m=px_per_m,
                range_m=fx_px / px_per_m if px_per_m > 0 else float("inf"),
                marker_ids=sorted(e[0] for e in estimates),
                residual_m=residual_m,
                largest_side_px=max(e[4] for e in estimates),
                corners_in_frame=not clipped,
                contributions=[(e[0], e[1], e[2]) for e in estimates],
            )
        )

    observations.sort(key=lambda o: o.largest_side_px, reverse=True)
    return observations


def select_pad(observations: list[PadObservation], expected_pad: int) -> PadObservation | None:
    """The observation for the pad we were sent to, or None.

    Three pads stand within metres of each other, so this filter is the whole of
    marker disambiguation -- and it works only because the producer emits every
    pad it saw rather than just the first.
    """
    return next((o for o in observations if o.pad_index == expected_pad), None)
