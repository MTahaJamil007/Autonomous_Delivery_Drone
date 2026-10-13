"""Annotated downward-camera frames for the dashboard.

Draws what the vision bridge actually concluded from a frame, on top of the
frame itself: every decoded marker's outline and id, each pad's fused centre
estimate with its range and metric offset, and a reticle at the image centre
the landing loop steers toward. The line from the reticle to a pad centre IS
the landing error, so an operator can watch the controller close it.

Runs on a livefeed viewer thread, never in the Gazebo callback (finding F5),
and only while someone is watching. Pure functions over numpy arrays, so the
tests can render a synthetic frame without Gazebo.
"""

from __future__ import annotations

import time

import cv2
import numpy as np

OUTPUT_WIDTH_PX = 640
"""The camera is 320x240. Doubling it before drawing keeps text and outlines
crisp; drawing at native size and letting the browser scale smears both."""

JPEG_QUALITY = 82

FONT = cv2.FONT_HERSHEY_SIMPLEX
BOLD = cv2.FONT_HERSHEY_DUPLEX
AA = cv2.LINE_AA

# BGR. One colour per pad role, the same as the dashboard's map pins
# (index.html PAD: pickup #d95926, drop #199e70, home #9085e9).
PAD_STYLE = {
    0: ("PICKUP", (38, 89, 217)),
    1: ("DROP-OFF", (112, 158, 25)),
    2: ("HOME", (233, 133, 144)),
}
UNKNOWN_STYLE = ("UNKNOWN", (180, 180, 180))
WHITE = (245, 245, 245)
MUTED = (175, 175, 175)
AMBER = (40, 190, 255)
RED = (70, 70, 235)


def pad_style(pad_id: int) -> tuple[str, tuple[int, int, int]]:
    return PAD_STYLE.get(pad_id, UNKNOWN_STYLE)


def marker_pad(marker_id: int) -> int | None:
    """Which pad a marker id belongs to (world.pad_layout's numbering scheme):
    centre marker p has id p, outer ring of pad p is 10p+10 .. 10p+13."""
    if marker_id in PAD_STYLE:
        return marker_id
    if marker_id >= 10:
        pad = (marker_id - 10) // 10
        if pad in PAD_STYLE:
            return pad
    return None


def _shade(image: np.ndarray, x0: int, y0: int, x1: int, y1: int, alpha: float = 0.55) -> None:
    """Darken a rectangle in place, for legible text over any background."""
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(image.shape[1], x1), min(image.shape[0], y1)
    if x1 <= x0 or y1 <= y0:
        return
    region = image[y0:y1, x0:x1]
    region[:] = (region * (1.0 - alpha)).astype(np.uint8)


def _text(image, text, org, scale=0.5, colour=WHITE, thickness=1, font=FONT) -> int:
    """Draw text with a soft shadow; returns its width in pixels."""
    x, y = org
    cv2.putText(image, text, (x + 1, y + 1), font, scale, (0, 0, 0), thickness + 1, AA)
    cv2.putText(image, text, (x, y), font, scale, colour, thickness, AA)
    return cv2.getTextSize(text, font, scale, thickness)[0][0]


def _label(image, text, org, colour, scale=0.45) -> None:
    """A filled tag: coloured box, dark text. Used for marker ids and pad names."""
    (w, h), base = cv2.getTextSize(text, BOLD, scale, 1)
    x, y = org
    x = int(min(max(0, x), image.shape[1] - w - 8))
    y = int(min(max(h + 6, y), image.shape[0] - 2))
    cv2.rectangle(image, (x, y - h - 6), (x + w + 8, y + base - 2), colour, -1, AA)
    cv2.putText(image, text, (x + 4, y - 3), BOLD, scale, (20, 20, 20), 1, AA)


def _reticle(image: np.ndarray, cx: int, cy: int) -> None:
    colour = (235, 235, 235)
    gap, arm = 10, 26
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cv2.line(
            image,
            (cx + dx * gap, cy + dy * gap),
            (cx + dx * arm, cy + dy * arm),
            colour,
            1,
            AA,
        )
    cv2.circle(image, (cx, cy), 2, colour, -1, AA)


def _corner_brackets(image: np.ndarray, top: int, bottom: int) -> None:
    h, w = image.shape[:2]
    m, length, colour = 14, 26, (200, 200, 200)
    for x, sx in ((m, 1), (w - m, -1)):
        for y, sy in ((top + m, 1), (bottom - m, -1)):
            cv2.line(image, (x, y), (x + sx * length, y), colour, 2, AA)
            cv2.line(image, (x, y), (x, y + sy * length), colour, 2, AA)


def _dashed_line(image, p0, p1, colour, dash=8) -> None:
    p0, p1 = np.array(p0, float), np.array(p1, float)
    length = float(np.linalg.norm(p1 - p0))
    if length < 1:
        return
    step = (p1 - p0) / length
    for start in np.arange(0, length, dash * 2):
        a = p0 + step * start
        b = p0 + step * min(start + dash, length)
        cv2.line(image, tuple(int(v) for v in a), tuple(int(v) for v in b), colour, 2, AA)


def render_frame(
    image_bgr: np.ndarray,
    raw: list[tuple[int, np.ndarray]],
    pads: list[dict],
    drone_id: str,
    fps: float,
    seq: int,
) -> np.ndarray:
    """The annotated frame. Never mutates image_bgr.

    Args:
        image_bgr: The camera frame at native resolution.
        raw: (marker_id, 4x2 corners) per decoded marker, in native pixels.
        pads: The bridge's per-pad contract dicts ("pads" in the UDP payload).
        drone_id: For the header.
        fps: Measured frame rate of the bridge, for the header.
        seq: The bridge's frame sequence number, for the header.
    """
    src_h, src_w = image_bgr.shape[:2]
    scale = OUTPUT_WIDTH_PX / float(src_w)
    out = cv2.resize(
        image_bgr,
        (OUTPUT_WIDTH_PX, int(round(src_h * scale))),
        interpolation=cv2.INTER_LINEAR,
    )
    h, w = out.shape[:2]
    cx, cy = w // 2, h // 2
    header_h, footer_h = 34, 36

    _corner_brackets(out, header_h, h - footer_h)
    _reticle(out, cx, cy)

    # ── every decoded marker ────────────────────────────────────────────────
    # The id goes INSIDE the outline: markers never overlap, so their labels
    # cannot either -- on a nested pad, labels outside them always collide.
    # A pad's centre marker is left unlabelled when that pad has an estimate:
    # its id is the pad id, and the pad crosshair sits right on top of it.
    estimated = {pad["pad_id"] for pad in pads}
    pad_boxes: dict[int, list[np.ndarray]] = {}
    for marker_id, corners in raw:
        pad = marker_pad(marker_id)
        colour = pad_style(pad)[1] if pad is not None else UNKNOWN_STYLE[1]
        pts = (np.asarray(corners, dtype=np.float32) * scale).astype(np.int32)
        cv2.polylines(out, [pts], True, colour, 2, AA)
        # The first corner is the marker's own top-left: marking it shows the
        # marker's orientation, which is how a yawed pad reads at a glance.
        cv2.circle(out, tuple(int(v) for v in pts[0]), 3, colour, -1, AA)
        if pad is not None:
            pad_boxes.setdefault(pad, []).append(pts)

        side = float(np.linalg.norm(pts[0] - pts[1]))
        if side < 24 or (marker_id == pad and pad in estimated):
            continue
        text = str(marker_id)
        (tw, th), _ = cv2.getTextSize(text, BOLD, 0.45, 1)
        mx, my = pts.mean(axis=0).astype(int)
        cv2.rectangle(
            out,
            (mx - tw // 2 - 4, my - th // 2 - 4),
            (mx + tw // 2 + 4, my + th // 2 + 4),
            colour,
            -1,
            AA,
        )
        cv2.putText(out, text, (mx - tw // 2, my + th // 2), BOLD, 0.45, (20, 20, 20), 1, AA)

    # ── each pad's fused centre estimate, and the error to it ───────────────
    best = None
    for pad in pads:
        name, colour = pad_style(pad["pad_id"])
        px = int(cx + pad["err_x"] * scale)
        py = int(cy + pad["err_y"] * scale)
        _dashed_line(out, (cx, cy), (px, py), colour)
        cv2.circle(out, (px, py), 13, colour, 2, AA)
        cv2.line(out, (px - 19, py), (px + 19, py), colour, 2, AA)
        cv2.line(out, (px, py - 19), (px, py + 19), colour, 2, AA)

        # The readout sits beside the pad's markers, never over them: to the
        # right if it fits, else to the left.
        dx = pad["err_x"] / pad["px_per_m"]
        dy = pad["err_y"] / pad["px_per_m"]
        title = f"PAD {pad['pad_id']}  {name}"
        lines = [f"range {pad['range_m']:.2f} m", f"dx {dx:+.2f} m", f"dy {dy:+.2f} m"]
        block_w = (
            max(
                cv2.getTextSize(t, f, sc, 1)[0][0]
                for t, f, sc in [(title, BOLD, 0.45), *((ln, FONT, 0.45) for ln in lines)]
            )
            + 12
        )
        boxes = pad_boxes.get(pad["pad_id"])
        if boxes:
            allpts = np.vstack(boxes)
            left, top_y = allpts.min(axis=0)
            right, bottom_y = allpts.max(axis=0)
        else:
            left, right, top_y, bottom_y = px - 20, px + 20, py - 20, py + 20
        bx = right + 14 if right + 14 + block_w < w else left - 14 - block_w
        bx = int(max(4, bx))
        by = int(min(max(header_h + 24, (top_y + bottom_y) // 2 - 30), h - footer_h - 64))
        _shade(out, bx - 4, by - 18, bx + block_w, by + 58, 0.55)
        _label(out, title, (bx, by), colour, 0.45)
        for i, line in enumerate(lines):
            _text(out, line, (bx + 2, by + 18 + 17 * i), 0.45, WHITE)

        if best is None or len(pad["markers"]) > len(best["markers"]):
            best = pad

    # ── header ──────────────────────────────────────────────────────────────
    _shade(out, 0, 0, w, header_h, 0.6)
    cv2.circle(out, (16, 17), 5, RED, -1, AA)
    x = 28 + _text(out, "LIVE", (28, 22), 0.48, WHITE, 1, BOLD)
    _text(out, f"   {drone_id.upper()}   DOWNWARD CAMERA", (x, 22), 0.48, MUTED)
    right = f"{fps:4.1f} fps   #{seq}   {time.strftime('%H:%M:%S')}"
    (rw, _), _ = cv2.getTextSize(right, FONT, 0.45, 1)
    _text(out, right, (w - rw - 12, 22), 0.45, MUTED)

    # ── footer: the one-line verdict ────────────────────────────────────────
    _shade(out, 0, h - footer_h, w, h, 0.6)
    if best is not None:
        name, colour = pad_style(best["pad_id"])
        offset = (
            np.hypot(best["err_x"], best["err_y"]) / best["px_per_m"] if best["px_per_m"] else 0.0
        )
        verdict = (
            f"LOCKED  PAD {best['pad_id']} {name}  |  {len(best['markers'])} markers  |  "
            f"{best['range_m']:.2f} m  |  off {offset:.2f} m"
        )
        if best.get("clipped"):
            verdict += "   (edge)"
    elif raw:
        colour = AMBER
        verdict = f"MARKERS {sorted(m for m, _ in raw)} DECODED - NO PAD MATCH"
    else:
        colour = AMBER
        verdict = "SEARCHING - NO MARKERS IN VIEW"
    cv2.rectangle(out, (0, h - footer_h), (5, h), colour, -1)
    _text(out, verdict, (16, h - 13), 0.5, colour, 1, BOLD)

    return out


def no_signal_frame(drone_id: str, age_s: float | None) -> np.ndarray:
    """Shown in place of a frozen last frame when the camera goes quiet."""
    h = int(OUTPUT_WIDTH_PX * 3 / 4)
    w = OUTPUT_WIDTH_PX
    gradient = np.linspace(38, 18, h, dtype=np.float32)[:, None, None]
    out = np.broadcast_to(gradient, (h, w, 3)).astype(np.uint8).copy()
    _corner_brackets(out, 0, h)

    title = "NO SIGNAL"
    (tw, _), _ = cv2.getTextSize(title, BOLD, 1.2, 2)
    _text(out, title, ((w - tw) // 2, h // 2 - 6), 1.2, RED, 2, BOLD)
    detail = (
        f"{drone_id} camera silent for {age_s:.0f} s"
        if age_s is not None
        else f"waiting for the first frame from {drone_id}"
    )
    (dw, _), _ = cv2.getTextSize(detail, FONT, 0.55, 1)
    _text(out, detail, ((w - dw) // 2, h // 2 + 30), 0.55, MUTED)
    hint = "is the drone spawned?  is perception.vision_bridge running?"
    (hw, _), _ = cv2.getTextSize(hint, FONT, 0.45, 1)
    _text(out, hint, ((w - hw) // 2, h // 2 + 56), 0.45, (130, 130, 130))
    return out


def encode_jpeg(image: np.ndarray, quality: int = JPEG_QUALITY) -> bytes | None:
    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buffer.tobytes() if ok else None
