#!/usr/bin/env python3
"""Downward-camera ArUco detector: Gazebo images in, JSON detections out over UDP.

Output contract (one datagram per frame, see docs/ARCHITECTURE.md):

    {"t": <monotonic s>, "seq": <int>, "w": 320, "h": 240, "fx": 277.19,
     "drone_id": "drone-0",
     "pads": [
        {"pad_id": 0, "err_x": -12.4, "err_y": 5.1, "px_per_m": 92.4,
         "range_m": 3.00, "residual_m": 0.0035, "markers": [0, 10, 11],
         "largest_side_px": 53.6, "clipped": false}
     ],
     "detections": [
        {"id": 0, "err_x": -12, "err_y": 5, "area_px": 3021,
         "size_px": 55.0, "corners": [[x,y], ...]}
     ]}

"pads" IS WHAT THE LANDING LOOP READS. Each entry folds every visible marker of
one pad into a single estimate of that PAD'S CENTRE, which is the thing the
controller actually steers to -- a marker is only ever a means of finding it.
Because each marker's real size is known, the entry also carries `px_per_m`, so
a consumer converts to metres without needing the drone's altitude, and gets
`range_m` back as an independent measurement of height above the pad. See
perception.pad_estimator.

"detections" is the per-marker view, kept for debugging, for
scripts/preflight.py and for the legacy landing.select_target() contract.

A datagram is sent for EVERY frame, including frames with no detections. That is
the only way a consumer can tell "the pad is not in view" from "the bridge is
dead" -- the original protocol sent the literal string "None" for no marker and
nothing at all when the process died, which are indistinguishable to a reader
that is not tracking time.

WHAT CHANGED AND WHY
--------------------
1. PER-PAD ESTIMATES, not just per-marker offsets. The landing loop used to take
   one marker's centroid and scale it by the EKF altitude. It now gets the pad
   centre from a homography over any marker's four corners, which is what lets
   the nested pad (world.pad_layout) stay measurable from 8 m down to 0.5 m --
   the single-marker pad it replaces was undetectable below about 2.5 m, and
   that is the whole reason precision landing never worked.

2. ALL detections are emitted. The original code took corners[0][0] and
   ids[0][0] -- the first marker only. With three pads within metres of each
   other, discarding every marker but the first made "land on ID 1"
   unimplementable.

3. The dictionary and detector are built ONCE. They were reconstructed inside
   the per-frame callback, at 10 Hz, forever. The pad marker index is likewise
   built once, for the same reason.

4. ONE detection pass per frame. detect_raw() runs the detector; the pad
   estimator and the per-marker contract view both consume its output. Detecting
   twice to produce two views of the same answer would halve the frame rate the
   landing loop depends on.

5. The image centre comes from msg.width/msg.height. Hardcoded 160/120 happened
   to be right for this camera and would have become silently wrong the moment
   the model changed -- with no error, just a landing that drifts off-pad.

6. JSON with a monotonic timestamp and a sequence number, replacing
   "err_x,err_y,id" CSV. The consumer expected JSON. The producer sent CSV.
   Neither file revealed the mismatch because no document owned that boundary;
   docs/ARCHITECTURE.md now does.

7. No cv2.imshow in the transport callback. A GUI window pumped from a Gazebo
   subscriber thread blocks it for tens of milliseconds per frame. --preview is
   available for debugging and is off by default (finding F5).

8. The topic is a CLI argument. Sensor topics are model-scoped so each drone has
   its own, which is what lets a fleet work at all (finding F7).

Usage:
    python3 -m perception.vision_bridge --drone-id drone-0
    python3 -m perception.vision_bridge --drone-id drone-1 --preview
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import socket
import sys
import time

import cv2
import numpy as np

import config
import world.marker_models as marker_models
import world.pad_layout as pad_layout
from perception import pad_estimator

logger = logging.getLogger("vision_bridge")


class ArucoDetectorWrapper:
    """One ArUco detector, built once, reused for every frame.

    Deliberately configured with exactly one dictionary, DICT_4X4_50. Widening
    it would be actively harmful: the pad_0 and pad_2 textures also decode
    under DICT_ARUCO_MIP_36h12 (as IDs 102 and 116), so a multi-dictionary
    detector reports phantom markers that no leg is looking for and that
    select_target would have to filter out downstream.
    """

    def __init__(self, dictionary_name: str = marker_models.ARUCO_DICTIONARY):
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            raise ValueError(f"OpenCV has no ArUco dictionary {dictionary_name!r}")

        self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        params = cv2.aruco.DetectorParameters()
        # Corner refinement pays for itself here: the landing loop turns corner
        # positions into a metric offset via a homography, so sub-pixel corner
        # accuracy translates directly into centring accuracy at touchdown.
        #
        # Nothing else is tuned, and that is a measured decision rather than an
        # omission. A sweep of 132 synthetic frames spanning 0.5-10 m, two pad
        # offsets, three blur/noise levels and two pad rotations scores
        # identically -- 116/132 frames usable, 383 markers, 0 phantom ids --
        # under the defaults and under a tuned set (looser
        # minMarkerPerimeterRate, polygonalApproxAccuracyRate and
        # minMarkerDistanceRate for the closely-spaced nested markers). Adding
        # parameters that change nothing would only add ways to be wrong later.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, params)
        self.dictionary_name = dictionary_name

    def detect_raw(self, image: np.ndarray) -> list[tuple[int, np.ndarray]]:
        """(aruco_id, 4x2 corner array) per marker, in cv2's own TL/TR/BR/BL order.

        The estimator needs the corners as floats, not the rounded contract
        dicts: rounding a corner to 2 decimal places before fitting a homography
        throws away precisely the sub-pixel accuracy corner refinement just
        bought.
        """
        corners, ids, _rejected = self._detector.detectMarkers(image)
        if ids is None or len(ids) == 0:
            return []
        return [
            (int(marker_id), marker_corners[0])
            for marker_corners, marker_id in zip(corners, ids.flatten(), strict=True)
        ]

    def contract_dicts(
        self, raw: list[tuple[int, np.ndarray]], width: int, height: int
    ) -> list[dict]:
        """The per-marker contract view of an already-detected frame.

        Takes the raw detections rather than an image so a frame is run through
        the detector ONCE. The callback runs on the Gazebo transport thread and
        detection is the expensive part of it; detecting twice to produce two
        views of the same answer would halve the frame rate the landing loop
        depends on.
        """
        centre_x = width / 2.0
        centre_y = height / 2.0

        detections = []
        for marker_id, points in raw:
            marker_x = float(points[:, 0].mean())
            marker_y = float(points[:, 1].mean())

            # Mean side length: a robust apparent-size estimate that lets the
            # consumer sanity-check against config.MARKER_MIN_DECODE_PX.
            sides = [float(np.linalg.norm(points[i] - points[(i + 1) % 4])) for i in range(4)]

            detections.append(
                {
                    "id": marker_id,
                    "err_x": round(marker_x - centre_x, 2),
                    "err_y": round(marker_y - centre_y, 2),
                    "area_px": round(float(cv2.contourArea(points)), 1),
                    "size_px": round(sum(sides) / 4.0, 2),
                    "corners": [[round(float(x), 2), round(float(y), 2)] for x, y in points],
                }
            )

        return detections

    def detect(self, image: np.ndarray) -> list[dict]:
        """Detect and render the per-marker contract view in one call.

        Convenience for tests and for scripts/preflight.py; the flight path uses
        detect_raw + contract_dicts so it can share one detection with the pad
        estimator.
        """
        height, width = image.shape[:2]
        return self.contract_dicts(self.detect_raw(image), width, height)


class VisionBridge:
    """Subscribes to one drone's camera and broadcasts its detections."""

    def __init__(
        self, drone_id: str, topic: str, udp_host: str, udp_port: int, preview: bool = False
    ):
        self._drone_id = drone_id
        self._topic = topic
        self._addr = (udp_host, udp_port)
        self._preview = preview

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._detector = ArucoDetectorWrapper()
        # Built once. It is a pure function of the pad layout, and rebuilding it
        # per frame is the same mistake the detector construction used to be.
        self._marker_index = pad_layout.marker_index()

        self._seq = 0
        self._geometry_logged = False
        self._last_ids: tuple[int, ...] = ()

    def on_image(self, msg) -> None:
        """Gazebo transport callback. Must stay fast: it blocks the subscriber.

        Everything expensive or blocking has been removed from this path -- no
        detector construction, no GUI by default, no synchronous subprocess.
        A slow callback here starves the image queue and drops frames the
        landing loop needs (finding F5).
        """
        now = time.monotonic()

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        except ValueError as exc:
            # A reshape failure means the pixel format is not RGB8; detecting on
            # a misinterpreted buffer would produce confident nonsense.
            logger.error(
                "cannot interpret %dx%d frame as RGB8 (%s). Check the camera's "
                "<format> in sim/models/%s/model.sdf.",
                msg.width,
                msg.height,
                exc,
                config.GZ_MODEL_BASE,
            )
            return

        if not self._geometry_logged:
            self._geometry_logged = True
            self._log_geometry(msg.width, msg.height)

        image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        raw = self._detector.detect_raw(image)
        detections = self._detector.contract_dicts(raw, msg.width, msg.height)

        fx_px = (msg.width / 2.0) / math.tan(config.CAMERA_HFOV_RAD / 2.0)

        # Per-PAD estimates, which is what the landing loop actually steers on.
        # Every visible marker of a pad votes on where that pad's CENTRE is, so
        # one marker is enough and five are better -- and because each marker's
        # size is known, the offset comes out in metres without needing the
        # drone's altitude at all. See perception.pad_estimator.
        pads = [
            {
                "pad_id": obs.pad_index,
                "err_x": round(obs.err_px[0], 3),
                "err_y": round(obs.err_px[1], 3),
                "px_per_m": round(obs.px_per_m, 4),
                "range_m": round(obs.range_m, 4),
                "residual_m": round(obs.residual_m, 5),
                "markers": obs.marker_ids,
                "largest_side_px": round(obs.largest_side_px, 2),
                "clipped": not obs.corners_in_frame,
            }
            for obs in pad_estimator.estimate_pads(
                raw, int(msg.width), int(msg.height), fx_px, index=self._marker_index
            )
        ]

        self._seq += 1
        payload = {
            "t": now,
            "seq": self._seq,
            "w": int(msg.width),
            "h": int(msg.height),
            # Derived from the LIVE width, not from config, so a consumer that
            # trusts this field is correct even if the model and config drift.
            "fx": round(fx_px, 3),
            "drone_id": self._drone_id,
            "pads": pads,
            # The per-marker view is kept for debugging, for preflight and for
            # the legacy select_target() contract. The landing loop reads "pads".
            "detections": detections,
        }

        try:
            self._sock.sendto(json.dumps(payload).encode(), self._addr)
        except OSError as exc:
            logger.warning("UDP send failed: %s", exc)

        ids = tuple(sorted(d["id"] for d in detections))
        if ids != self._last_ids:
            if pads:
                logger.info(
                    "%s",
                    " | ".join(
                        f"pad {p['pad_id']}: {p['range_m']:.2f} m, "
                        f"offset ({p['err_x'] / p['px_per_m']:+.3f}, "
                        f"{p['err_y'] / p['px_per_m']:+.3f}) m, "
                        f"markers {p['markers']}"
                        + (f", spread {p['residual_m'] * 1000:.0f} mm" if p["residual_m"] else "")
                        for p in pads
                    ),
                )
            elif ids:
                # Markers decoded but none belongs to a known pad -- worth
                # saying out loud, because it usually means the world is running
                # stale pad models that predate world/pad_layout.py.
                logger.info("markers %s decoded but none maps to a pad", list(ids))
            else:
                logger.info("no markers in view")
            self._last_ids = ids

        if self._preview:
            self._show(image, detections)

    def _log_geometry(self, width: int, height: int) -> None:
        """Report the live geometry and flag a config mismatch loudly."""
        fx = (width / 2.0) / math.tan(config.CAMERA_HFOV_RAD / 2.0)
        logger.info(
            "first frame: %dx%d, fx=%.1f px, %.1f m pad spans %.0f px at %.0f m",
            width,
            height,
            fx,
            config.PAD_SIZE_M,
            fx * config.PAD_SIZE_M / config.SEARCH_ALT_M,
            config.SEARCH_ALT_M,
        )
        if width != config.CAMERA_WIDTH_PX or height != config.CAMERA_HEIGHT_PX:
            logger.error(
                "GEOMETRY MISMATCH: live %dx%d but config says %dx%d. "
                "config.CAMERA_FX_PX (%.1f) is what the landing loop uses to "
                "convert pixels to metres, so it is now wrong by a factor of "
                "%.2f. Fix config.py or the model, then re-run "
                "scripts/preflight.py.",
                width,
                height,
                config.CAMERA_WIDTH_PX,
                config.CAMERA_HEIGHT_PX,
                config.CAMERA_FX_PX,
                fx / config.CAMERA_FX_PX,
            )

    def _show(self, image: np.ndarray, detections: list[dict]) -> None:
        """Debug preview. Off by default -- see the F5 note in on_image."""
        height, width = image.shape[:2]
        cv2.line(image, (width // 2, 0), (width // 2, height), (255, 0, 0), 1)
        cv2.line(image, (0, height // 2), (width, height // 2), (255, 0, 0), 1)
        for det in detections:
            cx = int(width / 2 + det["err_x"])
            cy = int(height / 2 + det["err_y"])
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(
                image, f"id{det['id']}", (cx + 8, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1
            )
        # blocking-ok: opt-in debug preview only, never on the flight path.
        # Reached solely when --preview is passed; on_image() documents why.
        cv2.imshow(f"{self._drone_id} downward camera", image)
        cv2.waitKey(1)  # blocking-ok: paired with imshow above

    def close(self) -> None:
        self._sock.close()
        if self._preview:
            cv2.destroyAllWindows()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="ArUco detection bridge for one drone.")
    parser.add_argument("--drone-id", default="drone-0")
    parser.add_argument(
        "--topic",
        default=None,
        help="Gazebo image topic. Default: derived from --drone-id via sim_topics.camera_topic().",
    )
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument(
        "--udp-port",
        type=int,
        default=None,
        help="Default: config.vision_port(drone_id).",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show an OpenCV debug window. Costs frames -- do not use in flight.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Explicitly disable the preview (default behaviour).",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s [vision] %(message)s",
    )

    try:
        config.drone_index(args.drone_id)
    except ValueError as exc:
        print(f"vision_bridge: {exc}", file=sys.stderr)
        return 2

    import sim_topics

    topic = args.topic or sim_topics.camera_topic(args.drone_id)
    udp_port = args.udp_port if args.udp_port is not None else config.vision_port(args.drone_id)
    preview = args.preview and not args.headless

    # Imported here, not at module scope, so this file can be imported (and
    # ArucoDetectorWrapper unit-tested) without the Gazebo Python bindings.
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    bridge = VisionBridge(args.drone_id, topic, args.udp_host, udp_port, preview)
    node = Node()

    if not node.subscribe(Image, topic, bridge.on_image):
        print(f"vision_bridge: failed to subscribe to {topic}", file=sys.stderr)
        print(
            "  Is the drone spawned? Check with:\n"
            f"    gz topic -l | grep {args.drone_id.replace('-', '_')}\n"
            "  and run scripts/preflight.py.",
            file=sys.stderr,
        )
        return 1

    logger.info(
        "vision bridge up: %s | topic=%s | udp=%s:%d | dict=%s | preview=%s",
        args.drone_id,
        topic,
        args.udp_host,
        udp_port,
        bridge._detector.dictionary_name,
        preview,
    )

    try:
        while True:
            # blocking-ok: this is a synchronous CLI entry point, not an async
            # context. The Gazebo subscriber runs on its own thread; this loop
            # only keeps the process alive, so it sleeps rather than spinning.
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
