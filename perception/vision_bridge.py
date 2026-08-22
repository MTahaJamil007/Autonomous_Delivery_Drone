#!/usr/bin/env python3
"""Downward-camera ArUco detector: Gazebo images in, JSON detections out over UDP.

Output contract (one datagram per frame, see docs/ARCHITECTURE.md):

    {"t": <monotonic s>, "seq": <int>, "w": 320, "h": 240, "fx": 277.2,
     "drone_id": "drone-0",
     "detections": [
        {"id": 0, "err_x": -12, "err_y": 5, "area_px": 3021,
         "size_px": 55.0, "corners": [[x,y], ...]}
     ]}

A datagram is sent for EVERY frame, including frames with no detections. That is
the only way a consumer can tell "the pad is not in view" from "the bridge is
dead" -- the previous protocol sent the literal string "None" for no marker and
nothing at all when the process died, which are indistinguishable to a reader
that is not tracking time.

WHAT CHANGED AND WHY
--------------------
1. ALL detections are emitted. The old code took corners[0][0] and ids[0][0] --
   the first marker only. Marker disambiguation is the documented behaviour and
   landing.select_target() exists to do it, but it can only choose between
   markers the producer bothered to send. With three pads within metres of each
   other, discarding every marker but the first is what made "land on ID 1"
   unimplementable.

2. The dictionary and detector are built ONCE. They were reconstructed inside
   the per-frame callback, at 10 Hz, forever.

3. The image centre comes from msg.width/msg.height. Hardcoded 160/120 happened
   to be right for this camera and would have become silently wrong the moment
   the model changed -- with no error, just a landing that drifts off-pad.

4. JSON with a monotonic timestamp and a sequence number, replacing
   "err_x,err_y,id" CSV. The consumer expected JSON. The producer sent CSV.
   Neither file revealed the mismatch because no document owned that boundary;
   docs/ARCHITECTURE.md now does.

5. No cv2.imshow in the transport callback. A GUI window pumped from a Gazebo
   subscriber thread blocks it for tens of milliseconds per frame. --preview is
   available for debugging and is off by default (finding F5).

6. The topic is a CLI argument. Sensor topics are model-scoped so each drone has
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
        # Corner refinement pays for itself here: the landing loop converts pixel
        # error to a metric offset, so sub-pixel corner accuracy translates
        # directly into centring accuracy at touchdown.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, params)
        self.dictionary_name = dictionary_name

    def detect(self, image: np.ndarray) -> list[dict]:
        """Every marker in the frame, as contract dicts.

        err_x/err_y are pixel offsets of the marker centre from the image
        centre: +x right, +y down, matching image convention. The consumer
        converts them to metres; see docs/CALIBRATION.md for the sign mapping
        into the body frame.
        """
        height, width = image.shape[:2]
        centre_x = width / 2.0
        centre_y = height / 2.0

        corners, ids, _rejected = self._detector.detectMarkers(image)
        if ids is None or len(ids) == 0:
            return []

        detections = []
        # strict=True: OpenCV returns one corner set per id, so a mismatch is a
        # broken detector result, not something to iterate past.
        for marker_corners, marker_id in zip(corners, ids.flatten(), strict=True):
            points = marker_corners[0]  # 4x2 float32
            marker_x = float(points[:, 0].mean())
            marker_y = float(points[:, 1].mean())

            # Mean side length: a robust apparent-size estimate that lets the
            # consumer sanity-check against config.MARKER_MIN_DECODE_PX.
            sides = [float(np.linalg.norm(points[i] - points[(i + 1) % 4])) for i in range(4)]

            detections.append(
                {
                    "id": int(marker_id),
                    "err_x": round(marker_x - centre_x, 2),
                    "err_y": round(marker_y - centre_y, 2),
                    "area_px": round(float(cv2.contourArea(points)), 1),
                    "size_px": round(sum(sides) / 4.0, 2),
                    "corners": [[round(float(x), 2), round(float(y), 2)] for x, y in points],
                }
            )

        return detections


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
        detections = self._detector.detect(image)

        self._seq += 1
        payload = {
            "t": now,
            "seq": self._seq,
            "w": int(msg.width),
            "h": int(msg.height),
            # Derived from the LIVE width, not from config, so a consumer that
            # trusts this field is correct even if the model and config drift.
            "fx": round((msg.width / 2.0) / math.tan(config.CAMERA_HFOV_RAD / 2.0), 3),
            "drone_id": self._drone_id,
            "detections": detections,
        }

        try:
            self._sock.sendto(json.dumps(payload).encode(), self._addr)
        except OSError as exc:
            logger.warning("UDP send failed: %s", exc)

        ids = tuple(sorted(d["id"] for d in detections))
        if ids != self._last_ids:
            if ids:
                sizes = ", ".join(f"id{d['id']}={d['size_px']:.0f}px" for d in detections)
                too_small = [
                    d["id"] for d in detections if d["size_px"] < config.MARKER_MIN_DECODE_PX
                ]
                note = f" (marginal size: {too_small})" if too_small else ""
                logger.info("markers %s%s", sizes, note)
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
