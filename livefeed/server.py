"""A small HTTP server that lets a browser watch a sensor process live.

The vision bridge and the avoider each run one, on a per-drone port from
config.camera_stream_port() / config.lidar_view_port(). The dashboard at
http://localhost:5000 embeds them directly:

    camera   GET /stream.mjpg   multipart JPEG, annotated with the detections
    lidar    GET /events        Server-Sent Events, one JSON scan per message
    both     GET /snapshot.json the latest value, for scripts and curl

WHY A SEPARATE SERVER, AND WHY THIS SHAPE
-----------------------------------------
These processes exist to make flight decisions. Watching them must not be able
to change one. So:

* The sensor callback only calls Latest.publish(), which stores a reference and
  notifies waiters. Rendering, JPEG encoding and JSON serialisation all happen
  on the HTTP server's own threads, and only while someone is watching. With no
  browser open, the cost is one lock and one assignment per frame.

* Every client gets its own daemon thread and only ever sees the NEWEST value.
  A slow browser skips frames; it never queues them, and it never blocks the
  callback.

* Nothing here raises into the caller. If the port is taken, start() logs a
  warning and returns False, and the bridge carries on flying without a viewer.
  A dashboard is never a reason to lose a sensor feed.

Standard library only: http.server is enough for a handful of local viewers,
and keeping it dependency-free means a sensor process can always import it.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

logger = logging.getLogger("livefeed")

KEEPALIVE_S = 1.0
"""How long a stream waits for a new value before telling the client that the
source has gone quiet. Short enough that the dashboard shows OFFLINE within a
second or two of the sensor stopping, which is the whole point of the view."""


class Latest:
    """The newest value from a sensor, plus a sequence number readers can wait on."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._seq = 0
        self._value: Any = None
        self._t = 0.0

    def publish(self, value: Any) -> None:
        """Replace the value. Cheap enough to call from a sensor callback."""
        with self._cond:
            self._value = value
            self._seq += 1
            self._t = time.monotonic()
            self._cond.notify_all()

    def get(self) -> tuple[int, Any, float]:
        """(seq, value, monotonic time of publish)."""
        with self._cond:
            return self._seq, self._value, self._t

    def wait_newer(self, seq: int, timeout_s: float) -> tuple[int, Any, float]:
        """Block until something newer than `seq` arrives, or the timeout.

        Returns the current state either way; the caller compares seq to tell
        a new value from a timeout.
        """
        with self._cond:
            self._cond.wait_for(lambda: self._seq != seq, timeout=timeout_s)
            return self._seq, self._value, self._t


Route = Callable[[BaseHTTPRequestHandler], None]


class LiveFeedServer:
    """Serve one sensor's live views. Routes are registered before start()."""

    def __init__(self, name: str, host: str, port: int) -> None:
        self.name = name
        self.host = host
        self.port = port
        self._routes: dict[str, Route] = {}
        self._httpd: ThreadingHTTPServer | None = None
        self._closing = threading.Event()

    # ── routes ──────────────────────────────────────────────────────────────

    def route_json(self, path: str, producer: Callable[[], Any]) -> None:
        """GET path -> producer() as JSON."""

        def handle(req: BaseHTTPRequestHandler) -> None:
            body = json.dumps(producer(), allow_nan=False).encode()
            req.send_response(200)
            _common_headers(req, "application/json")
            req.send_header("Content-Length", str(len(body)))
            req.end_headers()
            req.wfile.write(body)

        self._routes[path] = handle

    def route_sse(
        self,
        path: str,
        latest: Latest,
        render: Callable[[Any, float], Any],
    ) -> None:
        """GET path -> an event stream of render(value, age_s), one per publish.

        When the source goes quiet for KEEPALIVE_S, a {"stale": true} event is
        sent instead, so the browser can tell "nothing new" from "connection
        lost" -- the distinction the whole sensor contract is built around.
        """

        def handle(req: BaseHTTPRequestHandler) -> None:
            req.send_response(200)
            _common_headers(req, "text/event-stream")
            req.end_headers()

            last = -1
            while not self._closing.is_set():
                seq, value, published_at = latest.wait_newer(last, KEEPALIVE_S)
                age_s = time.monotonic() - published_at if published_at else None
                if seq == last or value is None:
                    event = {"stale": True, "age_s": None if age_s is None else round(age_s, 1)}
                else:
                    last = seq
                    event = render(value, age_s or 0.0)
                req.wfile.write(b"data: " + json.dumps(event, allow_nan=False).encode() + b"\n\n")
                req.wfile.flush()

        self._routes[path] = handle

    def route_mjpeg(
        self,
        path: str,
        latest: Latest,
        render_jpeg: Callable[[int, Any], bytes | None],
        idle_jpeg: Callable[[float | None], bytes | None],
    ) -> None:
        """GET path -> multipart JPEG. render_jpeg(seq, value) per new value;
        idle_jpeg(age_s) whenever the source is quiet, so the viewer shows a
        clear NO SIGNAL card rather than a frozen last frame."""

        def handle(req: BaseHTTPRequestHandler) -> None:
            req.send_response(200)
            _common_headers(req, "multipart/x-mixed-replace; boundary=frame")
            req.end_headers()

            last = -1
            while not self._closing.is_set():
                seq, value, published_at = latest.wait_newer(last, KEEPALIVE_S)
                if seq == last or value is None:
                    age_s = time.monotonic() - published_at if published_at else None
                    jpeg = idle_jpeg(age_s)
                else:
                    last = seq
                    jpeg = render_jpeg(seq, value)
                if not jpeg:
                    continue
                req.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
                req.wfile.flush()

        self._routes[path] = handle

    # ── lifecycle ───────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Bind and serve on a daemon thread. Never raises; False on failure."""
        routes = self._routes
        name = self.name

        class Handler(BaseHTTPRequestHandler):
            server_version = "livefeed/1"

            def do_GET(self) -> None:  # noqa: N802 - http.server's naming
                route = routes.get(self.path.split("?", 1)[0])
                if route is None:
                    body = json.dumps({"feed": name, "routes": sorted(routes)}).encode()
                    self.send_response(404 if self.path not in ("/", "") else 200)
                    _common_headers(self, "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                try:
                    route(self)
                except (BrokenPipeError, ConnectionResetError, TimeoutError):
                    pass  # the viewer closed the tab; nothing to report
                except Exception:  # noqa: BLE001 - a viewer must never take the process down
                    logger.exception("%s: error serving %s", name, self.path)

            def log_message(self, *_args) -> None:
                pass  # one line per MJPEG request would bury the sensor's own log

        try:
            self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:
            logger.warning(
                "%s: live view unavailable, cannot bind %s:%d (%s). The sensor "
                "itself is unaffected.",
                self.name,
                self.host,
                self.port,
                exc,
            )
            return False

        self._httpd.daemon_threads = True
        thread = threading.Thread(
            target=self._httpd.serve_forever, name=f"livefeed-{self.name}", daemon=True
        )
        thread.start()
        logger.info("%s: live view on http://%s:%d/", self.name, self.host, self.port)
        return True

    @property
    def bound_port(self) -> int | None:
        """The port actually bound -- differs from `port` when that was 0."""
        return self._httpd.server_address[1] if self._httpd is not None else None

    def close(self) -> None:
        self._closing.set()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None


def _common_headers(req: BaseHTTPRequestHandler, content_type: str) -> None:
    req.send_header("Content-Type", content_type)
    req.send_header("Cache-Control", "no-cache, no-store")
    # The dashboard is served from port 5000 and reads these from another port.
    req.send_header("Access-Control-Allow-Origin", "*")
