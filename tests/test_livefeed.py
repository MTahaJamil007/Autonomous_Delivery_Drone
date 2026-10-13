"""The dashboard's live sensor views: server, camera overlay, dispatcher payload.

The property that matters most is negative: watching a sensor must never be
able to affect it. The server never raises into its owner (a taken port is a
warning, not a crash), and the camera overlay never mutates the frame the
detector is working from.
"""

from __future__ import annotations

import json
import socket
import threading
import time
import urllib.request

import numpy as np
import pytest

import config
from livefeed.server import Latest, LiveFeedServer
from perception import camera_overlay

# ── Latest ──────────────────────────────────────────────────────────────────


def test_latest_wakes_a_waiter_on_publish():
    latest = Latest()
    got = {}

    def reader():
        got["result"] = latest.wait_newer(0, timeout_s=5.0)

    thread = threading.Thread(target=reader)
    thread.start()
    time.sleep(0.05)
    latest.publish("frame")
    thread.join(2.0)
    assert got["result"][0] == 1
    assert got["result"][1] == "frame"


def test_latest_times_out_without_news():
    latest = Latest()
    latest.publish("old")
    started = time.monotonic()
    seq, value, _t = latest.wait_newer(1, timeout_s=0.1)
    assert (seq, value) == (1, "old")
    assert time.monotonic() - started < 1.0


# ── LiveFeedServer ──────────────────────────────────────────────────────────


@pytest.fixture
def server():
    srv = LiveFeedServer("test", "127.0.0.1", 0)
    yield srv
    srv.close()


def _get(port, path, timeout=3.0):
    return urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout)


def test_json_route_with_cors(server):
    server.route_json("/snapshot.json", lambda: {"ok": True})
    assert server.start()
    with _get(server.bound_port, "/snapshot.json") as response:
        assert response.headers["Access-Control-Allow-Origin"] == "*"
        assert json.loads(response.read()) == {"ok": True}


def test_sse_streams_values_then_reports_stale(server):
    latest = Latest()
    server.route_sse("/events", latest, lambda value, age: {"v": value})
    assert server.start()
    latest.publish(42)
    with _get(server.bound_port, "/events") as response:
        assert response.headers["Content-Type"] == "text/event-stream"
        first = response.readline()
        assert json.loads(first[len(b"data: ") :]) == {"v": 42}
        response.readline()  # blank separator
        # Nothing new published: within KEEPALIVE_S the stream says so.
        stale = json.loads(response.readline()[len(b"data: ") :])
        assert stale["stale"] is True


def test_mjpeg_serves_frames_and_an_idle_card(server):
    latest = Latest()
    server.route_mjpeg(
        "/stream.mjpg", latest, lambda seq, value: b"JPEG" + value, lambda age: b"IDLE"
    )
    assert server.start()
    latest.publish(b"1")
    with _get(server.bound_port, "/stream.mjpg") as response:
        assert "multipart/x-mixed-replace" in response.headers["Content-Type"]
        body = b""
        while b"IDLE" not in body:
            body += response.read1(4096)
        assert b"JPEG1" in body


def test_a_taken_port_is_a_warning_not_a_crash():
    holder = socket.socket()
    holder.bind(("127.0.0.1", 0))
    holder.listen()
    try:
        srv = LiveFeedServer("test", "127.0.0.1", holder.getsockname()[1])
        assert srv.start() is False
        srv.close()
    finally:
        holder.close()


def test_view_ports_are_per_drone_and_collide_with_nothing():
    ports = []
    for index in range(config.MAX_FLEET_SIZE):
        drone = f"drone-{index}"
        ports += [
            config.camera_stream_port(drone),
            config.lidar_view_port(drone),
            config.vision_port(drone),
            config.lidar_port(drone),
            config.mavsdk_port(drone),
        ]
    ports += [config.FLEET_DISPATCH_PORT, config.OBSTACLE_MEMORY_PORT]
    assert len(ports) == len(set(ports))


# ── camera overlay ──────────────────────────────────────────────────────────


def _synthetic_pad_frame():
    """A real pad texture in a 320x240 frame, through the real detector."""
    import cv2

    import world.pad_layout as pad_layout
    from perception import pad_estimator
    from perception.vision_bridge import ArucoDetectorWrapper

    texture = np.asarray(pad_layout.render_pad_texture(0))
    if texture.ndim == 2:
        texture = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR)
    side = int(config.CAMERA_FX_PX * pad_layout.PAD_SIZE_M / 4.0)
    image = np.full((240, 320, 3), 160, np.uint8)
    pad = cv2.resize(texture, (side, side), interpolation=cv2.INTER_AREA)
    image[120 - side // 2 : 120 - side // 2 + side, 170 - side // 2 : 170 - side // 2 + side] = pad

    raw = ArucoDetectorWrapper().detect_raw(image)
    pads = [
        {
            "pad_id": obs.pad_index,
            "err_x": obs.err_px[0],
            "err_y": obs.err_px[1],
            "px_per_m": obs.px_per_m,
            "range_m": obs.range_m,
            "residual_m": obs.residual_m,
            "markers": obs.marker_ids,
            "largest_side_px": obs.largest_side_px,
            "clipped": not obs.corners_in_frame,
        }
        for obs in pad_estimator.estimate_pads(
            raw, 320, 240, config.CAMERA_FX_PX, index=pad_layout.marker_index()
        )
    ]
    return image, raw, pads


def test_overlay_annotates_without_touching_the_detector_frame():
    image, raw, pads = _synthetic_pad_frame()
    assert raw and pads, "fixture must actually decode a pad"
    before = image.copy()

    out = camera_overlay.render_frame(image, raw, pads, "drone-0", 10.0, 1)

    assert np.array_equal(image, before), "the overlay drew on the detector's frame"
    assert out.shape == (480, camera_overlay.OUTPUT_WIDTH_PX, 3)
    jpeg = camera_overlay.encode_jpeg(out)
    assert jpeg and jpeg[:2] == b"\xff\xd8"


def test_overlay_handles_an_empty_frame_and_no_signal():
    blank = np.zeros((240, 320, 3), np.uint8)
    assert camera_overlay.render_frame(blank, [], [], "drone-0", 0.0, 0).shape[0] == 480
    assert camera_overlay.encode_jpeg(camera_overlay.no_signal_frame("drone-0", None))
    assert camera_overlay.encode_jpeg(camera_overlay.no_signal_frame("drone-0", 12.3))


@pytest.mark.parametrize(("marker", "pad"), [(0, 0), (2, 2), (10, 0), (13, 0), (21, 1), (33, 2)])
def test_marker_to_pad_mapping(marker, pad):
    assert camera_overlay.marker_pad(marker) == pad


# ── dispatcher endpoints the dashboard depends on ───────────────────────────


@pytest.fixture
def client(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient

    from fleet_dispatch import app as app_module
    from fleet_dispatch import db as fleet_db

    monkeypatch.setattr(fleet_db, "DB_PATH", tmp_path / "fleet.db")
    monkeypatch.setattr(config, "DRONE_IDS", ["drone-0"])
    with TestClient(app_module.app) as test_client:
        yield test_client


def test_fleet_live_matches_the_socket_payload(client):
    body = client.get("/fleet/live").json()
    assert body["drones"][0]["id"] == "drone-0"
    assert body["live"]["drone-0"]["live"] is False
    with client.websocket_connect("/ws") as ws:
        assert ws.receive_json().keys() == body.keys()


def test_page_carries_the_stream_ports(client):
    html = client.get("/").text
    assert str(config.camera_stream_port("drone-0")) in html
    assert str(config.lidar_view_port("drone-0")) in html


def test_map_obstacles_degrades_instead_of_failing(client, monkeypatch):
    monkeypatch.setattr(config, "OBSTACLE_MEMORY_PORT", 1)  # nothing listens there
    body = client.get("/map/obstacles").json()
    assert body["obstacles"] == []
    assert "error" in body
