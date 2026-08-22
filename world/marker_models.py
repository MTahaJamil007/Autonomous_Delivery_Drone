"""Landing-pad model mapping: pad role <-> Gazebo model <-> ArUco marker ID.

WHY EACH PAD GETS ITS OWN MARKER
--------------------------------
The documented behaviour is precision landing on ArUco IDs 0, 1 and 2 with
disambiguation. Before remediation, drone_logic.py spawned `model://arucotag`
for all three pads and then asked the detector for marker 0 on every leg, so
three identical tags stood within metres of each other and the drone had no way
to tell the pickup pad from the drop pad from home. Distinct IDs are what make
`landing.select_target(detections, expected_id)` mean anything.

WHY NOT `model://arucotag` (finding F1)
--------------------------------------
That model's 354x354 texture fills its plane edge to edge with no white quiet
zone. ArUco's contour stage cannot segment a marker without one: the texture
decodes in ZERO of OpenCV's 27 predefined dictionaries as rendered, and only
becomes readable after adding a white border. So `vision_data["locked"]` could
never become True, landing always fell through to the search branch, timed out,
climbed back to cruise altitude, and the mission continued as though nothing
had happened. That single asset is the root cause of precision landing never
working, independent of every other bug.

The models below are DroneProgram's own vendored pads (sim/models/pad_N), which
carry PX4's decodable arucotag_N textures on a 2.0 m plane -- 2.0 m rather than
0.5 m because a 0.5 m marker spans only 14 px at cruise altitude and cannot be
decoded at all (finding F3). scripts/preflight.py asserts each texture really
decodes to its expected ID, so a regression here fails on the bench instead of
in the air.
"""

PAD_MODELS = {
    "pickup_pad": "model://pad_0",
    "drop_pad": "model://pad_1",
    "home_pad": "model://pad_2",
}
"""Pad role -> Gazebo model URI. Resolved via GZ_SIM_RESOURCE_PATH; source
sim/env.sh so DroneProgram's sim/models is on that path."""

ROLE_TO_ID = {
    "pickup_pad": 0,
    "drop_pad": 1,
    "home_pad": 2,
}
"""Pad role -> the DICT_4X4_50 marker ID printed on it."""

ID_TO_ROLE = {marker_id: role for role, marker_id in ROLE_TO_ID.items()}

ARUCO_DICTIONARY = "DICT_4X4_50"
"""The one dictionary the whole system uses. The vision bridge builds exactly
this; do not widen it. arucotag_0 and arucotag_2 also happen to decode under
DICT_ARUCO_MIP_36h12 (as IDs 102 and 116), so a detector configured with
multiple dictionaries would report phantom markers."""

LEG_ORDER = ("pickup_pad", "drop_pad", "home_pad")
"""Canonical 3-leg delivery sequence. drone_logic builds its legs from this
rather than from three hardcoded zeroes."""


def marker_id_for_role(role: str) -> int:
    """ArUco ID for a pad role.

    Raises:
        KeyError: on an unknown role, rather than silently defaulting to 0 --
            which is how all three legs ended up hunting for marker 0.
    """
    return ROLE_TO_ID[role]


def model_uri_for_role(role: str) -> str:
    """Gazebo model URI for a pad role."""
    return PAD_MODELS[role]
