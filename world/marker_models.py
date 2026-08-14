"""
Marker model mapping for ArUco pads.
Maps pad roles to distinct ArUco marker models to prevent collision/flickering.
"""

# Mapping: pad role → Gazebo model name
PAD_MODELS = {
    "pickup_pad": "model://arucotag_0",   # ArUco ID 0
    "drop_pad": "model://arucotag_1",      # ArUco ID 1
    "home_pad": "model://arucotag_2",      # ArUco ID 2
}

# Reverse mapping: ArUco ID → pad role (for disambiguation)
ID_TO_ROLE = {
    0: "pickup_pad",
    1: "drop_pad",
    2: "home_pad",
}
