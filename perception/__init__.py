"""Perception: sensor consumers that turn raw simulator data into contracts.

The one module here, `vision_bridge`, subscribes to a drone's downward camera
over Gazebo transport, detects ArUco markers, and republishes them as JSON over
UDP. Its output schema is documented in docs/ARCHITECTURE.md; change the schema
and that table in the same commit.
"""
