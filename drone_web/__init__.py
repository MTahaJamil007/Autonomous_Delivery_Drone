"""Web-facing per-drone orchestration.

`drone_logic` is the boundary between the dispatcher and the mission core: it
declares the contract the dispatcher calls and delegates the flying to
drone_agent.
"""
