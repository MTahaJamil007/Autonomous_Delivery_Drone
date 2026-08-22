"""
Obstacle Memory Client.

Provides functions for drone agents to interact with the obstacle memory service.
Per specification: obstacles are fetched ONCE at mission start and cached.
"""

import logging
from typing import Any

import aiohttp

import config

logger = logging.getLogger(__name__)


async def fetch_known_obstacles_for_mission(
    bbox: tuple[float, float, float, float],
) -> list[dict[str, Any]]:
    """
    Fetch known obstacles for a mission's bounding box.

    Called exactly ONCE, before Leg 1 of a mission begins. The result is
    cached for the entire mission — per specification, obstacle memory does
    NOT poll mid-mission or subscribe live.

    Args:
        bbox: (min_lat, max_lat, min_lon, max_lon) bounding box

    Returns:
        List of obstacle dicts from the memory service
    """
    min_lat, max_lat, min_lon, max_lon = bbox

    url = f"{config.OBSTACLE_MEMORY_URL}/obstacles"
    params = {
        "min_lat": min_lat,
        "max_lat": max_lat,
        "min_lon": min_lon,
        "max_lon": max_lon,
        "min_confidence": 0.15,
    }

    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(
                url, params=params, timeout=aiohttp.ClientTimeout(total=5.0)
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    obstacles = data.get("obstacles", [])
                    logger.info(
                        f"Fetched {len(obstacles)} known obstacles from memory service "
                        f"for bbox ({min_lat:.4f},{min_lon:.4f})-({max_lat:.4f},{max_lon:.4f})"
                    )
                    return obstacles
                else:
                    logger.error(f"Obstacle service returned status {response.status}")
                    return []

    except aiohttp.ClientError as e:
        logger.error(f"Failed to fetch obstacles from memory service: {e}")
        return []

    except Exception as e:
        logger.error(f"Unexpected error fetching obstacles: {e}", exc_info=True)
        return []


async def report_obstacle(
    lat: float,
    lon: float,
    source_drone: str,
    radius_m: float = 3.0,
    obstacle_type: str = "static_wall",
    confidence: float = 0.5,
) -> bool:
    """
    Report a newly discovered obstacle to the memory service.

    Called by avoider_node on first DODGE_* lock for a not-yet-known wall.

    Args:
        lat, lon: Obstacle coordinates
        source_drone: Reporting drone ID
        radius_m: Obstacle radius
        obstacle_type: Classification
        confidence: Initial confidence

    Returns:
        True if report succeeded, False otherwise
    """
    url = f"{config.OBSTACLE_MEMORY_URL}/obstacles"
    payload = {
        "lat": lat,
        "lon": lon,
        "radius_m": radius_m,
        "obstacle_type": obstacle_type,
        "source_drone": source_drone,
        "confidence": confidence,
    }

    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(
                url, json=payload, timeout=aiohttp.ClientTimeout(total=5.0)
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    obstacle_id = data.get("obstacle_id")
                    logger.info(
                        f"[{source_drone}] Reported obstacle {obstacle_id} at "
                        f"({lat:.6f}, {lon:.6f})"
                    )
                    return True
                else:
                    logger.error(f"Failed to report obstacle: status {response.status}")
                    return False

    except aiohttp.ClientError as e:
        logger.error(f"Failed to report obstacle to memory service: {e}")
        return False

    except Exception as e:
        logger.error(f"Unexpected error reporting obstacle: {e}", exc_info=True)
        return False
