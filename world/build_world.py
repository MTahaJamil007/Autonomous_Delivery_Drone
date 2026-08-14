"""
Lightweight real-world SDF generator.

Pulls building footprints from OpenStreetMap and generates a simplified
Gazebo world with flat-color low-poly boxes for buildings.
"""

import argparse
import logging
from typing import List, Tuple, Dict
from pathlib import Path

try:
    import requests
except ImportError:
    requests = None
    logging.warning("requests not installed - OSM fetching disabled")

logger = logging.getLogger(__name__)


def fetch_osm_buildings(
    min_lat: float,
    max_lat: float,
    min_lon: float,
    max_lon: float
) -> List[Dict]:
    """
    Fetch building footprints from OpenStreetMap Overpass API.
    
    Args:
        min_lat, max_lat: Latitude bounds
        min_lon, max_lon: Longitude bounds
        
    Returns:
        List of building dicts with 'nodes' (lat/lon coordinates)
    """
    if requests is None:
        logger.error("requests library not available")
        return []
    
    overpass_url = "http://overpass-api.de/api/interpreter"
    
    # Overpass query for buildings
    query = f"""
    [out:json];
    (
      way["building"]({min_lat},{min_lon},{max_lat},{max_lon});
    );
    out geom;
    """
    
    try:
        response = requests.post(overpass_url, data={"data": query}, timeout=30)
        response.raise_for_status()
        data = response.json()
        
        buildings = []
        for element in data.get("elements", []):
            if element.get("type") == "way" and "geometry" in element:
                nodes = [
                    (node["lat"], node["lon"])
                    for node in element["geometry"]
                ]
                buildings.append({
                    "id": element["id"],
                    "nodes": nodes,
                    "tags": element.get("tags", {})
                })
        
        logger.info(f"Fetched {len(buildings)} buildings from OSM")
        return buildings
    
    except Exception as e:
        logger.error(f"Failed to fetch OSM data: {e}")
        return []


def lat_lon_to_meters(
    lat: float,
    lon: float,
    ref_lat: float,
    ref_lon: float
) -> Tuple[float, float]:
    """
    Convert GPS to local meters (flat-earth approximation).
    
    Args:
        lat, lon: Target coordinates
        ref_lat, ref_lon: Reference (origin) coordinates
        
    Returns:
        (x_east, y_north) in meters
    """
    import math
    d_lat = lat - ref_lat
    d_lon = lon - ref_lon
    y_north = d_lat * 111_320.0
    x_east = d_lon * (111_320.0 * math.cos(math.radians(ref_lat)))
    return x_east, y_north


def building_to_box(
    building: Dict,
    ref_lat: float,
    ref_lon: float,
    default_height: float = 10.0
) -> Dict:
    """
    Convert building footprint to a simplified box model.
    
    Args:
        building: Building dict with 'nodes' list
        ref_lat, ref_lon: Reference coordinates for meter conversion
        default_height: Default building height in meters
        
    Returns:
        Dict with 'x', 'y', 'width', 'length', 'height', 'rotation'
    """
    nodes = building["nodes"]
    if len(nodes) < 3:
        return None
    
    # Convert to meters
    points = [lat_lon_to_meters(lat, lon, ref_lat, ref_lon) for lat, lon in nodes]
    
    # Compute bounding box
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    # Center and dimensions
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    width = max_x - min_x
    length = max_y - min_y
    
    # Extract height from tags if available
    height = default_height
    tags = building.get("tags", {})
    if "height" in tags:
        try:
            height = float(tags["height"])
        except ValueError:
            pass
    elif "building:levels" in tags:
        try:
            levels = int(tags["building:levels"])
            height = levels * 3.0  # Assume 3m per level
        except ValueError:
            pass
    
    return {
        "x": center_x,
        "y": center_y,
        "width": width,
        "length": length,
        "height": height,
        "rotation": 0.0  # Simplified - no rotation
    }


def generate_sdf(
    buildings: List[Dict],
    ref_lat: float,
    ref_lon: float,
    output_path: str = "lightweight_realworld.sdf"
) -> None:
    """
    Generate Gazebo SDF world file with simplified building models.
    
    Args:
        buildings: List of building dicts
        ref_lat, ref_lon: Reference coordinates
        output_path: Output SDF file path
    """
    sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="realworld_delivery">
    
    <!-- Physics -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Sun -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1000 1000</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>1000 1000</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.4 0.5 0.4 1</ambient>
            <diffuse>0.4 0.5 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
"""
    
    # Generate building models
    building_models = []
    for i, building_dict in enumerate(buildings):
        box = building_to_box(building_dict, ref_lat, ref_lon)
        if box is None or box["width"] < 1.0 or box["length"] < 1.0:
            continue  # Skip invalid/tiny buildings
        
        model = f"""
    <model name="building_{i}">
      <static>true</static>
      <pose>{box['x']} {box['y']} {box['height']/2} 0 0 {box['rotation']}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{box['width']} {box['length']} {box['height']}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{box['width']} {box['length']} {box['height']}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
        building_models.append(model)
    
    sdf_footer = """
  </world>
</sdf>
"""
    
    # Write SDF file
    with open(output_path, 'w') as f:
        f.write(sdf_header)
        f.write("\n".join(building_models))
        f.write(sdf_footer)
    
    logger.info(f"Generated SDF with {len(building_models)} buildings: {output_path}")


def main():
    """Command-line interface for world generation."""
    parser = argparse.ArgumentParser(description="Generate lightweight Gazebo world from OSM")
    parser.add_argument("--min-lat", type=float, required=True, help="Minimum latitude")
    parser.add_argument("--max-lat", type=float, required=True, help="Maximum latitude")
    parser.add_argument("--min-lon", type=float, required=True, help="Minimum longitude")
    parser.add_argument("--max-lon", type=float, required=True, help="Maximum longitude")
    parser.add_argument("--ref-lat", type=float, help="Reference latitude (default: min-lat)")
    parser.add_argument("--ref-lon", type=float, help="Reference longitude (default: min-lon)")
    parser.add_argument("--output", type=str, default="lightweight_realworld.sdf", help="Output SDF file")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO)
    
    ref_lat = args.ref_lat if args.ref_lat else args.min_lat
    ref_lon = args.ref_lon if args.ref_lon else args.min_lon
    
    buildings = fetch_osm_buildings(args.min_lat, args.max_lat, args.min_lon, args.max_lon)
    
    if buildings:
        generate_sdf(buildings, ref_lat, ref_lon, args.output)
        print(f"✅ World generated: {args.output}")
    else:
        print("❌ No buildings fetched - world not generated")


if __name__ == "__main__":
    main()
