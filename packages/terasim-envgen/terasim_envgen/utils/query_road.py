"""
query_road.py - Module for querying road information using OSMnx
"""


import os
import logging
import yaml
import random
import time
from pathlib import Path
import osmnx as ox
import networkx as nx
from shapely.geometry import Point, box

logger = logging.getLogger(__name__)


def distance_between_points(lat1, lon1, lat2, lon2):
    """
    Calculate the distance between two geographical points.

    Args:
        lat1 (float): Latitude of first point
        lon1 (float): Longitude of first point
        lat2 (float): Latitude of second point
        lon2 (float): Longitude of second point

    Returns:
        float: Distance in meters
    """
    return ox.distance.great_circle_vec(lat1, lon1, lat2, lon2)


def find_nearest_edge(G, lat, lon, max_dist=100):
    """
    Find the nearest edge to a given point within a specified distance.

    Args:
        G (networkx.Graph): NetworkX graph
        lat (float): Latitude of the point
        lon (float): Longitude of the point
        max_dist (float): Maximum distance in meters

    Returns:
        tuple: (u, v, data) of the nearest edge, or None if not found
    """
    try:
        # Find the nearest edge
        edge = ox.nearest_edges(G, lon, lat, return_dist=True)
        if edge[1] <= max_dist:
            return edge[0]
        return None
    except Exception as e:
        logger.error(f"Error finding nearest edge: {e}")
        return None


def query_road_info(lat, lon, dist=500):
    """
    Query road information at specific coordinates.

    Args:
        lat (float): Latitude
        lon (float): Longitude
        dist (float): Distance in meters around the point

    Returns:
        dict: Dictionary containing road information
    """
    try:
        # Download the network around the point
        G = ox.graph_from_point((lat, lon), dist=dist, network_type="drive")

        # Find the nearest edge
        nearest_edge = find_nearest_edge(G, lat, lon)
        if not nearest_edge:
            return None

        u, v, data = nearest_edge

        # Get information about the road
        info = {
            "highway_type": data.get("highway", "unknown"),
            "name": data.get("name", "unnamed"),
            "lanes": data.get("lanes", "unknown"),
            "max_speed": data.get("maxspeed", "unknown"),
            "length": data.get("length", 0),
            "geometry": data.get("geometry", None),
        }

        # Get connected edges
        connected_edges = list(G.edges(u, data=True))
        info["num_connected_roads"] = len(connected_edges)

        # Count road types
        road_types = {}
        for _, _, edge_data in connected_edges:
            road_type = edge_data.get("highway", "unknown")
            road_types[road_type] = road_types.get(road_type, 0) + 1
        info["connected_road_types"] = road_types

        return info

    except Exception as e:
        logger.error(f"Error querying road info: {e}")
        return None


def find_roads_by_tag(city, highway_type="motorway_link", road_name=None, junction=None):
    """
    Find all roads in a city that match a given highway type, road name, or junction type.

    Args:
        city (str): City name to search in
        highway_type (str): Type of highway to search for
        road_name (str, optional): Road name to search for
        junction (str, optional): Junction type to search for (e.g., "roundabout")

    Returns:
        list: List of matching road segments
    """
    # Convert highway_type to list if it's a string
    if isinstance(highway_type, str):
        highway_type = [highway_type]
    elif isinstance(highway_type, list):
        pass
    else:
        logger.error(f"Invalid highway_type: {highway_type}")
        
    # Download the street network
    logger.info(f"Downloading street network for {city}")
    G = ox.graph_from_place(city, network_type="drive")
    logger.info(
        f"Downloaded network with {len(G.nodes)} nodes and {len(G.edges)} edges"
    )

    # Find matching roads
    matching_roads = []
    for u, v, data in G.edges(data=True):
        # Match by junction type if specified
        if junction and data.get("junction") == junction:
            road_info = {
                "from": (G.nodes[u]["y"], G.nodes[u]["x"]),
                "to": (G.nodes[v]["y"], G.nodes[v]["x"]),
                "from_node": u,
                "to_node": v,
                "name": data.get("name", "unnamed"),
                "highway_type": data.get("highway", "unknown"),
                "junction_type": data.get("junction", "unknown"),
                "lanes": data.get("lanes", "unknown"),
                "max_speed": data.get("maxspeed", "unknown"),
                "length": data.get("length", 0),
            }
            matching_roads.append(road_info)
        # Otherwise match by highway type and road name
        elif not junction:
            if data.get("highway") in highway_type:
                if road_name is None or data.get("name") == road_name:
                    road_info = {
                        "from": (G.nodes[u]["y"], G.nodes[u]["x"]),
                        "to": (G.nodes[v]["y"], G.nodes[v]["x"]),
                        "from_node": u,
                        "to_node": v,
                        "name": data.get("name", "unnamed"),
                        "highway_type": data.get("highway", "unknown"),
                        "lanes": data.get("lanes", "unknown"),
                        "max_speed": data.get("maxspeed", "unknown"),
                        "length": data.get("length", 0),
                    }
                    matching_roads.append(road_info)

    logger.info(f"Found {len(matching_roads)} matching roads")
    return matching_roads


def get_random_road_networks(
    city,
    highway_type="motorway_link",
    road_name=None,
    num_samples=3,
    radius=500,
    random_seed=None,
    junction=None,
):
    """
    Randomly select road segments and get their surrounding networks.

    Args:
        city (str): City name to search in
        highway_type (str): Type of highway to search for
        road_name (str, optional): Road name to search for
        num_samples (int): Number of random samples to get
        radius (float): Radius in meters around each point
        random_seed (int, optional): Random seed for reproducibility
        junction (str, optional): Junction type to search for (e.g., "roundabout")

    Returns:
        list: List of network information dictionaries
    """
    try:
        # Set random seed if provided
        if random_seed is not None:
            random.seed(random_seed)

        # Find matching roads
        roads = find_roads_by_tag(city, highway_type, road_name, junction=junction)
        if not roads:
            logger.warning("No matching roads found")
            return []

        # Randomly select roads
        if len(roads) > num_samples:
            roads = random.sample(roads, num_samples)

        # Get networks around each road
        networks = []
        for i, road in enumerate(roads):
            try:
                # Get midpoint of the road
                mid_lat = (road["from"][0] + road["to"][0]) / 2
                mid_lon = (road["from"][1] + road["to"][1]) / 2

                # Download network around the road
                G = ox.graph_from_point(
                    (mid_lat, mid_lon), dist=radius, network_type="drive"
                )

                # Save network to OSM file
                output_dir = Path("osm_exports")
                output_dir.mkdir(exist_ok=True)
                osm_path = output_dir / f"road_network_{i+1:03d}.osm"
                ox.save_graph_xml(G, osm_path)

                # Get network statistics
                stats = {
                    "num_nodes": len(G.nodes),
                    "num_edges": len(G.edges),
                    "road_types": {},
                    "total_length": 0,
                }

                # Count road types and calculate total length
                for _, _, data in G.edges(data=True):
                    road_type = data.get("highway", "unknown")
                    stats["road_types"][road_type] = (
                        stats["road_types"].get(road_type, 0) + 1
                    )
                    stats["total_length"] += data.get("length", 0)

                networks.append(
                    {"osm_path": str(osm_path), "stats": stats, "road_info": road}
                )

            except Exception as e:
                logger.warning(f"Failed to process road {i+1}: {e}")
                continue

        return networks

    except Exception as e:
        logger.error(f"Error getting random road networks: {e}")
        return []


if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(level=logging.INFO)

    # Test road query functions
    city = "San Francisco, CA, USA"

    # Test finding roads
    roads = find_roads_by_tag(city, highway_type="motorway_link")
    print(f"Found {len(roads)} roads")

    # Test getting road info
    if roads:
        info = query_road_info(roads[0]["from"][0], roads[0]["from"][1])
        print("Road info:", info)

    # Test getting random networks
    networks = get_random_road_networks(city, num_samples=2)
    print(f"Generated {len(networks)} random networks")
