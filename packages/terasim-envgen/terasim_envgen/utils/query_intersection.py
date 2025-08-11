#!/usr/bin/env python3
"""
query_intersection.py - Module for querying and analyzing intersections from OpenStreetMap data
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


class IntersectionQuery:
    def __init__(self, config_file=None):
        """Initialize the intersection query module."""
        # Load configuration
        if config_file is None:
            config_file = Path(__file__).parent / "config.yaml"
        with open(config_file, "r") as f:
            self.config = yaml.safe_load(f)

        # Get settings from config
        self.default_city = self.config["map_search"]["default_city"]
        self.bbox_size = self.config["map_search"]["bbox_size"]
        self.max_results = self.config["map_search"]["max_results"]

        # Get randomization settings
        viz_config = self.config.get("visualization", {}).get("preview", {})
        self.randomize_enabled = viz_config.get("randomize", {}).get("enabled", True)
        self.randomize_node_selection = viz_config.get("randomize", {}).get(
            "node_selection", True
        )

        # Set random seed
        seed_value = viz_config.get("randomize", {}).get("seed", 0)
        if seed_value == 0:
            random.seed(time.time())
            logger.info("Using time-based random seed")
        else:
            random.seed(seed_value)
            logger.info(f"Using fixed random seed: {seed_value}")

    def find_junction(self, city=None, filters=None, max_samples=None):
        """
        Find intersections in a city that match the specified filters.

        Args:
            city (str): City name to search in
            filters (dict): Filter criteria from prompt_parser
            max_samples (int, optional): Maximum number of samples to return, overrides config value if provided

        Returns:
            list: List of tuples (node_id, x, y) for matching intersections
        """
        if city is None:
            city = self.default_city
            
        # Use provided max_samples if available, otherwise use config value
        max_results = max_samples if max_samples is not None else self.max_results

        try:
            # Download the street network
            logger.info(f"Downloading street network for {city}")
            G = ox.graph_from_place(city, network_type="drive", retain_all=True)
            # nodes = ox.graph_to_gdfs(G, nodes=True, edges=False)
            logger.info(
                f"Downloaded network with {len(G.nodes)} nodes and {len(G.edges)} edges"
            )

            # Calculate center point from graph nodes
            node_ys = [data["y"] for _, data in G.nodes(data=True)]
            node_xs = [data["x"] for _, data in G.nodes(data=True)]
            center_y = sum(node_ys) / len(node_ys)
            center_x = sum(node_xs) / len(node_xs)

            # Get the bounding box
            bbox = ox.utils_geo.bbox_from_point(
                (center_y, center_x), dist=self.bbox_size
            )
            logger.info(f"Using bounding box: {bbox}")

            # Find intersections based on filters
            matching_nodes = []
            for node in G.nodes(data=True):
                node_id, data = node
                if self._matches_filters(G, node_id, filters):
                    matching_nodes.append((node_id, data["x"], data["y"]))

            # Randomly select nodes if needed
            if self.randomize_enabled and self.randomize_node_selection:
                if len(matching_nodes) > max_results:
                    matching_nodes = random.sample(matching_nodes, max_results)
                    logger.info(f"Randomly selected {max_results} intersections")

            logger.info(f"Found {len(matching_nodes)} matching intersections")
            return matching_nodes

        except Exception as e:
            logger.error(f"Error finding intersections: {e}")
            return []

    def _matches_filters(self, G, node_id, filters):
        """Check if a node matches the given filters."""
        if filters is None:
            return True

        # Get connected edges
        street_count = G.nodes[node_id].get("street_count", 0)
        
        # Check street count constraints if specified
        if "min_street_count" in filters and street_count < filters["min_street_count"]:
            return False
        if "max_street_count" in filters and street_count > filters["max_street_count"]:
            return False

        # Check highway tag if specified
        if "highway" in filters:
            # Get node's highway tag
            node_highway = G.nodes[node_id].get("highway", None)
            # Return False if node has no highway tag or doesn't match filter
            if not node_highway or node_highway not in filters["highway"]:
                return False

        return True

    def get_intersection_info(self, lat, lon, dist=500):
        """
        Get detailed information about an intersection at specific coordinates.

        Args:
            lat (float): Latitude
            lon (float): Longitude
            dist (float): Distance in meters around the point

        Returns:
            dict: Dictionary containing intersection information
        """
        try:
            # Download the street network around the point
            G = ox.graph_from_point((lat, lon), dist=dist, network_type="drive")

            # Find the nearest node
            nearest_node = ox.nearest_nodes(G, lon, lat)
            node_data = G.nodes[nearest_node]

            # Get connected edges
            edges = list(G.edges(nearest_node, data=True))

            # Count edge types
            edge_types = {}
            for _, _, data in edges:
                edge_type = data.get("highway", "unknown")
                edge_types[edge_type] = edge_types.get(edge_type, 0) + 1

            return {
                "node_id": nearest_node,
                "coordinates": (node_data["y"], node_data["x"]),
                "num_edges": len(edges),
                "edge_types": edge_types,
                "tags": node_data.get("tags", {}),
            }

        except Exception as e:
            logger.error(f"Error getting intersection info: {e}")
            return None


if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(level=logging.INFO)

    # Test the intersection query module
    query = IntersectionQuery()

    # Test finding intersections
    intersections = query.find_junction(
        city="San Francisco, CA, USA", filters={"min_street_count": 3, "tags": ["stop"]}
    )
    print(f"Found {len(intersections)} intersections")

    # Test getting intersection info
    if intersections:
        info = query.get_intersection_info(intersections[0][1], intersections[0][2])
        print("Intersection info:", info)
