#!/usr/bin/env python3
"""
map_searcher.py - Main coordinator for map processing operations
"""
import json
import uuid
import logging
import numpy as np
import yaml
import random
import time
from pathlib import Path
import osmnx as ox
ox.settings.all_oneway = True
import matplotlib.pyplot as plt
import os
import sys
# Import specialized modules
from terasim_envgen.utils.query_road import find_roads_by_tag, query_road_info
from terasim_envgen.utils.query_intersection import IntersectionQuery
from terasim_envgen.utils.metadata import save_metadata, load_metadata, update_metadata
import requests
import math
logger = logging.getLogger(__name__)
import googlemaps
from googlemaps.convert import decode_polyline
from shapely.geometry import LineString, Polygon
import geopandas as gpd
from pyproj import Geod
import polyline

# SUMO tools path is automatically configured by terasim_envgen.__init__.py

class MapSearcher:
    def __init__(self, config_file=None):
        """Initialize the map searcher coordinator."""
        # Load configuration
        if config_file is None:
            config_file = Path(__file__).parent / "config.yaml"
        with open(config_file, "r") as f:
            self.config = yaml.safe_load(f)

        # Initialize specialized modules
        self.junction_query = IntersectionQuery(config_file)

        # Get settings from config
        self.default_city = self.config["map_search"]["default_city"]
        self.bbox_size = self.config["map_search"]["bbox_size"]
        self.max_results = self.config["map_search"]["max_results"]

        # Get output settings from config
        output_config = self.config.get("output", {})
        self.base_dir = Path(output_config.get("base_dir", "output"))
        self.output_configs = {
            "intersections": output_config.get(
                "intersections", {"prefix": "intersection_"}
            ),
            "roads": output_config.get("roads", {"prefix": "road_"}),
        }

        # Get visualization settings from config
        viz_config = self.config.get("visualization", {}).get("preview", {})
        self.randomize_enabled = viz_config.get("randomize", {}).get("enabled", True)
        self.randomize_node_selection = viz_config.get("randomize", {}).get(
            "node_selection", True
        )
        self.randomize_visualization = viz_config.get("randomize", {}).get(
            "visualization", True
        )
        self.randomize_map_style = viz_config.get("randomize", {}).get(
            "map_style", True
        )
        self.satellite_view = viz_config.get("satellite_view", True)
        self.interactive_map = viz_config.get("interactive_map", True)
        self.dpi = viz_config.get("dpi", 300)
        self.fig_width = viz_config.get("figure_width", 16)
        self.fig_height = viz_config.get("figure_height", 8)

        # Set random seed
        seed_value = viz_config.get("randomize", {}).get("seed", 0)
        if seed_value == 0:
            random.seed(time.time())
            logger.info("Using time-based random seed")
        else:
            random.seed(seed_value)
            logger.info(f"Using fixed random seed: {seed_value}")

    def get_maps_through_route_from_mapbox_response(self, mapbox_response, output_dir=None, target_split_distance=2000, bbox_size=1000, multiprocess=True):
        polyline_str = mapbox_response[0]["geometry"]
        coordinates = polyline.decode(polyline_str)
        coordinates_geod = [(coord[1], coord[0]) for coord in coordinates]   # lon, lat for geod

        geod = Geod(ellps='WGS84')
        # Create a LineString from the coordinates for geometric operations
        line_geod = LineString(coordinates_geod)        
        length = geod.geometry_length(line_geod)
        num_points = int(length / target_split_distance) + 1
        points = [line_geod.interpolate(i/float(num_points-1), normalized=True) for i in range(num_points)]
        interpolated_coords_geod = [(point.x, point.y) for point in points]
        interpolated_coords = [(coord[1], coord[0]) for coord in interpolated_coords_geod] # revert lon, lat to lat, lon
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        from multiprocessing import Pool, cpu_count

        # Create a list of arguments for each process
        process_args = []
        for center_point in interpolated_coords:
            av_route = self.get_av_route(line_geod, center_point, bbox_size)
            scene_dir = output_dir / f"{interpolated_coords.index(center_point)}"
            process_args.append((center_point, scene_dir, bbox_size, av_route))

        if multiprocess:
            with Pool(processes=cpu_count()) as pool:
                pool.starmap(self.save_osm_and_visualization_for_point, process_args)
        else:
            for process_arg in process_args:
                self.save_osm_and_visualization_for_point(*process_arg)
        
        return coordinates

    def get_maps_through_route(self, origin:str, destination:str, mode="driving", output_dir=None, target_split_distance=2000, bbox_size=1000):
        """
        Get maps through route using Google Maps API, token defined in .env file.
        
        Args:
            origin: Starting point address or coordinates
            destination: Destination address or coordinates
            mode: Transportation mode (driving, walking, bicycling, transit)
            output_dir: Directory to save the output
            target_split_distance: Target distance between sample points in meters
            bbox_size: Size of bounding box in meters
            
        Returns:
            List of coordinates along the route
        """
        # Get token from .env file
        token = os.getenv("GOOGLE_MAPS_API_KEY")
        if not token:
            raise ValueError("GOOGLE_MAPS_API_KEY not found in .env file")
        gmaps = googlemaps.Client(key=token)
        directions_result = gmaps.directions(origin, destination, mode=mode)

        # Get all steps from the route
        steps = directions_result[0]["legs"][0]["steps"]
        
        # Extract and decode polylines from each step
        coordinates = []
        for step in steps:
            step_polyline = step["polyline"]["points"] 
            step_coords = decode_polyline(step_polyline)
            coordinates.extend(step_coords)
            
        geod = Geod(ellps='WGS84')
    
        # Convert coordinates to lat, lon format
        coordinates = [(coord['lat'], coord['lng']) for coord in coordinates] # lat, lon
        coordinates_geod = [(coord[1], coord[0]) for coord in coordinates]   # lon, lat for geod

        
        # Create a LineString from the coordinates for geometric operations
        line_geod = LineString(coordinates_geod)        
        length = geod.geometry_length(line_geod)
        num_points = int(length / target_split_distance) + 1
        points = [line_geod.interpolate(i/float(num_points-1), normalized=True) for i in range(num_points)]
        interpolated_coords_geod = [(point.x, point.y) for point in points]
        interpolated_coords = [(coord[1], coord[0]) for coord in interpolated_coords_geod] # revert lon, lat to lat, lon
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        from multiprocessing import Pool, cpu_count

        # Create a list of arguments for each process
        process_args = []
        for center_point in interpolated_coords:
            av_route = self.get_av_route(line_geod, center_point, bbox_size)
            scene_dir = output_dir / f"{interpolated_coords.index(center_point)}"
            process_args.append((center_point, scene_dir, bbox_size, av_route))

        # self.plot_maps_through_route(interpolated_coords, process_args)

        # Use multiprocessing pool to process points in parallel
        # with Pool(processes=cpu_count()) as pool:
        #     pool.starmap(self.save_osm_and_visualization_for_point, process_args)

        for process_arg in process_args:
            self.save_osm_and_visualization_for_point(*process_arg)
        
        return coordinates
    
    def plot_maps_through_route(self, interpolated_coords, process_args):
        import matplotlib.pyplot as plt
        x_coords = [coord[1] for coord in interpolated_coords]
        y_coords = [coord[0] for coord in interpolated_coords]
        plt.scatter(x_coords, y_coords, s=100)

        # line_geod_x_coords = list(line_geod.coords.xy[0])
        # line_geod_y_coords = list(line_geod.coords.xy[1])
        # plt.scatter(line_geod_x_coords, line_geod_y_coords, s=20)

        for process_arg in process_args:
            x_coords = [coord[1] for coord in process_arg[3]]
            y_coords = [coord[0] for coord in process_arg[3]]
            # Plot bounding box around center point
            earth_radius = 6371000  # Earth's radius in meters
            bbox_size_meters = process_arg[2]  # bbox_size from process_arg tuple
            center_lat, center_lon = process_arg[0]  # center point from process_arg tuple
            
            # Calculate bbox offsets
            lat_offset = (bbox_size_meters / earth_radius) * (180 / math.pi)
            lon_offset = (bbox_size_meters / earth_radius) * (180 / math.pi) / math.cos(center_lat * math.pi / 180)
            
            # Create bbox coordinates
            bbox_x = [center_lon - lon_offset, center_lon + lon_offset, 
                     center_lon + lon_offset, center_lon - lon_offset, center_lon - lon_offset]
            bbox_y = [center_lat - lat_offset, center_lat - lat_offset,
                     center_lat + lat_offset, center_lat + lat_offset, center_lat - lat_offset]
            
            # Plot bbox
            # plt.plot(bbox_x, bbox_y, 'b--', alpha=0.5)
            plt.plot(x_coords, y_coords, "ro")
        
        plt.savefig("route.png")

        plt.close()
    
    def analyse_polyline_geod(self, coordinates_geod):
        # calculate the distance between each point and print the max distance
        distances = []
        for i in range(len(coordinates_geod) - 1):
            # get the distance between the two points
            lons = [coordinates_geod[i][0], coordinates_geod[i+1][0]]
            lats = [coordinates_geod[i][1], coordinates_geod[i+1][1]]
            distance = geod.line_length(lons, lats)
            distances.append(distance)
        print(f"Max distance: {max(distances)}")
        print(f"Min distance: {min(distances)}")
        print(f"Average distance: {sum(distances) / len(distances)}")
        x_coords = [coord[0] for coord in coordinates_geod]
        y_coords = [coord[1] for coord in coordinates_geod]
        plt.plot(x_coords, y_coords)
        plt.savefig("route.png")
        plt.close()
    
    def get_av_route(self, line_geod, center_point, bbox_size, interpolate_distance=50):
        """
        Get AV route for a center point within a bounding box
        
        Args:
            line_geod: Shapely LineString of the route in geod format [(lon, lat), ...]
            center_point: (lat, lon) center point of the bounding box
            bbox_size: Size of bounding box in meters
            
        Returns:
            List of coordinates [(lat, lon), ...] within the bounding box
        """
        # Convert center_point to (lon, lat) format for consistency
        center_lon, center_lat = center_point[1], center_point[0]
        
        # Calculate bounding box coordinates
        earth_radius = 6371000  # Earth's radius in meters
        lat_offset = (bbox_size / earth_radius) * (180 / math.pi)
        lon_offset = (bbox_size / earth_radius) * (180 / math.pi) / math.cos(center_lat * math.pi / 180)
        
        # Define bbox coordinates
        north = center_lat + lat_offset
        south = center_lat - lat_offset
        east = center_lon + lon_offset
        west = center_lon - lon_offset

        # Create a polygon for the bounding box (in lon, lat format)
        bbox_polygon = Polygon([
            (west, south), (east, south), (east, north), (west, north), (west, south)
        ])
        
        # Check if line intersects with bounding box
        if not line_geod.intersects(bbox_polygon):
            logger.warning("Route does not intersect with the bounding box")
            return []
        
        # Get the intersection of the line with the bounding box
        # This gives us the portion of the route within the bounding box
        route_section = line_geod.intersection(bbox_polygon)
        
        # Handle different geometry types that could result from the intersection
        if route_section.geom_type == 'MultiLineString':
            route_section = route_section.geoms[0]
        elif route_section.geom_type == 'LineString':
            pass
        else:
            logger.warning(f"Unexpected geometry type: {route_section.geom_type}")
            return [] # TODO: handle this case
    

        num_interpolate_points = int(2 * bbox_size / interpolate_distance) + 1
        points = [route_section.interpolate(i/float(num_interpolate_points-1), normalized=True) for i in range(num_interpolate_points)]
        interpolated_coords_geod = [(point.x, point.y) for point in points]
        route_points_in_bbox = [(coord[1], coord[0]) for coord in interpolated_coords_geod] # revert lon, lat to lat, lon
        
        return route_points_in_bbox

    def find_center_points(self, data):
        """
        Find center points from road or intersection data.
        
        Args:
            data: List of dictionaries containing road or intersection data
            
        Returns:
            List of dictionaries with center points and metadata
        """
        center_points = []
        
        for item in data:
            # Get coordinates
            from_coords = item["from"]
            to_coords = item["to"]
            
            # Get center point for the map
            mid_lat = (from_coords[0] + to_coords[0]) / 2
            mid_lon = (from_coords[1] + to_coords[1]) / 2
            
            # Create metadata for the point
            point_data = (mid_lat, mid_lon)
            center_points.append(point_data)
        return center_points

    def save_osm_and_visualization_for_point(self, point_data, scene_dir, bbox_size=None, av_route=None):
        """
        Save OSM data and visualization for a single center point.
        
        Args:
            point_data: (lat, lon)
            scene_dir: Directory to save the output
            bbox_size: Size of bounding box in meters
            av_route: Optional list of route coordinates [(lat, lon), ...] within the bounding box
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Ensure the scene directory exists
        scene_dir = Path(scene_dir)
        scene_dir.mkdir(exist_ok=True, parents=True)
        
        # Get center coordinates
        mid_lat, mid_lon = point_data
        
        # Create metadata.json for the scene
        metadata = {
            "center_coordinates": (mid_lat, mid_lon),
            "bbox_size": bbox_size,
            "scene_id": scene_dir.name,
            "av_route": av_route
        }
        
        metadata_path = scene_dir / "metadata.json"
        save_metadata(metadata_path, metadata)
        
        # Save OSM data
        try:
            osm_path = self._save_osm_data_webwizard(mid_lat, mid_lon, scene_dir, bbox_size)
        except Exception as e:
            logger.warning(f"Failed to save OSM WebWizard data for scene {scene_dir.name}: {str(e)}")
        # try:
        #     osm_path = self._save_osm_data_osmnx(mid_lat, mid_lon, scene_dir, bbox_size)
        # except Exception as e:
        #     logger.warning(f"Failed to save OSM OSMNX data for scene {scene_dir.name}: {str(e)}")
        #     return False
        if not osm_path:
            logger.error(f"Failed to save OSM data for scene {scene_dir.name}")
            return False
        
        # Create visualization for < 500m
        if bbox_size < 500:
            self._create_visualization(mid_lat, mid_lon, scene_dir, bbox_size, av_route)
        else:
            logger.warning(f"Bbox size is too large for visualization: {bbox_size}, skipping visualization")
        
        return True

    def _create_visualization(self, mid_lat, mid_lon, scene_dir, bbox_size, av_route=None):
        """
        Create and save visualization for a center point.
        
        Args:
            mid_lat: Latitude of center point
            mid_lon: Longitude of center point
            scene_dir: Directory to save visualization
            bbox_size: Size of bounding box in meters
            av_route: Optional list of route coordinates [(lat, lon), ...] within the bounding box
        """
        # Create subgraph around the point
        subgraph = ox.graph_from_point(
            (mid_lat, mid_lon), 
            dist=bbox_size, 
            network_type="drive", 
            simplify=False, 
            retain_all=True, 
            truncate_by_edge=True
        )
        
        # Create a single figure and axes
        # Using a square figure size, adjust as needed
        fig, ax = plt.subplots(figsize=(10, 10)) 
        
        # Plot base network (will be mostly overlaid by satellite imagery)
        ox.plot_graph(
            subgraph,
            ax=ax,
            show=False,
            close=False,
            node_color="none",  # Hide nodes initially
            node_size=0,
            edge_color="white", # Roads will be white on satellite imagery
            edge_linewidth=1.5, # Thicker lines for satellite overlay
            bgcolor="k", # Black background, will be covered by basemap
        )
        
        # Add satellite imagery
        try:
            import contextily as cx
            cx.add_basemap(
                ax,
                crs=subgraph.graph["crs"],
                source=cx.providers.Esri.WorldImagery,
                zoom='auto' # Adjust zoom automatically
            )
        except Exception as e:
            logger.error(f"Failed to add satellite imagery: {str(e)}")

        # Plot AV route on top if available
        if av_route and len(av_route) >= 2:
            route_x = [point[1] for point in av_route]  # lon values
            route_y = [point[0] for point in av_route]  # lat values
            ax.plot(route_x, route_y, color="red", linewidth=2, marker='o', markersize=3) # Using plot for a connected line
            ax.scatter(route_x, route_y, color="red", s=10) # Add scatter for distinct points if needed

        ax.set_title("Map & AV Route") # Updated title
        ax.set_aspect('equal') # Ensure XY scales are equal
        
        # Turn off axis labels and ticks for a cleaner map view
        ax.set_axis_off()

        # Adjust layout and save the figure
        plt.tight_layout(pad=0.1) 
        plt.savefig(scene_dir / "preview.png", dpi=self.dpi, bbox_inches="tight")
        plt.close(fig) # Ensure the correct figure is closed

    def save_map_and_visualization(self, data, output_dir, output_dir_name=None, bbox_size=None):
        """
        Save road visualizations (static and interactive) for multiple points
        
        Args:
            data: List of dictionaries containing road or intersection data
            output_dir: Directory to save the output
            output_dir_name: Name prefix for output directories
            bbox_size: Size of bounding box in meters
            
        Returns:
            List: List of paths to created scene directories
        """
        # Find center points
        center_points = self.find_center_points(data)
        
        # Process each point
        scene_dirs = []
        output_dir = Path(output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        
        for point_data in center_points:
            # Generate a unique ID for this scene
            scene_id = str(uuid.uuid4())[:8]
            
            # Create scene directory
            if output_dir_name:
                scene_dir = output_dir / f"{output_dir_name}_{scene_id}"
            else:
                logger.error("output_dir_name is not set")
                continue
            
            # Process the point
            success = self.save_osm_and_visualization_for_point(
                point_data, scene_dir, bbox_size
            )
            
            if success:
                scene_dirs.append(scene_dir)
        
        return scene_dirs

    def find_section(self, city=None, section_type=None, save_plots=True, max_samples=3, output_dir_name=None):
        """
        Find sections in a city based on section type.

        Args:
            city (str): City name to search in
            section_type (str): Type of section to search for
            save_plots (bool): Whether to save visualization plots

        Returns:
            list: List of matching sections
        """
        city = city or self.default_city

        # Define section type mappings
        road_types = {
            "highway": ["motorway"],
            "highway_ramp": ["motorway_link"],
            "arterial": ["primary", "secondary"],
            "collector": ["tertiary"],
            "local": ["residential", "unclassified"],
            "roundabout": ["roundabout"]
        }

        junction_types = {
            "signalized": {"highway": ["traffic_signals"]},
            "stop_sign": {"highway": ["stop"]},
            "t_intersection": {"min_street_count": 3, "max_street_count": 3},
            "crossroads": {"min_street_count": 4}
        }

        bbox_size = 500 if "highway" in section_type else 150

        if output_dir_name is None:
            # Shorten city name by taking first word and removing special chars
            short_city = city.replace(' ', '_').replace(",", "")
            output_dir_name = f"{short_city}_{section_type}"
            logger.info(f"Using output directory name: {output_dir_name}")
        # Check if section type is a road type
        if section_type in road_types:
            logger.info(f"Searching for road type: {section_type}")
            
            # Special handling for roundabout
            if section_type == "roundabout":
                logger.info("Searching for roundabout roads")
                return self.search_roads(
                    city=city,
                    road_types=road_types[section_type],
                    save_plots=save_plots,
                    junction_tag="roundabout",
                    max_samples=max_samples,
                    output_dir_name=output_dir_name,
                    bbox_size=bbox_size
                )
            else:
                return self.search_roads(
                    city=city,
                    road_types=road_types[section_type],
                    save_plots=save_plots,
                    max_samples=max_samples,
                    output_dir_name=output_dir_name,
                    bbox_size=bbox_size
                )

        # Check if section type is a junction type
        elif section_type in junction_types:
            logger.info(f"Searching for junction type: {section_type}")
            return self.find_junction(
                city=city,
                filters=junction_types[section_type],
                save_plots=save_plots,
                max_samples=max_samples,
                output_dir_name=output_dir_name,
                bbox_size=bbox_size
            )

        else:
            logger.error(f"Unknown section type: {section_type}")
            return []

    def _get_output_dir(self):
        """
        Get the output directory for a specific type of output.

        Returns:
            Path: Path to the output directory
        """

        # Instead of using a subdirectory, now we just use the base directory
        output_dir = self.base_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir
    
    def _save_osm_data_osmnx(self, lat, lon, output_dir, bbox_size=None):
        """
        Save OSM data for a given location using OSMnx.
        """
        if bbox_size is None:
            bbox_size = self.bbox_size
            
        # Get the road network for the bounding box
        graph = ox.graph_from_point((lat, lon), dist=bbox_size, network_type="drive", simplify=False, retain_all=True, truncate_by_edge=True)
        
        # Save the graph to an OSM file
        osm_file = output_dir / "map.osm"
        ox.save_graph_xml(graph, osm_file)
        return osm_file
    
    def _save_osm_data_webwizard(self, lat, lon, output_dir, bbox_size=None):
        """
        Save OSM data for a given location using SUMO's osmWebWizard.
        """
        if bbox_size is None:
            bbox_size = self.bbox_size

        abs_output_dir = os.path.abspath(output_dir)

        # Calculate bounding box coordinates
        earth_radius = 6371000  # Earth's radius in meters
        lat_offset = (bbox_size / earth_radius) * (180 / math.pi)
        lon_offset = (bbox_size / earth_radius) * (180 / math.pi) / math.cos(lat * math.pi / 180)
        
        # Define bbox coordinates
        north = lat + lat_offset
        south = lat - lat_offset
        east = lon + lon_offset
        west = lon - lon_offset

        from osmWebWizard import Builder
        data = {
            'poly': True,
            'duration': 3600,
            'publicTransport': False,
            'leftHand': False,
            'decal': False,
            'carOnlyNetwork': False,
            'vehicles': {
                'passenger': {'fringeFactor': 5, 'count': 12},
                # 'bicycle': {'fringeFactor': 5, 'count': 12},
                # 'pedestrian': {'fringeFactor': 5, 'count': 12}
            },
            'roadTypes': {
                'Highway': ['motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link', 'secondary', 'secondary_link', 'tertiary', 'tertiary_link', 'unclassified', 'residential', 'living_street', "cycleway"],
                # 'Highway': ['motorway', 'motorway_link'],
                # 'Railway': [],
                # 'Aeroway': [],
                # 'Waterway': [],
                # 'Aerialway': [],
                # 'Route': []
            },
            'coords': [west, south, east, north],
            'outputDir': str(abs_output_dir),
            'outputDirExistOk': True,
            'options': '--default.lanewidth,3.5',
        }
        builder = Builder(data, True)
        builder.build()
        # builder.makeConfigFile()
        # builder.createBatch()

        # Get the path to the OSM file
        osm_file = os.path.join(output_dir, "map.osm")
        return osm_file

    def _save_osm_data_overpass(self, lat, lon, output_dir, bbox_size=None):
        """
        Save OSM data for a given location, including all road types (drivable, cycling, pedestrian).

        Args:
            lat (float): Latitude
            lon (float): Longitude
            output_dir (Path): Directory to save the OSM file
            bbox_size (float, optional): Size of bounding box in meters

        Returns:
            str: Path to the saved OSM file
        """
        # Set OSMnx settings for proper OSM XML export
        ox.settings.all_oneway = True  # Required for OSM XML export
        if bbox_size is None:
            bbox_size = self.bbox_size
            
        # Calculate bounding box coordinates
        earth_radius = 6371000  # Earth's radius in meters
        lat_offset = (bbox_size / earth_radius) * (180 / math.pi)
        lon_offset = (bbox_size / earth_radius) * (180 / math.pi) / math.cos(lat * math.pi / 180)
        
        # Define bbox coordinates
        north = lat + lat_offset
        south = lat - lat_offset
        east = lon + lon_offset
        west = lon - lon_offset
        
        # Download OSM data using Overpass API
        # Using a more JOSM-friendly format with explicit OSM headers
        overpass_url = "https://overpass-api.de/api/interpreter"
        
        # Query to get all main roads (drivable, cycling, pedestrian) except for service roads (e.g., parking lots, driveways, etc.)
        query = f"""
        [out:xml][timeout:90];
        (
          way["highway"!="service"]({south},{west},{north},{east});  // Get all highways except service roads
          >;  // Get all nodes that are part of ways
          relation(bw)->.rel;  // Get all relations that ways are part of
          node(r.rel);  // Get all nodes that are part of those relations
          way(r.rel);  // Get all ways that are part of those relations
        );
        out meta;  // Include metadata which JOSM expects
        """
        
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        response = requests.post(overpass_url, data=query, headers=headers)
        
        if response.status_code != 200:
            raise Exception(f"Failed to download OSM data: {response.status_code}, Response: {response.text}")
        
        # Check if the response contains valid OSM XML
        if not response.content.startswith(b'<?xml') and not b'<osm' in response.content[:100]:
            logger.error("Response doesn't appear to be valid OSM XML")
            logger.debug(f"Response starts with: {response.content[:200]}")
            raise Exception("Invalid OSM XML received from Overpass API")
            
        # Create a file to store the OSM data with proper XML formatting
        temp_osm_file = output_dir / "map.osm"
        with open(temp_osm_file, 'wb') as f:
            f.write(response.content)
            
        logger.info(f"Saved complete road network to {temp_osm_file}")
        
        # Verify the file is readable as XML
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(temp_osm_file)
            root = tree.getroot()
            if root.tag != 'osm':
                logger.warning("Root element is not 'osm', JOSM may not recognize this file")
        except Exception as e:
            logger.error(f"Generated file is not valid XML: {str(e)}")
            
        return str(temp_osm_file)

    def find_junction(
        self, city=None, filters=None, save_plots=True, max_samples=3, output_dir_name=None, bbox_size=None
    ):
        """Find intersections in a city."""
        city = city or self.default_city
        filters = filters or {}

        # Get intersections
        intersections = self.junction_query.find_junction(city, filters, max_samples=max_samples)
        
        # Randomly sample intersections if we have more than max_samples
        # Note: Now handling this in the junction_query.find_junction method
        # Keeping this comment for reference
        
        # Convert intersection data to road-like format
        road_like_intersections = []
        for node_id, x, y in intersections:
            # Create a road-like dictionary for each intersection
            # x is longitude, y is latitude
            road_like_intersections.append(
                {
                    "from": (y, x),  # Use (lat, lon) format to match road data
                    "to": (y, x),  # Use (lat, lon) format to match road data
                    "from_node": node_id,
                    "to_node": node_id,
                    "name": f"Intersection_{node_id}",
                }
            )

        if save_plots and road_like_intersections:
            output_dir = self._get_output_dir()
            self.save_map_and_visualization(road_like_intersections, output_dir, output_dir_name, bbox_size)

        return road_like_intersections

    def search_roads(
        self,
        city=None,
        road_types=None,
        road_name=None,
        save_plots=True,
        max_samples=3,
        junction_tag=None,
        output_dir_name=None,
        bbox_size=None
    ):
        """Search for roads in a city."""
        city = city or self.default_city
        road_types = road_types or ["motorway_link"]

        # Use the road query module to find roads
        if junction_tag:
            # For roads with specific junction tags (like roundabouts)
            all_roads = find_roads_by_tag(city, junction="roundabout")
            logger.info(f"Searching for roads with junction tag: {junction_tag}")
        else:
            # Regular road search by highway type
            all_roads = find_roads_by_tag(city, highway_type=road_types[0], road_name=road_name)

        # Randomly sample roads if we have more than max_samples
        if len(all_roads) > max_samples:
            roads = random.sample(all_roads, max_samples)
            logger.info(
                f"Randomly sampled {max_samples} roads from {len(all_roads)} total roads"
            )
        else:
            roads = all_roads
            logger.info(f"Using all {len(roads)} found roads")

        if save_plots and roads:
            output_dir = self._get_output_dir()
            self.save_map_and_visualization(roads, output_dir, output_dir_name, bbox_size)

        return roads

    def get_road_info(self, lat, lon, dist=500):
        """
        Get detailed information about roads near specific coordinates.

        Args:
            lat (float): Latitude
            lon (float): Longitude
            dist (float): Distance in meters around the point

        Returns:
            dict: Dictionary containing road information
        """
        return query_road_info(lat, lon, dist)


if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(level=logging.INFO)

    # Test the coordinator
    searcher = MapSearcher()

    # Test intersection search
    intersections = searcher.find_junction(
        city="San Francisco, CA, USA", filters={"min_street_count": 3, "tags": ["stop"]}
    )
    print(f"Found {len(intersections)} intersections")

    # Test road search
    roads = searcher.search_roads(
        city="San Francisco, CA, USA", road_types=["motorway_link"]
    )
    print(f"Found {len(roads)} roads")
