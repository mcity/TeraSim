#!/usr/bin/env python3
"""
traffic_flow_generator.py - Generate traffic flows for SUMO simulations
"""

import json
import os
import logging
from pathlib import Path
import yaml
import subprocess
import xml.etree.ElementTree as ET
import random
import glob
from tqdm import tqdm
import sumolib

logger = logging.getLogger(__name__)


class TrafficFlowGenerator:
    def __init__(self, config_file=None, allow_fringe=True):
        # Load environment variables
        if config_file is None:
            config_file = Path(__file__).parent / "config.yaml"
        # Load configuration
        with open(config_file, "r") as f:
            self.config = yaml.safe_load(f)

        # Check SUMO environment - first try from .env file
        self.sumo_home = os.getenv("SUMO_HOME")
        self.tools_path = os.getenv("SUMO_TOOLS_PATH")
        
        # Fall back to environment variables if not in .env
        if not self.sumo_home:
            self.sumo_home = os.environ.get("SUMO_HOME")
            
        if not self.sumo_home:
            logger.warning(
                "SUMO_HOME environment variable not set. "
                "Using default paths for SUMO tools."
            )
            # Try to use common locations
            if os.path.exists("/usr/share/sumo"):
                self.sumo_home = "/usr/share/sumo"
                self.tools_path = "/usr/share/sumo/tools"
            elif os.path.exists("C:\\Program Files (x86)\\Eclipse\\Sumo"):
                self.sumo_home = "C:\\Program Files (x86)\\Eclipse\\Sumo"
                self.tools_path = os.path.join(self.sumo_home, "tools")
            else:
                self.tools_path = None
                logger.warning(
                    "SUMO tools path not found. Random trips generation may fail."
                )
        elif not self.tools_path:
            self.tools_path = os.path.join(self.sumo_home, "tools")
            
        # Default random trips script path
        self.random_trips_script = None
        
        # Try to find randomTrips.py in the project
        project_random_trips = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "utils",
            "randomTrips.py"
        )
        if os.path.exists(project_random_trips):
            self.random_trips_script = project_random_trips
            logger.info(f"Using project randomTrips.py: {project_random_trips}")
            
        # Initialize allow_fringe property 
        self.allow_fringe = allow_fringe
        
        # Log configuration
        logger.info(f"SUMO_HOME: {self.sumo_home}")
        logger.info(f"SUMO tools path: {self.tools_path}")
        logger.info(f"Allow fringe: {self.allow_fringe}")

    def generate_av_route(self, map_path, map_metadata_path):
        """
        Generate a route for a vehicle using the map and map metadata. If av route is in metadata, use it. Otherwise, generate a new route.
        
        Args:
            map_path (str): Path to SUMO network file
            map_metadata_path (str): Path to metadata JSON file containing AV route
            
        Returns:
            list: List of SUMO edge IDs representing the matched route
        """
        
        # Load the map
        sumo_net = sumolib.net.readNet(map_path, withInternal=True)
        sumo_net_no_internal = sumolib.net.readNet(map_path)
        
        # Load the map metadata
        with open(map_metadata_path, 'r') as f:
            map_metadata = json.load(f)
        
        if "av_route" in map_metadata and map_metadata["av_route"] is not None:
            # Get AV route coordinates (expected format is a list of [lat, lon] points)
            av_route = map_metadata["av_route"]
            av_route_xy = [sumo_net.convertLonLat2XY(point[1], point[0]) for point in av_route]
            # Verify format and convert if necessary
            # Map the lat/lon coordinates to SUMO edges
            # delta parameter controls how far to search for edges (in meters)
            av_route_sumo_edges = sumolib.route.mapTrace(av_route_xy, sumo_net, delta=10, fillGaps=100, verbose=True)
            # av_route_sumo_edges_no_internal = sumolib.route.mapTrace(av_route_xy, sumo_net_no_internal, delta=10, verbose=True)
            av_route_sumo_with_internal = sumolib.route.addInternal(sumo_net, av_route_sumo_edges)
        else:
            av_route_sumo_edges, av_route_sumo_with_internal = self.generate_av_fallback_route(map_path)

        av_route_sumo_with_internal_xy = []
        for edge in av_route_sumo_with_internal:
            edge_shape = edge.getShape()
            edge_shape_xy = [sumo_net.convertXY2LonLat(point[0], point[1]) for point in edge_shape]
            av_route_sumo_with_internal_xy.extend(edge_shape_xy)
        av_route_sumo_points_latlon = [(point[1], point[0]) for point in av_route_sumo_with_internal_xy]

        # save av_route_sumo_points_latlon to metadata.json
        map_metadata["av_route_sumo"] = av_route_sumo_points_latlon
        if "av_route" not in map_metadata:
            map_metadata["av_route"] = av_route_sumo_points_latlon
        with open(map_metadata_path, "w") as f:
            json.dump(map_metadata, f)

        return av_route_sumo_edges, av_route_sumo_points_latlon
    
    def generate_av_fallback_route(self, map_path, *, max_candidates=128, use_time_metric=False, seed=None):
        """
        Generate a fallback AV route by selecting the longest (by distance or time) shortest path
        between two peripheral edges in a SUMO network.

        Args:
            map_path (str): Path to SUMO network file
            max_candidates (int): cap on number of candidate edges to consider (for O(k^2) search)
            use_time_metric (bool): if True, maximize travel time (route cost); else maximize distance (sum of edge lengths)
            seed (int|None): random seed for reproducibility

        Returns:
            tuple: (av_route_sumo_edges, av_route_sumo_with_internal)
        """
        try:
            if seed is not None:
                random.seed(seed)

            sumo_net = sumolib.net.readNet(map_path, withInternal=True)
            edges = sumo_net.getEdges()

            # Filter for non-internal edges (regular road edges)
            regular_edges = [e for e in edges if not e.isSpecial()]
            if not regular_edges:
                logger.warning("No regular edges found in network")
                return [], []

            # Find peripheral edges: edges with low in/out degree (≤1)
            peripheral_edges = []
            for e in regular_edges:
                incoming = len(e.getIncoming())
                outgoing = len(e.getOutgoing())
                if incoming <= 1 or outgoing <= 1:
                    peripheral_edges.append(e)

            # Candidate set: prefer peripheral edges, use all if not enough
            candidates = peripheral_edges if len(peripheral_edges) >= 2 else regular_edges
            if len(peripheral_edges) < 2:
                logger.info("Not enough peripheral edges found, using all regular edges")

            # Sample to control complexity
            if len(candidates) > max_candidates:
                candidates = random.sample(candidates, max_candidates)

            # Helper: calculate path length score
            def path_length_score(path_edges):
                # Return None if path is empty
                if not path_edges:
                    return None
                if use_time_metric:
                    # If scoring by time, this won't actually be used (we'll use route_cost directly)
                    # Still provide function to keep interface uniform
                    return sum(e.getLength() for e in path_edges)
                else:
                    return sum(e.getLength() for e in path_edges)

            best_score = -1.0
            best_route_edges = None
            best_cost = None

            # Exhaustive search of candidate pairs (O(k^2)); for k=128 this is typically fast enough
            n = len(candidates)
            for i in range(n):
                for j in range(i + 1, n):
                    src = candidates[i]
                    dst = candidates[j]
                    try:
                        route_edges, route_cost = sumo_net.getShortestPath(src, dst)  # route_cost is often time/weighted cost
                        if not route_edges:
                            # If one direction is not reachable, try reverse
                            route_edges, route_cost = sumo_net.getShortestPath(dst, src)
                            if not route_edges:
                                continue
                        if use_time_metric:
                            score = route_cost if route_cost is not None else -1
                        else:
                            score = path_length_score(route_edges)
                            if score is None:
                                continue
                        if score > best_score:
                            best_score = score
                            best_route_edges = route_edges
                            best_cost = route_cost
                    except Exception as ex:
                        # Some pairs might not be connected or raise errors, skip
                        continue

            # If no connected pairs found, fallback: select the longest single edge
            if not best_route_edges:
                logger.warning("No connected candidate pairs found; falling back to a single long edge")
                single = max(regular_edges, key=lambda e: e.getLength(), default=None)
                av_route_sumo_edges = [single] if single else []
            else:
                av_route_sumo_edges = best_route_edges

            # Try to add internal edges
            if av_route_sumo_edges:
                try:
                    av_route_sumo_with_internal = sumolib.route.addInternal(sumo_net, av_route_sumo_edges)
                except Exception:
                    av_route_sumo_with_internal = av_route_sumo_edges
                    logger.warning("Failed to add internal edges, using original route")
            else:
                av_route_sumo_with_internal = []

            metric_name = "time" if use_time_metric else "distance"
            logger.info(f"Generated longest ({metric_name}) route with {len(av_route_sumo_edges)} edges; score={best_score}, cost={best_cost}")
            return av_route_sumo_edges, av_route_sumo_with_internal

        except Exception as e:
            logger.error(f"Error generating fallback AV route: {e}")
            return [], []

    def generate_flows(self, net_path, end_time=None, period=None, traffic_level=None, vehicle_types = ["vehicle"]):
        """
        Generate traffic flows for a SUMO network.

        Args:
            net_path (str): Path to SUMO network file
            scene_id (int): Scene ID for file naming
            end_time (int, optional): Simulation end time in seconds
            period (float, optional): Period between vehicle insertions
            traffic_level (str, optional): Traffic density level ('low', 'medium', 'high')
                                          Overrides period if provided

        Returns:
            str: Path to the generated routes file
        """
        # Define traffic flow levels if using traffic_level parameter
        flow_levels = {
            'low': 5,   # 5 seconds per vehicle
            'medium': 0.7,   # 0.7 seconds per vehicle
            'high': 0.3    # 0.3 seconds per vehicle
        }
        map_metadata_path = os.path.join(os.path.dirname(net_path), "metadata.json")
        av_route_edges, av_route_points_latlon = self.generate_av_route(net_path, map_metadata_path)
        
        # Override period with traffic level if provided
        if traffic_level and traffic_level in flow_levels:
            period = flow_levels[traffic_level]
            logger.info(f"Using {traffic_level} traffic level (period={period})")
        
        if end_time is None:
            end_time = self.config["traffic"]["end_time"]
        if period is None:
            period = self.config["traffic"]["vehicle_period"]

        logger.info(f"Generating traffic flows for network {net_path} with period {period} and end time {end_time}")
        print(f"Generating traffic flows for network {net_path} with period {period} and end time {end_time}")

        # Create output paths - place in the same directory as the network file
        output_dir = os.path.dirname(net_path)
        
        # Add traffic level to filenames if provided
        # level_suffix = f"_{traffic_level}" if traffic_level else ""
        level_suffix = ""
        
        trips_path = os.path.join(output_dir, f"trips{level_suffix}.trips.xml")
        vehicles_path = os.path.join(output_dir, f"vehicles{level_suffix}.rou.xml")
        pedestrians_path = os.path.join(output_dir, f"pedestrians{level_suffix}.rou.xml")
        bicycles_path = os.path.join(output_dir, f"bicycles{level_suffix}.rou.xml")
        sumo_cfg_path = os.path.join(output_dir, f"simulation{level_suffix}.sumocfg")

        # Make sure the network file exists
        if not os.path.exists(net_path):
            logger.error(f"Network file {net_path} does not exist")
            return None

        try:
            # Find randomTrips.py script
            if self.random_trips_script:
                # Use directly specified script path
                random_trips = self.random_trips_script
                logger.info(f"Using specified randomTrips script: {random_trips}")
            elif self.tools_path:
                random_trips = os.path.join(self.tools_path, "randomTrips.py")
            else:
                # Try to find it in common locations or use directly if in PATH
                random_trips = "randomTrips.py"

            # Define random seeds for reproducibility
            seed = random.randint(1, 10000)

            # Build randomTrips command
            cmd_vehicle = self.get_vehicle_random_trips_command(random_trips, net_path, trips_path, vehicles_path, end_time, period, seed, fringe_factor=10)
            cmd_pedestrian = self.get_pedestrian_random_trips_command(random_trips, net_path, trips_path, pedestrians_path, end_time, period*0.5, seed)
            cmd_bicycle = self.get_bicycle_random_trips_command(random_trips, net_path, trips_path, bicycles_path, end_time, period*10, seed)

            cmd_map = {
                "vehicle": cmd_vehicle,
                "pedestrian": cmd_pedestrian,
                "bicycle": cmd_bicycle
            }
            cmds = [cmd_map[vtype] for vtype in vehicle_types] if vehicle_types else [cmd_vehicle, cmd_pedestrian, cmd_bicycle]
            
            for cmd in cmds:
                if self.allow_fringe:
                    cmd.extend(["--allow-fringe"])
                    logger.info("Using --allow-fringe option")

                # Run randomTrips.py
                logger.info(f"Running command: {' '.join(cmd)}")
                try:
                    process = subprocess.run(cmd, capture_output=True, text=True, check=True)
                    logger.info(f"randomTrips stdout: {process.stdout}")
                except subprocess.CalledProcessError as e:
                    print("\033[91mVehicle random trips generation failed, retrying with fringe_factor=1.0\033[0m")
                    cmd_vehicle = self.get_vehicle_random_trips_command(
                        random_trips, 
                        net_path,
                        trips_path,
                        vehicles_path,
                        end_time,
                        period,
                        seed,
                        fringe_factor=1.0
                    )
                    if self.allow_fringe:
                        cmd_vehicle.extend(["--allow-fringe"])
                    
                    logger.info(f"Running retry command: {' '.join(cmd_vehicle)}")
                    process = subprocess.run(cmd_vehicle, capture_output=True, text=True, check=True)
                    
                    logger.info(f"Retry stdout: {process.stdout}")
                    if process.stderr:
                        logger.warning(f"Retry stderr: {process.stderr}")

            # Add vehicle types to routes file
            # self._add_vehicle_types(vehicles_path)

            # Add AV route to routes file
            self._add_av_route(av_route_edges, vehicles_path)

            # Add bicycle types to routes file
            if "bicycle" in vehicle_types:
                self._add_bicycle_types(bicycles_path)

            path_map = {
                "vehicle": vehicles_path,
                "pedestrian": pedestrians_path,
                "bicycle": bicycles_path
            }

            # Create SUMO configuration file
            self._create_sumo_config(
                net_path, 
                [path_map[vtype] for vtype in vehicle_types], 
                output_dir,
                end_time,
                config_file=sumo_cfg_path
            )

            logger.info(f"Successfully generated traffic flows: {[path_map[vtype] for vtype in vehicle_types]}")
            return str([path_map[vtype] for vtype in vehicle_types])

        except subprocess.CalledProcessError as e:
            logger.error(f"Error running randomTrips: {e}")
            if e.stdout:
                logger.error(f"stdout: {e.stdout}")
            if e.stderr:
                logger.error(f"stderr: {e.stderr}")

            # If randomTrips fails, create a basic routes file
            self._create_fallback_routes(vehicles_path, net_path)
            
            # Create SUMO configuration for fallback routes
            self._create_sumo_config(
                net_path, 
                vehicles_path, 
                output_dir,
                end_time,
                config_file=sumo_cfg_path
            )
            
            return str(vehicles_path)
        except Exception as e:
            logger.error(f"Error generating traffic flows: {e}")
            return None
        
    def _add_av_route(self, av_route_edges, routes_path):
        """Add AV route to the routes file"""
        print(f"Adding AV route to {routes_path}")
        try:
            tree = ET.parse(routes_path)
            root = tree.getroot()
            av_route_edges_str = [edge.getID() for edge in av_route_edges]
            
            # First check if av_route already exists
            existing_route = root.find("route[@id='av_route']")
            if existing_route is not None:
                # replace the existing route with the new one
                root.remove(existing_route)

            # Create new route element for av_route
            route_element = ET.Element("route")
            route_element.set("id", "av_route")
            route_element.set("edges", " ".join(av_route_edges_str))
            
            # Find a good place to insert it - after other routes
            last_route = root.findall("route")[-1] if root.findall("route") else None
            if last_route is not None:
                # Insert after the last route
                last_route_index = list(root).index(last_route)
                root.insert(last_route_index + 1, route_element)
                # Add newline after route element
                last_route.tail = "\n"
            else:
                # No routes found, add after vType definitions or at start
                vtypes = root.findall("vType")
                if vtypes:
                    last_vtype_index = list(root).index(vtypes[-1])
                    root.insert(last_vtype_index + 1, route_element)
                    # Add newline after vType element
                    vtypes[-1].tail = "\n"
                else:
                    # Add at the beginning
                    root.insert(0, route_element)
                    # Add newline after route element
                    route_element.tail = "\n"
            
            print(f"Added new av_route to {routes_path}")
            # Save changes
            tree.write(routes_path)
            print(f"Successfully saved av_route to {routes_path}")
            
        except Exception as e:
            print(f"Error adding AV route to routes file: {e}")
            logger.error(f"Error adding AV route to routes file: {e}")

    def _add_bicycle_types(self, routes_path):
        """Add bicycle types to the routes file"""
        print(f"Adding bicycle types to {routes_path}")
        try:
            tree = ET.parse(routes_path)
            root = tree.getroot()

            # Find the bicycle vType element
            bicycle_type = root.find("vType[@id='bicycle_bicycle']")
            if bicycle_type is not None:
                # Add the two additional attributes
                bicycle_type.set("latAlignment", "right")
                bicycle_type.set("lcStrategic", "0.5")
            # Save changes
            tree.write(routes_path)
            print(f"Successfully added bicycle types to {routes_path}")
        except Exception as e:
            print(f"Error updating bicycle types in routes file: {e}")
            logger.error(f"Error updating bicycle types in routes file: {e}")

            
                

        

    def get_pedestrian_random_trips_command(self, random_trips, net_path, trips_path, routes_path, end_time, period, seed):
        """
        Generate a command for the pedestrian randomTrips.py script.
        """
        cmd = [
            "python3",
            random_trips,
            "-n",
            net_path,
            "-o",
            str(trips_path),
            "-r",
            str(routes_path),
            "-e",
            str(end_time),
            "-p",
            str(period),
            "--random",
            "--seed",
            str(seed),
            "--pedestrians",  # Use pedestrians flag instead of vehicle-class
            "--prefix",
            "pedestrian",
            "--validate",
            "--route-file",
            str(routes_path),
        ]
        return cmd
    
    def get_bicycle_random_trips_command(self, random_trips, net_path, trips_path, routes_path, end_time, period, seed):
        """
        Generate a command for the bicycle randomTrips.py script.
        """
        cmd = [
            "python3",
            random_trips,
            "-n",
            net_path,
            "-o",
            str(trips_path),
            "-r",
            str(routes_path),
            "-e",
            str(end_time),
            "-p",
            str(period),
            "--random",
            "--seed",
            str(seed),
            "--vehicle-class",
            "bicycle",  # Use bicycle vehicle class
            "--vclass",
            "bicycle",
            "--prefix",
            "bicycle",
            "--validate",
            "--route-file",
            str(routes_path),
        ]
        return cmd
    

    def get_vehicle_random_trips_command(self, random_trips, net_path, trips_path, routes_path, end_time, period, seed, fringe_factor=1):
        """
        Generate a command for the vehicle randomTrips.py script.

        Args:
            random_trips (str): Path to randomTrips.py script
            net_path (str): Path to SUMO network file
            trips_path (str): Path to generated trips file
            routes_path (str): Path to generated routes file
            end_time (int): End time of the simulation
            period (float): Period between vehicle insertions
            seed (int): Seed for the random number generator

        Returns:
            list: Command for the vehicle randomTrips.py script
        """
        cmd = [
                "python3",
                random_trips,
                "-n",
                net_path,
                "-o",
                str(trips_path),
                "-r",
                str(routes_path),
                "-e",
                str(end_time),
                "-p",
                str(period),
                "--random",
                "--seed",
                str(seed),
                "--speed-exponent",
                str(5.0),
                "--fringe-speed-exponent",
                str(5.0),
                "--lanes",
                "--length",
                "--vehicle-class",
                "passenger",  # Default vehicle class
                "--vclass",
                "passenger",
                "--prefix",
                "vehicle",
                "--validate",
                "--route-file",
                str(routes_path),
                "--fringe-factor",
                # str(fringe_factor),
                "100",
            ]
        return cmd
    

    def generate_multi_level_flows(self, directory_path):
        """
        Generate traffic flow with multiple levels (low, medium, high) for SUMO maps in a directory.
        
        This method searches for map.net.xml files in the specified directory,
        then generates traffic flows at three different levels using the generate_flows method.
        
        Args:
            directory_path (str): Path to directory containing SUMO network files
        Returns:
            dict: Dictionary mapping network files to generated route files for each level
        """
        logger.info(f"Generating multi-level traffic flows in directory: {directory_path}")
    
        
        # Get simulation settings
        sim_time = 3600  # Default: 1 hour
        
        # Define traffic flow levels
        # flow_levels = ['low', 'medium', 'high']
        flow_levels = ['medium']
        
        # Find map.net.xml files in the directory (recursively)
        network_files = glob.glob(os.path.join(directory_path, "**", "map.net.xml"), recursive=True)
        
        if not network_files:
            logger.warning(f"No map.net.xml files found in {directory_path}")
            return None
        
        # Results dictionary
        results = {}
        
        # Process each network file
        for net_file in tqdm(network_files):
            logger.info(f"Processing network file: {net_file}")
            
            results[net_file] = {}
            
            # Generate flows for each traffic level
            for level in flow_levels:
                logger.info(f"Generating {level} traffic flow")
                
                try:
                    # Use the enhanced generate_flows method with traffic_level parameter
                    routes_file = self.generate_flows(
                        net_path=net_file,
                        end_time=sim_time,
                        traffic_level=level
                    )
                    
                    if routes_file:
                        # Get the other generated files based on naming convention
                        net_dir = os.path.dirname(net_file)
                        trips_file = os.path.join(net_dir, f"trips_{level}.trips.xml")
                        sumo_cfg_file = os.path.join(net_dir, f"simulation{level}.sumocfg")
                        
                        # Store result
                        results[net_file][level] = {
                            'routes': routes_file,
                            'trips': trips_file,
                            'config': sumo_cfg_file
                        }
                        
                        logger.info(f"Successfully generated {level} traffic flow: {routes_file}")
                    else:
                        results[net_file][level] = {
                            'error': "Failed to generate routes file"
                        }
                        
                except Exception as e:
                    logger.error(f"Error generating {level} traffic flow: {e}")
                    results[net_file][level] = {
                        'error': str(e)
                    }
        
        return results

    def _create_sumo_config(self, net_file, routes_files, output_dir, sim_time, config_file=None):
        """Create a SUMO configuration file for the simulation"""
        if config_file is None:
            cfg_path = os.path.join(output_dir, "simulation.sumocfg")
        else:
            cfg_path = config_file
        
        try:
            # Create XML structure
            root = ET.Element("configuration")
            
            # Input section
            input_section = ET.SubElement(root, "input")
            ET.SubElement(input_section, "net-file", {"value": os.path.basename(net_file)})
            ET.SubElement(input_section, "route-files", {"value": ",".join([os.path.basename(route_file) for route_file in routes_files])})
            ET.SubElement(input_section, "step-length", {"value": "0.1"})
            
            # Time section
            time_section = ET.SubElement(root, "time")
            ET.SubElement(time_section, "begin", {"value": "0"})
            ET.SubElement(time_section, "end", {"value": str(sim_time)})
            
            # # Random number section
            # random_section = ET.SubElement(root, "random_number")
            # ET.SubElement(random_section, "random", {"value": "true"})
            
            # Processing section
            processing = ET.SubElement(root, "processing")
            ET.SubElement(processing, "lateral-resolution", {"value": "0.5"})
            ET.SubElement(processing, "time-to-teleport", {"value": "-1"})
            ET.SubElement(processing, "collision.action", {"value": "warn"})
            ET.SubElement(processing, "collision.check-junctions", {"value": "true"})
            ET.SubElement(processing, "collision.mingap-factor", {"value": "0"})
            
            # Report section
            report = ET.SubElement(root, "report")
            ET.SubElement(report, "verbose", {"value": "true"})
            ET.SubElement(report, "no-step-log", {"value": "true"})
            
            # Write to file
            tree = ET.ElementTree(root)
            ET.indent(tree, space="  ")
            tree.write(cfg_path, encoding="utf-8", xml_declaration=True)
            
            logger.info(f"Created SUMO configuration file: {cfg_path}")
            return cfg_path
            
        except Exception as e:
            logger.error(f"Error creating SUMO configuration: {e}")
            return None

    def _add_vehicle_types(self, routes_path):
        """Add vehicle types to the routes file"""
        try:
            tree = ET.parse(routes_path)
            root = tree.getroot()

            # Check if vehicle types already exist
            has_types = any(child.tag == "vType" for child in root)

            if not has_types:
                # Add car type
                car_type = ET.Element(
                    "vType",
                    {
                        "id": "car",
                        "accel": "2.6",
                        "decel": "4.5",
                        "sigma": "0.5",
                        "length": "5.0",
                        "minGap": "2.5",
                        "maxSpeed": "55.55",  # 200 km/h
                        "color": "1,0,0",
                        "guiShape": "passenger",
                    },
                )

                # Add motorcycle type
                motorcycle_type = ET.Element(
                    "vType",
                    {
                        "id": "motorcycle",
                        "accel": "4.0",
                        "decel": "7.0",
                        "sigma": "0.5",
                        "length": "2.0",
                        "minGap": "1.0",
                        "maxSpeed": "50.0",
                        "color": "0,0,1",
                        "guiShape": "motorcycle",
                    },
                )

                # Add bus type
                bus_type = ET.Element(
                    "vType",
                    {
                        "id": "bus",
                        "accel": "1.2",
                        "decel": "3.0",
                        "sigma": "0.5",
                        "length": "12.0",
                        "minGap": "2.5",
                        "maxSpeed": "30.0",
                        "color": "0,1,0",
                        "guiShape": "bus",
                    },
                )

                bicycle_type = ET.Element(
                    "vType",
                    {
                        "id": "bike",
                        "vClass": "bicycle", 
                        "latAlignment": "right",
                        "lcStrategic": "0.5"
                    },
                )

                # Insert at the beginning of the file
                root.insert(0, car_type)
                root.insert(1, motorcycle_type)
                root.insert(2, bus_type)
                root.insert(3, bicycle_type)
                # Assign vehicle types to vehicles
                for vehicle in root.findall("vehicle"):
                    r = random.random()
                    if r < 0.8:  # 80% cars
                        vehicle.set("type", "car")
                    elif r < 0.95:  # 15% motorcycles
                        vehicle.set("type", "motorcycle")
                    else:  # 5% buses
                        vehicle.set("type", "bus")

                # Write back to file
                tree.write(routes_path)
                logger.info(f"Added vehicle types to {routes_path}")

        except Exception as e:
            logger.error(f"Error adding vehicle types: {e}")

    def _create_fallback_routes(self, routes_path, net_path):
        """Create a fallback routes file when randomTrips fails"""
        try:
            logger.info("Creating fallback routes file")

            # Parse the network file to find edges
            net_tree = ET.parse(net_path)
            net_root = net_tree.getroot()

            # Find all edges
            edges = []
            for edge in net_root.findall(".//edge"):
                # Skip internal edges
                if edge.get("id") and not edge.get("id").startswith(":"):
                    edges.append(edge.get("id"))

            if not edges:
                logger.error("No edges found in the network")
                return

            # Create basic routes
            root = ET.Element("routes")

            # Add vehicle types
            car_type = ET.SubElement(
                root,
                "vType",
                {
                    "id": "car",
                    "accel": "2.6",
                    "decel": "4.5",
                    "sigma": "0.5",
                    "length": "5.0",
                    "minGap": "2.5",
                    "maxSpeed": "55.55",
                    "color": "1,0,0",
                    "guiShape": "passenger",
                },
            )

            # Find valid routes by looking at edge connections
            valid_routes = []

            # Extract connections between edges
            connections = {}
            for connection in net_root.findall(".//connection"):
                from_edge = connection.get("from")
                to_edge = connection.get("to")
                if (
                    from_edge
                    and to_edge
                    and not from_edge.startswith(":")
                    and not to_edge.startswith(":")
                ):
                    if from_edge not in connections:
                        connections[from_edge] = []
                    connections[from_edge].append(to_edge)

            # Create a few simple routes based on connected edges
            route_count = 0
            visited = set()

            # Try to find paths between edges
            for start_edge in edges:
                if route_count >= 5:  # Limit to 5 routes
                    break

                if start_edge in visited:
                    continue

                # Simple DFS to find a path
                path = [start_edge]
                current_edge = start_edge
                path_length = 1

                while path_length < 5:  # Limit path length
                    if current_edge in connections and connections[current_edge]:
                        next_edge = connections[current_edge][
                            0
                        ]  # Take first connection
                        path.append(next_edge)
                        current_edge = next_edge
                        path_length += 1
                    else:
                        break

                if path_length > 1:
                    # Valid path found
                    route_id = f"route_{route_count}"
                    ET.SubElement(
                        root, "route", {"id": route_id, "edges": " ".join(path)}
                    )
                    valid_routes.append(route_id)
                    route_count += 1
                    visited.add(start_edge)

            # If no valid connected routes found, try a simpler approach
            if not valid_routes:
                logger.warning(
                    "No connected paths found, attempting to generate simpler routes"
                )

                # Try to get connection info from SUMO directly
                try:
                    import sumolib

                    net = sumolib.net.readNet(net_path)

                    # Get a few routes
                    for i in range(min(5, len(edges))):
                        edge_obj = net.getEdge(edges[i])
                        out_edges = edge_obj.getOutgoing()

                        if out_edges:
                            # Create a simple route from this edge to first outgoing edge
                            out_edge = list(out_edges.keys())[0]
                            route_id = f"route_{i}"
                            edge_path = [edges[i], out_edge.getID()]

                            ET.SubElement(
                                root,
                                "route",
                                {"id": route_id, "edges": " ".join(edge_path)},
                            )
                            valid_routes.append(route_id)

                except ImportError:
                    logger.warning("Cannot import sumolib - using single-edge routes")

                    # Last resort: create routes on single edges
                    for i in range(min(5, len(edges))):
                        route_id = f"route_{i}"
                        ET.SubElement(
                            root, "route", {"id": route_id, "edges": edges[i]}
                        )
                        valid_routes.append(route_id)

            # Add a few vehicles for each valid route
            vehicle_count = 0
            for route_id in valid_routes:
                for i in range(3):  # 3 vehicles per route
                    vehicle_id = f"vehicle_{vehicle_count}"
                    ET.SubElement(
                        root,
                        "vehicle",
                        {
                            "id": vehicle_id,
                            "type": "car",
                            "route": route_id,
                            "depart": str(vehicle_count * 5),
                        },
                    )
                    vehicle_count += 1

            # Write to file
            tree = ET.ElementTree(root)
            tree.write(routes_path)
            logger.info(
                f"Created fallback routes file: {routes_path} with {len(valid_routes)} routes"
            )

        except Exception as e:
            logger.error(f"Error creating fallback routes: {e}")
            # Create an absolutely minimal file with no vehicles as last resort
            try:
                minimal_root = ET.Element("routes")
                ET.SubElement(
                    minimal_root,
                    "vType",
                    {
                        "id": "car",
                        "accel": "2.6",
                        "decel": "4.5",
                        "sigma": "0.5",
                        "length": "5.0",
                        "minGap": "2.5",
                        "maxSpeed": "55.55",
                        "color": "1,0,0",
                    },
                )
                minimal_tree = ET.ElementTree(minimal_root)
                minimal_tree.write(routes_path)
                logger.warning(f"Created minimal routes file with no vehicles")
            except:
                logger.error("Failed to create even a minimal routes file")


def process_network(net_path, traffic_level="medium"):
    try:
        # Extract scene ID from the path
        scene_dir = os.path.dirname(net_path)
        scene_name = os.path.basename(scene_dir)
        
        # Create new generator instance for each process
        generator = TrafficFlowGenerator("config/config.yaml")
        
        # Generate flows for this network
        print(f"Generating flows for {net_path}")
        routes_path = generator.generate_flows(net_path, traffic_level=traffic_level, vehicle_types=["vehicle"])
        
        if routes_path:
            print(f"Successfully generated flows at {routes_path}")
            return True
        else:
            print(f"Failed to generate flows for {net_path}")
            return False
            
    except Exception as e:
        print(f"Error processing {net_path}: {e}")
        return False

def test_generate_flows_for_all_networks(output_dir="test_output_texas_1km_bbox", multi_process=False, traffic_level="medium"):
    base_dir = Path(output_dir)
    
    # Find all .net.xml files recursively
    try:
        net_files = sorted(glob.glob(os.path.join(base_dir, "**/*.net.xml"), recursive=True), key=lambda x: int(os.path.basename(os.path.dirname(x))))
    except Exception as e:
        print("Net files cannot be sorted using int, using string sort instead")
        net_files = sorted(glob.glob(os.path.join(base_dir, "**/*.net.xml"), recursive=True))
    if not net_files:
        print(f"No .net.xml files found in {base_dir}")
        return
        
    print(f"Found {len(net_files)} .net.xml files to process")
    
    # Process each net file
    from multiprocessing import Pool, cpu_count

    if multi_process:
        # Use number of CPUs for parallel processing
        num_processes = cpu_count()
        
        # Import functools for partial function
        from functools import partial
        
        # Create a partial function with traffic_level parameter
        process_network_with_traffic = partial(process_network, traffic_level=traffic_level)

        with Pool(processes=num_processes) as pool:
            results = list(tqdm(pool.imap(process_network_with_traffic, net_files), 
                              total=len(net_files),
                              desc="Processing networks",
                              unit="network"))
        
        # Print summary for multiprocessing
        successful = sum(results)
        print(f"\nProcessed {len(net_files)} networks in parallel:")
        print(f"Successful: {successful}")
        print(f"Failed: {len(net_files) - successful}")
    else:
        # Single process method
        successful = 0
        for net_path in net_files:
            result = process_network(net_path, traffic_level=traffic_level)
            if result:
                successful += 1
        
        # Print summary for single process
        print(f"\nProcessed {len(net_files)} networks sequentially:")
        print(f"Successful: {successful}")
        print(f"Failed: {len(net_files) - successful}")