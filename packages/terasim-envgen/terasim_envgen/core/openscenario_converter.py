#!/usr/bin/env python3
"""
openscenario_converter.py - Convert OpenDRIVE maps and SUMO FCD trajectories to OpenSCENARIO format
"""

import os
import sys
import logging
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
import uuid
import datetime
import math
import random
from scenariogeneration import xosc, prettyprint

logger = logging.getLogger(__name__)


class OpenScenarioConverter:
    def __init__(self):
        """Initialize the OpenSCENARIO converter"""
        # Default parameters
        self.max_waypoints = 500  # Limit waypoints per vehicle for testing
        self.max_vehicles = 5  # Limit number of vehicles for testing

    def convert(
        self,
        map_path,
        fcd_path,
        output_path,
        debug_mode=False,
        start_time=None,
        end_time=None,
    ):
        """
        Convert OpenDRIVE map and SUMO FCD trajectories to OpenSCENARIO format

        Args:
            map_path (str): Path to OpenDRIVE map file (.xodr)
            fcd_path (str): Path to SUMO FCD trajectory file (.fcd.xml)
            output_path (str): Path to output OpenSCENARIO file (.xml)
            debug_mode (bool): If True, limit the size of the output for testing
            start_time (float): Start time in seconds for trajectory data (None=beginning)
            end_time (float): End time in seconds for trajectory data (None=end)

        Returns:
            bool: True if conversion was successful, False otherwise
        """
        logger.info(
            f"Converting OpenDRIVE map {map_path} and FCD trajectories {fcd_path} to OpenSCENARIO"
        )

        if start_time is not None and end_time is not None:
            logger.info(f"Extracting time window from {start_time}s to {end_time}s")

        try:
            # Parse FCD file to extract trajectory data
            vehicle_trajectories = self._parse_fcd_file(
                fcd_path, debug_mode, start_time, end_time
            )

            if not vehicle_trajectories:
                logger.error("No valid trajectories found in FCD file")
                return False

            # Create OpenSCENARIO document using scenariogeneration
            scenario = self._create_openscenario_doc(
                map_path, vehicle_trajectories
            )

            # Write to output file
            scenario.write_xml(output_path, prettyprint=True)

            # Print summary information
            self._print_summary(vehicle_trajectories, start_time, end_time, output_path)

            logger.info(f"Successfully created OpenSCENARIO file at {output_path}")

            return True

        except Exception as e:
            logger.error(f"Error converting to OpenSCENARIO: {e}")
            return False

    def _print_summary(self, vehicle_trajectories, start_time, end_time, output_path):
        """
        Print summary information about the converted scenario

        Args:
            vehicle_trajectories (dict): Dictionary of vehicle trajectories
            start_time (float): Start time used for conversion
            end_time (float): End time used for conversion
            output_path (str): Path to the generated output file
        """
        # Calculate statistics
        total_vehicles = len(vehicle_trajectories)

        if total_vehicles == 0:
            logger.warning("No vehicles in the scenario!")
            return

        total_waypoints = sum(
            len(data["trajectory"]) for data in vehicle_trajectories.values()
        )
        avg_waypoints = total_waypoints / total_vehicles

        # Find min and max time values
        all_times = []
        for vehicle_id, data in vehicle_trajectories.items():
            if data["trajectory"]:
                all_times.extend([wp["time"] for wp in data["trajectory"]])

        min_time = min(all_times) if all_times else 0
        max_time = max(all_times) if all_times else 0
        duration = max_time - min_time

        # Find min and max positions
        x_coords = []
        y_coords = []
        for vehicle_id, data in vehicle_trajectories.items():
            for wp in data["trajectory"]:
                x_coords.append(wp["x"])
                y_coords.append(wp["y"])

        x_min = min(x_coords) if x_coords else 0
        x_max = max(x_coords) if x_coords else 0
        y_min = min(y_coords) if y_coords else 0
        y_max = max(y_coords) if y_coords else 0

        scenario_width = x_max - x_min
        scenario_height = y_max - y_min

        # Calculate file size
        file_size_mb = os.path.getsize(output_path) / (1024 * 1024)

        # Print summary
        logger.info("=" * 50)
        logger.info("OpenSCENARIO Conversion Summary")
        logger.info("=" * 50)
        logger.info(
            f"Time window: {min_time:.2f}s - {max_time:.2f}s (duration: {duration:.2f}s)"
        )
        if start_time is not None:
            logger.info(f"Original time window: {start_time:.2f}s - {end_time:.2f}s")
        logger.info(f"Total vehicles: {total_vehicles}")
        logger.info(f"Total waypoints: {total_waypoints}")
        logger.info(f"Average waypoints per vehicle: {avg_waypoints:.2f}")
        logger.info(
            f"Scenario dimensions: {scenario_width:.2f}m x {scenario_height:.2f}m"
        )
        logger.info(f"Output file size: {file_size_mb:.2f} MB")
        logger.info("=" * 50)

    def _parse_fcd_file(
        self, fcd_path, debug_mode=False, start_time=None, end_time=None
    ):
        """
        Parse SUMO FCD file to extract vehicle trajectories

        Args:
            fcd_path (str): Path to SUMO FCD trajectory file
            debug_mode (bool): If True, limit the number of vehicles and waypoints
            start_time (float): Start time in seconds for trajectory data
            end_time (float): End time in seconds for trajectory data

        Returns:
            dict: Dictionary of vehicle trajectories
        """
        logger.info(f"Parsing FCD file: {fcd_path}")

        vehicle_trajectories = {}
        vehicle_count = 0

        # Find random time window if not specified
        if start_time is None and end_time is None and debug_mode:
            # Try to find a good window with traffic by sampling some timesteps
            logger.info("Finding suitable time window with traffic...")
            try:
                # Quick scan to find non-empty timesteps
                tree = ET.parse(fcd_path)
                root = tree.getroot()
                timesteps = root.findall("timestep")

                # Get all timestep values
                times = [
                    float(ts.get("time"))
                    for ts in timesteps
                    if len(ts.findall("vehicle")) > 0
                ]

                if times:
                    # Choose a random time that has some vehicles
                    sample_time = random.choice(times)
                    start_time = max(0, sample_time - 0.5)  # 0.5s buffer before
                    end_time = start_time + 15.0  # 15 second window
                    logger.info(f"Selected time window: {start_time}s - {end_time}s")
                else:
                    logger.warning("No timesteps with vehicles found")
            except Exception as e:
                logger.warning(
                    f"Error scanning for time window: {e}, will use entire file"
                )

        try:
            tree = ET.parse(fcd_path)
            root = tree.getroot()

            # Extract timestep information
            for timestep in root.findall("timestep"):
                time = float(timestep.get("time"))

                # Skip if outside requested time window
                if (start_time is not None and time < start_time) or (
                    end_time is not None and time > end_time
                ):
                    continue

                # Extract vehicle information at this timestep
                for vehicle in timestep.findall("vehicle"):
                    vehicle_id = vehicle.get("id")

                    # Skip if we've reached the maximum number of vehicles in debug mode
                    if (
                        debug_mode
                        and vehicle_id not in vehicle_trajectories
                        and vehicle_count >= self.max_vehicles
                    ):
                        continue

                    if vehicle_id not in vehicle_trajectories:
                        if debug_mode and vehicle_count >= self.max_vehicles:
                            continue
                        vehicle_trajectories[vehicle_id] = {
                            "type": vehicle.get("type", "car"),
                            "trajectory": [],
                        }
                        vehicle_count += 1

                    # Skip if we've reached the maximum number of waypoints for this vehicle in debug mode
                    if (
                        debug_mode
                        and len(vehicle_trajectories[vehicle_id]["trajectory"])
                        >= self.max_waypoints
                    ):
                        continue

                    # Extract position, velocity, and heading
                    x = float(vehicle.get("x", 0))
                    y = float(vehicle.get("y", 0))
                    speed = float(vehicle.get("speed", 0))
                    angle = float(vehicle.get("angle", 0))

                    # Convert SUMO angle to rad (SUMO: 0=north, 90=east, OpenSCENARIO: 0=east, pi/2=north)
                    heading_rad = math.radians((90 - angle) % 360)
                    if vehicle_id == "vehicle15":
                        print(heading_rad)

                    # Store waypoint data
                    vehicle_trajectories[vehicle_id]["trajectory"].append(
                        {
                            "time": time,
                            "x": x,
                            "y": y,
                            "z": float(vehicle.get("z", 0)),
                            "speed": speed,
                            "heading": heading_rad,
                        }
                    )

            # Remove vehicles with too few waypoints for a meaningful trajectory
            to_remove = []
            for vehicle_id, data in vehicle_trajectories.items():
                if len(data["trajectory"]) < 2:
                    to_remove.append(vehicle_id)

            for vehicle_id in to_remove:
                del vehicle_trajectories[vehicle_id]

            logger.info(
                f"Extracted trajectories for {len(vehicle_trajectories)} vehicles"
            )

            # Limit trajectory size for debugging if needed
            if debug_mode:
                for vehicle_id in vehicle_trajectories:
                    if (
                        len(vehicle_trajectories[vehicle_id]["trajectory"])
                        > self.max_waypoints
                    ):
                        vehicle_trajectories[vehicle_id]["trajectory"] = (
                            vehicle_trajectories[vehicle_id]["trajectory"][
                                : self.max_waypoints
                            ]
                        )

            # Normalize trajectory times if using a time window
            if start_time is not None:
                for vehicle_id in vehicle_trajectories:
                    for waypoint in vehicle_trajectories[vehicle_id]["trajectory"]:
                        # Adjust time so first waypoint starts at 0
                        waypoint["time"] = waypoint["time"] - start_time

            return vehicle_trajectories

        except Exception as e:
            logger.error(f"Error parsing FCD file: {e}")
            return {}

    def _create_openscenario_doc(self, map_path, vehicle_trajectories):
        """
        Create OpenSCENARIO XML document using scenariogeneration

        Args:
            map_path (str): Path to OpenDRIVE map file
            vehicle_trajectories (dict): Dictionary of vehicle trajectories

        Returns:
            xosc.Scenario: OpenSCENARIO scenario object
        """

        parameters = xosc.ParameterDeclarations()
        catalog = xosc.Catalog()

        # Create a road network
        road = xosc.RoadNetwork(
            roadfile=os.path.basename(map_path),
            scenegraph=''
        )
        # Create entities
        entities = xosc.Entities()

        # Add vehicle entities
        for vehicle_id, vehicle_data in vehicle_trajectories.items():
            self._add_vehicle_entity(entities, vehicle_id, vehicle_data)

        # Create an init action
        init = xosc.Init()

        # Add initial positions for all vehicles
        for vehicle_id, vehicle_data in vehicle_trajectories.items():
            if vehicle_data["trajectory"]:
                self._add_init_action(init, vehicle_id, vehicle_data["trajectory"][1])

        # Find the max time across all trajectories
        max_time = 0
        for vehicle_data in vehicle_trajectories.values():
            if vehicle_data["trajectory"]:
                last_waypoint = vehicle_data["trajectory"][-1]
                max_time = max(max_time, last_waypoint["time"])

        # Create stop trigger
        stop_trigger = xosc.ValueTrigger(
            'stop_trigger', 
            0, 
            xosc.ConditionEdge.rising, 
            xosc.SimulationTimeCondition(max_time + 0.5, xosc.Rule.greaterThan),
            'stop'
        )

        # Create storyboard with init and stop trigger
        storyboard = xosc.StoryBoard(init, stop_trigger)

        # Create a story
        story = xosc.Story('TrafficStory')
         
        # Add trajectory actions for each vehicle
        for vehicle_id, vehicle_data in vehicle_trajectories.items():
            self._add_trajectory_action(story, vehicle_id, vehicle_data["trajectory"])
            
        storyboard.add_story(story)
                        
        # Create the scenario
        scenario = xosc.Scenario(
            name='ConvertedScenario',
            author='TeraSim-Agent',
            parameters=parameters,
            entities=entities,
            storyboard=storyboard,
            roadnetwork=road,
            catalog=catalog,
            osc_minor_version=0
        )

        return scenario

    def _add_vehicle_entity(self, entities, vehicle_id, vehicle_data):
        """Add a vehicle entity to the Entities section"""
        # Create bounding box
        bb = xosc.BoundingBox(2.0, 5.0, 1.8, 2.0, 0.0, 0.9)
        
        # Create axles
        fa = xosc.Axle(0.5, 0.8, 1.8, 1.93, 0.4)
        ra = xosc.Axle(0.5, 0.8, 1.8, 1.93, 0.4)
        
        # Create vehicle
        vehicle = xosc.Vehicle(
            f'vehicle_{vehicle_id}', 
            xosc.VehicleCategory.car,  # Changed from VehicleType to VehicleCategory
            bb, 
            fa, 
            ra, 
            69.444,  # maxSpeed in m/s (250 km/h)
            10,       # maxAcceleration
            10        # maxDeceleration
        )
        
        # Add to entities
        entities.add_scenario_object(f'vehicle_{vehicle_id}', vehicle)

    def _add_init_action(self, init, vehicle_id, initial_waypoint):
        """Add an initialization action for a vehicle"""
        # Add teleport action
        init.add_init_action(
            f'vehicle_{vehicle_id}', 
            xosc.TeleportAction(
                xosc.WorldPosition(
                    initial_waypoint["x"], 
                    initial_waypoint["y"], 
                    initial_waypoint["z"], 
                )
            )
        )
        
        # Add speed action
        init.add_init_action(
            f'vehicle_{vehicle_id}', 
            xosc.AbsoluteSpeedAction(
                initial_waypoint["speed"], 
                xosc.TransitionDynamics(
                    xosc.DynamicsShapes.step, 
                    xosc.DynamicsDimension.time, 
                    0
                )
            )
        )

    def _add_trajectory_action(self, story, vehicle_id, trajectory):
        """Add a trajectory action for a vehicle"""
        if len(trajectory) < 2:
            logger.warning(
                f"Not enough waypoints for vehicle {vehicle_id}, skipping trajectory"
            )
            return
            
        # Prepare time and positions arrays for Polyline
        times = []
        positions = []
        
        # Process waypoints to extract time and position data
        for i, waypoint in enumerate(trajectory):
            
            calculated_heading = waypoint["heading"]
            
            # Add time and position to arrays
            times.append(waypoint["time"])
            positions.append(
                xosc.WorldPosition(
                    waypoint["x"], 
                    waypoint["y"], 
                    waypoint["z"], 
                    calculated_heading
                )
            )
        
        # Create trajectory shape with prepared arrays
        shape = xosc.Polyline(times, positions)

        
        # Create trajectory
        traj = xosc.Trajectory(f'Trajectory_{vehicle_id}', False)
        traj.add_shape(shape)
        
        # Create trajectory action
        traj_action = xosc.FollowTrajectoryAction(
            traj,
            xosc.FollowingMode.position
        )
        
        # Create event
        event = xosc.Event(
            f'TrajectoryEvent_{vehicle_id}',
            xosc.Priority.overwrite
        )
        event.add_action(f'TrajectoryAction_{vehicle_id}', traj_action)
        
        # Create start trigger
        event.add_trigger(
            xosc.ValueTrigger(
                f'StartTime_{vehicle_id}',
                0,
                xosc.ConditionEdge.rising,
                xosc.SimulationTimeCondition(trajectory[0]["time"], xosc.Rule.greaterThan)
            )
        )
        
        # Create maneuver
        maneuver = xosc.Maneuver(f'TrajectoryManeuver_{vehicle_id}')
        maneuver.add_event(event)
        
        # Create maneuver group
        maneuver_group = xosc.ManeuverGroup(
            f'TrajectorySequence_{vehicle_id}',
            1,
            False
        )
        maneuver_group.add_actor(f'vehicle_{vehicle_id}')
        maneuver_group.add_maneuver(maneuver)
        
        # Create act
        act = xosc.Act(f'TrajectoryAct_{vehicle_id}')
        act.add_maneuver_group(maneuver_group)
        
        # Add act to story
        story.add_act(act)


def main():
    """Main function for command-line interface"""
    parser = argparse.ArgumentParser(
        description="Convert OpenDRIVE maps and SUMO FCD trajectories to OpenSCENARIO"
    )
    parser.add_argument(
        "--map_path", required=True, help="Path to OpenDRIVE map file (.xodr)"
    )
    parser.add_argument(
        "--fcd_path", required=True, help="Path to SUMO FCD trajectory file (.fcd.xml)"
    )
    parser.add_argument(
        "--output_path", required=True, help="Path to output OpenSCENARIO file (.xml)"
    )
    parser.add_argument("--log_level", default="INFO", help="Logging level")
    parser.add_argument(
        "--debug", action="store_true", help="Debug mode - limit output size"
    )
    parser.add_argument(
        "--start_time", type=float, help="Start time in seconds for trajectory data"
    )
    parser.add_argument(
        "--end_time", type=float, help="End time in seconds for trajectory data"
    )
    parser.add_argument(
        "--time_window",
        type=float,
        default=15.0,
        help="Duration of time window in seconds (used if only start_time is provided)",
    )

    args = parser.parse_args()

    # Configure logging
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    # If only start_time is provided, calculate end_time
    start_time = args.start_time
    end_time = args.end_time

    if start_time is not None and end_time is None:
        end_time = start_time + args.time_window
        logger.info(
            f"Setting end_time to start_time + {args.time_window}s = {end_time}s"
        )

    # Create converter and run conversion
    converter = OpenScenarioConverter()
    success = converter.convert(
        args.map_path, args.fcd_path, args.output_path, args.debug, start_time, end_time
    )

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())