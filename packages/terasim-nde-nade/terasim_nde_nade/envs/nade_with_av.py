from addict import Dict
import copy
from loguru import logger
import numpy as np
import random

from terasim.overlay import traci, profile
import terasim.utils as utils
from terasim.params import AgentType

from .nade import NADE

from ..utils import (
    apply_collision_avoidance,
    calclulate_distance_from_centered_agent,
    CommandType,
    NDECommand,
    get_collision_type_and_prob,
    is_car_following,
    update_control_cmds_from_predicted_trajectory,
)

AV_ID = "AV"
AV_ROUTE_ID = "av_route"

class NADEWithAV(NADE):
    def __init__(self, av_cfg, av_debug_control=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.av_cfg = av_cfg
        self.cache_radius = 100 if "cache_radius" not in av_cfg else av_cfg.cache_radius
        self.control_radius = 50 if "control_radius" not in av_cfg else av_cfg.control_radius
        self.excluded_agent_set = set([AV_ID])
        self.insert_bv = False
        # AV control state management
        self.av_control_mode = "pending"  # "pending", "sumo", "external"
        self.av_warmup_enabled = False
        self.av_warmup_config = None
        self.av_warmup_start_time = None
        self.av_warmup_completed = False
        # Set AV control mode: "sumo" for SUMO-managed warmup, "external" for immediate external control
        self.av_debug_control = av_debug_control
        # If "sumo", AV is initially SUMO-controlled and switched to external after warmup.
        # If "external", AV is controlled externally from the start, with speed and lane change modes set for external control.
        
        # Parse warmup configuration
        if hasattr(av_cfg, 'warmup_control'):
            warmup_control = av_cfg.warmup_control
            if isinstance(warmup_control, dict) and warmup_control.get('enabled', False):
                self.av_warmup_enabled = True
                self.av_warmup_config = warmup_control
                logger.info(f"AV warmup enabled with trigger type: {self.av_warmup_config.get('trigger_type')}")
            elif hasattr(warmup_control, 'enabled') and warmup_control.enabled:
                self.av_warmup_enabled = True
                self.av_warmup_config = warmup_control
                logger.info(f"AV warmup enabled with trigger type: {self.av_warmup_config.get('trigger_type')}")

    def on_start(self, ctx):
        """Initialize the surrogate model and add AV to env with optional warmup.
        
        Args:
            ctx (dict): Context dictionary.
        """
        # Step 1: Execute parent class initialization (includes first sumo_warmup)
        super().on_start(ctx)
        
        # Step 2: Add AV to simulation
        self.add_av_safe()
        
        # Step 3: If AV warmup is enabled, execute AV warmup phase
        if self.av_warmup_enabled:
            logger.info("Starting AV warmup phase...")
            self.execute_av_warmup()
            logger.info("AV warmup completed, external control activated")
        
        return True

    def add_av_unsafe(self, edge_id="EG_35_1_14", lane_id=None, position=0.0, speed=0.0):
        """Add a AV to the simulation.

        Args:
            edge_id (str): Edge ID where the AV is added.
            lane_id (str): Lane ID where the AV is added.
            position (float): Position of the AV.
            speed (float): Speed of the AV.
        """
        if lane_id is None:
            lane_id = edge_id + "_0"

        if hasattr(self.av_cfg, "type"):
            av_type = self.av_cfg.type
        else:
            av_type = "DEFAULT_VEHTYPE"

        self.add_vehicle(
            veh_id=AV_ID,
            route_id=AV_ROUTE_ID,
            lane="best",
            lane_id=lane_id,
            position=position,
            speed=speed,
            type_id=av_type,
        )
        # set the AV with white color
        traci.vehicle.setColor(AV_ID, (255, 255, 255, 255))

        traci.vehicle.subscribeContext(
            AV_ID,
            traci.constants.CMD_GET_VEHICLE_VARIABLE,
            self.cache_radius,
            [traci.constants.VAR_DISTANCE],
        )
        if not self.av_debug_control:
            # Set SpeedMode and LaneChangeMode to 0 for external control
            traci.vehicle.setSpeedMode(AV_ID, 0)
            traci.vehicle.setLaneChangeMode(AV_ID, 0)
        traci.vehicle.setSpeed(AV_ID, speed)
        
        # Set control mode to external
        self.av_control_mode = "external"

    def add_av_safe(self):
        """Add a AV to the simulation safely.
        """
        # Handle coordinate conversion based on initial_position_type
        if hasattr(self.av_cfg, "initial_position_type"):
            try:
                if self.av_cfg.initial_position_type == "xy":
                    x = float(self.av_cfg.initial_xy.x)
                    y = float(self.av_cfg.initial_xy.y)
                    # Convert XY to road position
                    edge_id, position, lane_index = traci.simulation.convertRoad(x, y)
                    if edge_id:
                        self.av_cfg.initial_lane_index = lane_index
                        self.av_cfg.initial_lane_position = position
                        logger.info(f"Converted XY ({x}, {y}) to edge {edge_id}, lane {lane_index}, position {position}")
                    else:
                        logger.warning(f"Failed to convert XY ({x}, {y}) to road position")
                        
                elif self.av_cfg.initial_position_type == "latlon":
                    lat = float(self.av_cfg.initial_latlon.lat)
                    lon = float(self.av_cfg.initial_latlon.lon)
                    # Convert lat/lon to XY
                    x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
                    # Convert XY to road position
                    edge_id, position, lane_index = traci.simulation.convertRoad(x, y)
                    if edge_id:
                        self.av_cfg.initial_lane_index = lane_index
                        self.av_cfg.initial_lane_position = position
                        logger.info(f"Converted lat/lon ({lat}, {lon}) to edge {edge_id}, lane {lane_index}, position {position}")
                    else:
                        logger.warning(f"Failed to convert lat/lon ({lat}, {lon}) to road position")
            except Exception as e:
                logger.error(f"Error during coordinate conversion: {e}")
                # Continue with default behavior
                
        # handle the route of av: first check if there are any existing routes with the same name
        if AV_ROUTE_ID in traci.route.getIDList():
            av_route = traci.route.getEdges(AV_ROUTE_ID)
        else:
            # add the av_route to the SUMO simulation
            assert hasattr(self.av_cfg, "route"), "AV route is not defined in the config file"
            # check if AV_ROUTE_ID is in the SUMO simulation
            if AV_ROUTE_ID in traci.route.getIDList():
                av_route = traci.route.getEdges(AV_ROUTE_ID)
                av_route = [edge.getID() for edge in av_route]
            else:
                av_route = self.av_cfg.route
                traci.route.add(AV_ROUTE_ID, av_route)
        edge_id = av_route[0]
        lanes = traci.edge.getLaneNumber(edge_id)
        max_attempts = 10
        if hasattr(self.av_cfg, "type"):
            av_type = self.av_cfg.type
        else:
            av_type = "DEFAULT_VEHTYPE"
        if hasattr(self.av_cfg, "clearance_distance"):
            min_safe_distance = self.av_cfg.clearance_distance + traci.vehicletype.getLength(av_type)  # Minimum safe distance from other vehicles
        else:
            min_safe_distance = 50 + traci.vehicletype.getLength(av_type)  # Minimum safe distance from other vehicles

        if hasattr(self.av_cfg, "initial_lane_index"):
            possible_lane_indexes = [int(self.av_cfg.initial_lane_index)]
        else:
            possible_lane_indexes = list(range(0, lanes - 1))

        for attempt in range(max_attempts):
            if not len(possible_lane_indexes):
                break
            lane = random.choice(possible_lane_indexes)
            lane_id = f"{edge_id}_{lane}"
            lane_length = traci.lane.getLength(lane_id)
            if lane_length < min_safe_distance:
                break

            if hasattr(self.av_cfg, "initial_lane_position"):
                position = float(self.av_cfg.initial_lane_position)
            else:
                position = random.uniform(0, lane_length-min_safe_distance)

            if self.is_position_safe(lane_id, position, min_safe_distance):
                if hasattr(self.av_cfg, "initial_speed"):
                    speed = float(self.av_cfg.initial_speed)
                else:
                    speed = 0.0
                    
                # Choose add method based on warmup configuration
                if self.av_warmup_enabled:
                    # Use SUMO-controlled add method
                    self.add_av_with_sumo_control(edge_id, lane_id, position, speed)
                else:
                    # Use original external control method
                    self.add_av_unsafe(edge_id, lane_id, position, speed)
                    
                logger.info(f"AV added safely at lane {lane_id}, position {position}, mode: {self.av_control_mode}")
                if self.simulator.gui_flag:
                    traci.gui.trackVehicle("View #0", AV_ID)
                    # traci.gui.setZoom("View #0", 10000)
                return
            else:
                possible_lane_indexes.remove(lane)

        logger.warning("Unable to find a safe position for AV, using fallback method")
        self.add_av_fallback(edge_id)

    def is_position_safe(self, lane_id, position, min_safe_distance):
        """Check if the position is safe to add a AV.

        Args:
            lane_id (str): Lane ID.
            position (float): Position to check.
            min_safe_distance (float): Minimum safe distance from other vehicles.

        Returns:
            bool: True if the position is safe, False otherwise.
        """
        # check if this lane allows vehicles
        disallowed = traci.lane.getDisallowed(lane_id)
        if "passenger" in disallowed or "truck" in disallowed:
            return False

        # Check vehicles on the same lane
        vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
        for veh in vehicles:
            veh_pos = traci.vehicle.getLanePosition(veh)
            if abs(veh_pos - position) < min_safe_distance:
                return False
        return True

    def add_av_fallback(self, edge_id):
        """Add a AV to the simulation using a fallback method.

        Args:
            edge_id (str): Edge ID where the AV is added.
        """
        lane = random.randint(0, traci.edge.getLaneNumber(edge_id) - 1)
        lane_id = f"{edge_id}_{lane}"
        if hasattr(self.av_cfg, "initial_lane_position"):
            position = float(self.av_cfg.initial_lane_position)
        else:
            position = traci.lane.getLength(lane_id) / 2
        if hasattr(self.av_cfg, "type"):
            av_type = self.av_cfg.type
        else:
            av_type = "DEFAULT_VEHTYPE"
        if hasattr(self.av_cfg, "clearance_distance"):
            min_safe_distance = self.av_cfg.clearance_distance + traci.vehicletype.getLength(av_type)  # Minimum safe distance from other vehicles
        else:
            min_safe_distance = 50 + traci.vehicletype.getLength(av_type)  # Minimum safe distance from other vehicles
        # Clear area around the chosen position
        self.clear_area_around_position(
            lane_id, position, min_safe_distance
        )  # Clear 10m around the position
        if hasattr(self.av_cfg, "initial_speed"):
            speed = float(self.av_cfg.initial_speed)
        else:
            speed = 0.0
            
        # Choose add method based on warmup configuration
        if self.av_warmup_enabled:
            self.add_av_with_sumo_control(edge_id, lane_id, position, speed)
        else:
            self.add_av_unsafe(edge_id, lane_id, position, speed)
            
        logger.warning(
            f"AV added using fallback method at lane {lane_id}, position {position}, mode: {self.av_control_mode}"
        )
        if self.simulator.gui_flag:
            traci.gui.trackVehicle("View #0", AV_ID)
            # traci.gui.setZoom("View #0", 10000)

    def clear_area_around_position(self, lane_id, position, clear_distance):
        """Clear the area around a position on a lane.

        Args:
            lane_id (str): Lane ID.
            position (float): Position to clear around.
            clear_distance (float): Distance to clear around the position.
        """
        vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
        for veh in vehicles:
            veh_pos = traci.vehicle.getLanePosition(veh)
            if abs(veh_pos - position) < clear_distance:
                traci.vehicle.remove(veh)
        logger.info(f"Cleared area around position {position} on lane {lane_id}")
    
    def add_av_with_sumo_control(self, edge_id, lane_id, position, speed):
        """Add AV while maintaining SUMO control for warmup phase.
        
        Args:
            edge_id (str): Edge ID where the AV is added.
            lane_id (str): Lane ID where the AV is added.
            position (float): Position of the AV.
            speed (float): Speed of the AV.
        """
        av_type = self.av_cfg.type if hasattr(self.av_cfg, "type") else "DEFAULT_VEHTYPE"
        
        # Add vehicle
        self.add_vehicle(
            veh_id=AV_ID,
            route_id=AV_ROUTE_ID,
            lane="best",
            lane_id=lane_id,
            position=position,
            speed=speed,
            type_id=av_type,
        )
        
        # Set yellow color to indicate SUMO control
        traci.vehicle.setColor(AV_ID, (255, 255, 0, 255))
        
        # Subscribe to context information
        traci.vehicle.subscribeContext(
            AV_ID,
            traci.constants.CMD_GET_VEHICLE_VARIABLE,
            self.cache_radius,
            [traci.constants.VAR_DISTANCE],
        )
        
        # Key: DO NOT set SpeedMode and LaneChangeMode to 0
        # Let SUMO continue to control AV normally
        
        # Update control mode
        self.av_control_mode = "sumo"
        logger.info(f"AV added with SUMO control at {lane_id}, position {position}")
    
    def execute_av_warmup(self):
        """Execute AV warmup phase with monitoring."""
        self.av_warmup_start_time = traci.simulation.getTime()
        warmup_step_count = 0
        
        # Get trigger condition
        if isinstance(self.av_warmup_config, dict):
            trigger_type = self.av_warmup_config.get('trigger_type', 'position')
            log_progress = self.av_warmup_config.get('log_progress', False)
        else:
            trigger_type = getattr(self.av_warmup_config, 'trigger_type', 'position')
            log_progress = getattr(self.av_warmup_config, 'log_progress', False)
        
        logger.info(f"AV warmup started at simulation time {self.av_warmup_start_time}")
        
        while not self.check_warmup_trigger(trigger_type):
            traci.simulationStep()
            warmup_step_count += 1
            
            # Log progress periodically
            if log_progress and warmup_step_count % 50 == 0:
                self.log_warmup_progress(trigger_type)
                
        # Warmup complete, switch to external control
        self.switch_av_to_external_control()
        
        warmup_duration = traci.simulation.getTime() - self.av_warmup_start_time
        logger.info(f"AV warmup completed after {warmup_duration:.1f} seconds, {warmup_step_count} steps")
    
    def check_warmup_trigger(self, trigger_type):
        """Check if warmup end condition is met.
        
        Args:
            trigger_type (str): Type of trigger condition.
            
        Returns:
            bool: True if trigger condition is met.
        """
        if not AV_ID in traci.vehicle.getIDList():
            logger.warning("AV not in simulation during warmup")
            return True
            
        if trigger_type == "position":
            if isinstance(self.av_warmup_config, dict):
                config = self.av_warmup_config.get('position_trigger', {})
                target_pos = config.get('target_position', 500.0)
                tolerance = config.get('tolerance', 5.0)
            else:
                config = getattr(self.av_warmup_config, 'position_trigger', {})
                target_pos = getattr(config, 'target_position', 500.0) if hasattr(config, 'target_position') else config.get('target_position', 500.0)
                tolerance = getattr(config, 'tolerance', 5.0) if hasattr(config, 'tolerance') else config.get('tolerance', 5.0)
            
            current_pos = traci.vehicle.getLanePosition(AV_ID)
            return current_pos >= (target_pos - tolerance)
            
        elif trigger_type == "time":
            if isinstance(self.av_warmup_config, dict):
                config = self.av_warmup_config.get('time_trigger', {})
                duration = config.get('duration', 20.0)
            else:
                config = getattr(self.av_warmup_config, 'time_trigger', {})
                duration = getattr(config, 'duration', 20.0) if hasattr(config, 'duration') else config.get('duration', 20.0)
            
            elapsed = traci.simulation.getTime() - self.av_warmup_start_time
            return elapsed >= duration
            
        elif trigger_type == "zone":
            config = self.av_warmup_config.get('zone_trigger', {})
            target_edge = config.get('edge_id', '')
            min_pos = config.get('min_position', 0)
            max_pos = config.get('max_position', 1000)
            
            current_edge = traci.vehicle.getRoadID(AV_ID)
            current_pos = traci.vehicle.getLanePosition(AV_ID)
            
            return (target_edge in current_edge and 
                    min_pos <= current_pos <= max_pos)
            
        elif trigger_type == "edge":
            config = self.av_warmup_config.get('edge_trigger', {})
            target_edge = config.get('edge_id', '')
            
            current_edge = traci.vehicle.getRoadID(AV_ID)
            return target_edge in current_edge
            
        else:
            logger.warning(f"Unknown trigger type: {trigger_type}")
            return True
    
    def log_warmup_progress(self, trigger_type):
        """Log warmup progress information.
        
        Args:
            trigger_type (str): Type of trigger condition.
        """
        current_pos = traci.vehicle.getLanePosition(AV_ID)
        current_speed = traci.vehicle.getSpeed(AV_ID)
        current_edge = traci.vehicle.getRoadID(AV_ID)
        
        if trigger_type == "position":
            if isinstance(self.av_warmup_config, dict):
                target = self.av_warmup_config.get('position_trigger', {}).get('target_position', 0)
            else:
                config = getattr(self.av_warmup_config, 'position_trigger', {})
                target = getattr(config, 'target_position', 0) if hasattr(config, 'target_position') else 0
            progress = (current_pos / target) * 100 if target > 0 else 0
            logger.info(f"AV warmup progress: {progress:.1f}% (pos: {current_pos:.1f}/{target}m)")
            
        elif trigger_type == "time":
            elapsed = traci.simulation.getTime() - self.av_warmup_start_time
            if isinstance(self.av_warmup_config, dict):
                target = self.av_warmup_config.get('time_trigger', {}).get('duration', 0)
            else:
                config = getattr(self.av_warmup_config, 'time_trigger', {})
                target = getattr(config, 'duration', 0) if hasattr(config, 'duration') else 0
            progress = (elapsed / target) * 100 if target > 0 else 0
            logger.info(f"AV warmup progress: {progress:.1f}% (time: {elapsed:.1f}/{target}s)")
            
        logger.debug(f"AV state - Edge: {current_edge}, Pos: {current_pos:.1f}m, Speed: {current_speed:.1f}m/s")
    
    def switch_av_to_external_control(self):
        """Switch AV from SUMO control to external control."""
        if self.av_control_mode == "external":
            logger.warning("AV already in external control mode")
            return
            
        # Get current state for logging
        current_pos = traci.vehicle.getLanePosition(AV_ID)
        current_speed = traci.vehicle.getSpeed(AV_ID)
        current_edge = traci.vehicle.getRoadID(AV_ID)
        
        # Disable SUMO automatic control
        if not self.av_debug_control:
            traci.vehicle.setSpeedMode(AV_ID, 0)
            traci.vehicle.setLaneChangeMode(AV_ID, 0)
        
        # Maintain current speed to avoid sudden changes
        traci.vehicle.setSpeed(AV_ID, current_speed)
        
        # Update color to green for external control
        traci.vehicle.setColor(AV_ID, (0, 255, 0, 255))
        
        # Update control mode
        self.av_control_mode = "external"
        self.av_warmup_completed = True
        
        logger.info(f"AV control switched to EXTERNAL at edge {current_edge}, "
                   f"position {current_pos:.1f}m, speed {current_speed:.1f}m/s")

    def preparation(self):
        """Prepare for the NADE step."""
        super().preparation()

    @profile
    def NDE_decision(self, ctx):
        if AV_ID in traci.vehicle.getIDList():
            av_context_subscription_results = traci.vehicle.getContextSubscriptionResults(AV_ID)
            tmp_terasim_controlled_vehicle_ids = list(av_context_subscription_results.keys())
            # also exclude the static adversarial vehicles
            static_adversarial_object_id_list = []
            if self.static_adversity is not None and self.static_adversity.adversities is not None:
                for adversity in self.static_adversity.adversities:
                    for object_id in adversity._static_adversarial_object_id_list:
                        static_adversarial_object_id_list.append(object_id)
                        if object_id in tmp_terasim_controlled_vehicle_ids:
                            tmp_terasim_controlled_vehicle_ids.remove(object_id)
            self.simulator.ctx = {
                "terasim_controlled_vehicle_ids": tmp_terasim_controlled_vehicle_ids,
                "static_adversarial_object_id_list": static_adversarial_object_id_list,
            }
        return super().NDE_decision(self.simulator.ctx)

    @profile
    def NADE_decision(self, env_command_information, env_observation):
        """Make decisions using the NADE model.

        Args:
            env_command_information (dict): Command information from the environment.
            env_observation (dict): Observation from the environment.

        Returns:
            tuple: Tuple containing the control commands, updated command information, weight, future trajectory, maneuver challenge, and criticality.
        """
        predicted_AV_control_command = self.predict_av_control_command(env_observation)
        if env_command_information[AgentType.VEHICLE][AV_ID] is None:
            env_command_information[AgentType.VEHICLE][AV_ID] = Dict()
        if predicted_AV_control_command is not None:
            env_command_information[AgentType.VEHICLE][AV_ID]["ndd_command_distribution"] = Dict(
                {
                    # "adversarial": predicted_AV_control_command,
                    # "normal": NDECommand(command_type=CommandType.DEFAULT, prob=0),
                    "normal": predicted_AV_control_command,
                }
            )
        else:
            env_command_information[AgentType.VEHICLE][AV_ID]["ndd_command_distribution"] = Dict(
                {
                    "normal": NDECommand(command_type=CommandType.DEFAULT, prob=1),
                }
            )
        AV_command_cache = copy.deepcopy(env_command_information[AgentType.VEHICLE][AV_ID]["command_cache"])

        # filter the env_command_information and env_observation by the control radius from AV
        distance_from_AV = calclulate_distance_from_centered_agent(
            env_observation, AV_ID, AgentType.VEHICLE
        )
        neighbor_agent_set = set(
            [agent_id for agent_id in distance_from_AV if distance_from_AV[agent_id] <= self.control_radius]
        )
        filtered_env_command_information = {
            agent_type: {
                agent_id: env_command_information[agent_type][agent_id]
                for agent_id in env_command_information[agent_type]
                if agent_id in neighbor_agent_set
            }
            for agent_type in env_command_information
        }
        filtered_env_observation = {
            agent_type: {
                agent_id: env_observation[agent_type][agent_id]
                for agent_id in env_observation[agent_type]
                if agent_id in neighbor_agent_set
            }
            for agent_type in env_observation
        }

        (
            nade_control_commands,
            filtered_env_command_information,
            weight,
            filtered_env_future_trajectory,
            filtered_env_maneuver_challenge,
            filtered_env_criticality,
        ) = super().NADE_decision(filtered_env_command_information, filtered_env_observation)
        nade_control_commands[AgentType.VEHICLE][AV_ID] = AV_command_cache
        return (
            nade_control_commands,
            env_command_information,
            weight,
            filtered_env_future_trajectory,
            filtered_env_maneuver_challenge,
            filtered_env_criticality,
        )

    @profile
    def NADE_decision_and_control(self, env_command_information, env_observation):
        """Make decisions and control the agents around the AV using the NADE model.

        Args:
            env_command_information (dict): Command information from the environment.
            env_observation (dict): Observation from the environment.
        """
        if AV_ID in traci.vehicle.getIDList():
            AV_control_command_cache = env_command_information[AgentType.VEHICLE][AV_ID]["command_cache"]
            (
                nade_control_commands,
                env_command_information,
                weight,
                env_future_trajectory,
                _,
                _,
            ) = self.NADE_decision(
                env_command_information, env_observation
            )
            self.importance_sampling_weight *= weight  # update weight by negligence
            nade_control_commands, env_command_information, weight, self.record = apply_collision_avoidance(
                env_future_trajectory, env_command_information, nade_control_commands, self.record, excluded_agent_set=self.excluded_agent_set
            )
            self.importance_sampling_weight *= (
                weight  
            ) # update weight by collision avoidance
            nade_control_commands = update_control_cmds_from_predicted_trajectory(
                nade_control_commands, env_future_trajectory, excluded_agent_set=self.excluded_agent_set
            ) # update the control commands according to the predicted trajectory
            if hasattr(self, "nnde_make_decisions"):
                nnde_control_commands, _ = self.nnde_make_decisions()
                nade_control_commands = self.merge_NADE_NeuralNDE_control_commands(
                    nade_control_commands, nnde_control_commands
                )
            self.refresh_control_commands_state()
            if AV_ID in nade_control_commands[AgentType.VEHICLE] and AV_control_command_cache is not None:
                nade_control_commands[AgentType.VEHICLE][AV_ID] = AV_control_command_cache
            self.execute_control_commands(nade_control_commands)
            self.record_step_data(env_command_information)

    def calculate_total_distance(self):
        """Calculate the total distance traveled by the AV.

        Returns:
            float: Total distance traveled by the AV.
        """
        try:
            AV_distance = traci.vehicle.getDistance(AV_ID)
        except:
            AV_distance = 0
            if AV_ID not in self.distance_info.before:
                AV_distance += self.distance_info.after[AV_ID]
            else:
                AV_distance += (
                    self.distance_info.after[AV_ID] - self.distance_info.before[AV_ID]
                )
        return AV_distance

    def predict_av_control_command(
        self, env_observation
    ):
        """Predict the control command for the AV.

        Args:
            env_observation (dict): Observation from the environment.

        Returns:
            NDECommand: Predicted control command for the AV.
        """
        original_av_speed = env_observation[AgentType.VEHICLE][AV_ID]["ego"]["velocity"]
        original_av_acceleration = env_observation[AgentType.VEHICLE][AV_ID]["ego"]["acceleration"]
        new_av_speed = traci.vehicle.getSpeedWithoutTraCI(AV_ID)
        new_av_acceleration = (
            new_av_speed - original_av_speed
        ) / utils.get_step_size()

        original_av_angle = env_observation[AgentType.VEHICLE][AV_ID]["ego"]["heading"]
        av_lane_id = traci.vehicle.getLaneID(AV_ID)
        av_lane_position = traci.vehicle.getLanePosition(AV_ID)
        av_lane_angle = traci.lane.getAngle(
            laneID=av_lane_id,
            relativePosition=max(
                av_lane_position - 0.5 * traci.vehicle.getLength(AV_ID), 0
            ),
        )
        AV_command = None

        # step 1. use AV signal to predict the control command
        av_signal = traci.vehicle.getSignals(AV_ID)
        if av_signal == 1: # right turn signal, please consider the drive rule: lefthand or righthand
            if self.configuration.drive_rule == "righthand":
                AV_command = NDECommand(
                    command_type=CommandType.RIGHT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "RightFoll"},
                )
            else:
                AV_command = NDECommand(
                    command_type=CommandType.LEFT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "LeftFoll"},
                )
        elif av_signal == 2: # left turn signal, please consider the drive rule: lefthand or righthand
            if self.configuration.drive_rule == "righthand":
                AV_command = NDECommand(
                    command_type=CommandType.LEFT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "LeftFoll"},
                )
            else:
                AV_command = NDECommand(
                    command_type=CommandType.RIGHT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "RightFoll"},
                )

        elif av_signal == 0: # no signal
            # step 2. use the difference between the lane change angle adn the original av angle to predict the control command (LEFT turn or RIGHT turn)
            # the angle is defined as SUmo's angle, the north is 0, the east is 90, the south is 180, the west is 270
            # the angle is in degree
            angle_diff = (av_lane_angle - original_av_angle + 180) % 360 - 180

            if angle_diff > 10:
                AV_command = NDECommand(
                    command_type=CommandType.LEFT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "LeftFoll"},
                )
            elif angle_diff < -10:
                AV_command = NDECommand(
                    command_type=CommandType.RIGHT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "RightFoll"},
                )

            if original_av_acceleration - new_av_acceleration > 1.5:
                # predict the av control command as negligence
                leader_info = traci.vehicle.getLeader(AV_ID)
                is_car_following_flag = False
                if leader_info is not None:
                    is_car_following_flag = is_car_following(AV_ID, leader_info[0])
                AV_command = NDECommand(
                    command_type=CommandType.ACCELERATION,
                    acceleration=original_av_acceleration,
                    prob=1,
                    duration=1.0,
                    info={
                        "adversarial_mode": "Lead",
                        "is_car_following_flag": is_car_following_flag,
                    },
                )

        if AV_command:
            _, predicted_collision_type = get_collision_type_and_prob(
                obs_dict=env_observation[AgentType.VEHICLE][AV_ID],
                adversarial_command=AV_command,
            )
            AV_command.info.update(
                {"predicted_collision_type": predicted_collision_type}
            )
        return AV_command
    
    def analyse_liability(self, veh_1_id, veh_2_id):
        """Analyse the liability of the collision. return collider_id, victim_id
        """
        # veh_1_lane_id = traci.vehicle.getLaneID(veh_1_id)
        # veh_2_lane_id = traci.vehicle.getLaneID(veh_2_id)
        veh_1_edge_id = traci.vehicle.getRoadID(veh_1_id)
        veh_2_edge_id = traci.vehicle.getRoadID(veh_2_id)
        print("Analysing liability for collision between vehicle {} and vehicle {}".format(veh_1_id, veh_2_id))
        if veh_1_edge_id == veh_2_edge_id: # rear end or side collision
            # get the front vehicle using traci lane position
            print("two vehicles are on the same edge", veh_1_edge_id, veh_2_edge_id)
            veh_1_lane_position = traci.vehicle.getLanePosition(veh_1_id)
            veh_2_lane_position = traci.vehicle.getLanePosition(veh_2_id)
            if veh_1_lane_position > veh_2_lane_position:
                front_veh_id = veh_1_id
                rear_veh_id = veh_2_id
            else:
                front_veh_id = veh_2_id
                rear_veh_id = veh_1_id
            print("front vehicle: ", front_veh_id, "rear vehicle: ", rear_veh_id)
            # if the front vehicle has non-zero lateral speed, it is the front vehicle cutting in and generating the collision
            if abs(traci.vehicle.getLateralSpeed(front_veh_id)) > 0.2:
                print(f"Fix error, vehicle {front_veh_id} is the collider, vehicle {rear_veh_id} is the victim")
                return front_veh_id, rear_veh_id
            else:
                return rear_veh_id, front_veh_id
        else:
            return veh_1_id, veh_2_id
        

    def should_continue_simulation(self):
        """Check if the simulation should continue. There are four conditions to stop the simulation:
        1. Collision happens between two vehicles.
        2. Collision happens between a vehicle and a VRU.
        3. AV leaves the simulation.
        4. Simulation timeout.

        Returns:
            bool: True if the simulation should continue, False otherwise.
        """
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        colliding_vehicles = self.simulator.get_colliding_vehicles()
        self._vehicle_in_env_distance("after")
        collision_objects = traci.simulation.getCollisions()
        collision_object_ids = traci.simulation.getCollidingVehiclesIDList()
        if num_colliding_vehicles >= 2 and "AV" in collision_object_ids:
            # collision_objects = self.analyse_liability()
            collison_object = None
            for obj in collision_objects:
                if obj.collider == AV_ID or obj.victim == AV_ID:
                    collison_object = obj
                    break
            assert collison_object is not None
            veh_1_id = collison_object.collider
            veh_2_id = collison_object.victim
            collider_id, victim_id = self.analyse_liability(veh_1_id, veh_2_id)
            self.record.update(
                {
                    "finish_reason": "collision",
                    "collider": collider_id,
                    "victim": victim_id,
                    "veh_1_id": collider_id,
                    "veh_1_obs": self.vehicle_list[collider_id].observation if collider_id in self.vehicle_list else None,
                    "veh_2_id": victim_id,
                    "veh_2_obs": self.vehicle_list[victim_id].observation if victim_id in self.vehicle_list else None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                }
            )
            return False
        elif num_colliding_vehicles == 1 and "AV" in collision_object_ids: # collision happens between a vehicle and a vru.
            veh_1_id = colliding_vehicles[0]
            if veh_1_id == collision_objects[0].collider:
                vru_1_id = collision_objects[0].victim
            elif veh_1_id == collision_objects[0].victim:
                vru_1_id = collision_objects[0].collider
            else:
                vru_1_id = None
            self.record.update(
                {
                    "finish_reason": "collision",
                    "collider": collision_objects[0].collider,
                    "victim": collision_objects[0].victim,
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation if veh_1_id in self.vehicle_list else None,
                    "vru_1_id": vru_1_id,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                }
            )
            return False
        elif AV_ID not in traci.vehicle.getIDList():
            logger.info(f"{AV_ID} left the simulation, stop the simulation.")
            self.record.update(
                {
                    "finish_reason": "AV_left",
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                }
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            logger.info("Simulation timeout, stop the simulation.")
            self.record.update(
                {
                    "finish_reason": "timeout",
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                }
            )
            return False
        return True

    def record_step_data(self, env_command_information):
        """Record the step data.

        Args:
            env_command_information (dict): Command information from the environment.
        """
        step_log = Dict()
        bv_criticality_list = []
        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for agent_id, agent_command_info in env_command_information[agent_type].items():
                maneuver_challenge = agent_command_info.get("maneuver_challenge", None)
                if maneuver_challenge and maneuver_challenge.get("adversarial", None):
                    step_log[agent_id]["maneuver_challenge"] = maneuver_challenge

                keys = ["avoidable", "conflict_vehicle_list", "mode"]
                step_log[agent_id].update(
                    {key: agent_command_info[key] for key in keys if agent_command_info.get(key)}
                )
                if step_log[agent_id].get("avoidable"):
                    step_log[agent_id].pop("avoidable") # remove the avoidable key if it is True
                if agent_id != AV_ID:
                    criticality = 0.0
                    if (
                        "criticality" in agent_command_info
                        and "adversarial" in agent_command_info["criticality"]
                    ):
                        criticality = agent_command_info["criticality"]["adversarial"]
                    bv_criticality_list.append(criticality)
        # pop the empty dict
        step_log = {k: v for k, v in step_log.items() if v}
        step_log = {
            "weight": self.importance_sampling_weight,
            "vehicle_log": step_log,
        }
        time_step = utils.get_time()
        self.record.step_info[time_step] = step_log
        self.record.weight_step_info[time_step] = self.step_weight
        self.record.epsilon_step_info[time_step] = self.step_epsilon
        self.record.criticality_step_info[time_step] = sum(bv_criticality_list)
        self.record.drl_obs[time_step] = self.collect_drl_obs(env_command_information).tolist()
        overall_avoidable = True
        for agent_type in env_command_information:
            for agent_id in env_command_information[agent_type]:
                if not env_command_information[agent_type][agent_id].get("avoidable", True):
                    overall_avoidable = False
        self.record.avoidable[time_step] = overall_avoidable

        return step_log

    def collect_drl_obs(self, env_command_information):
        """Collect the observation for D2RL-based adversity model.

        Args:
            env_command_information (dict): Command information from the environment.

        Returns:
            np.ndarray: Observation for D2RL-based adversity model.
        """
        AV_global_position = list(traci.vehicle.getPosition(AV_ID))
        AV_speed = traci.vehicle.getSpeed(AV_ID)
        AV_heading = traci.vehicle.getAngle(AV_ID)
        AV_driving_distance = traci.vehicle.getDistance(AV_ID)
        # position x, position y, AV driving distance, velocity, heading
        vehicle_info_list = []
        controlled_bv_num = 1
        for veh_id, veh_command_information in env_command_information[AgentType.VEHICLE].items():
            if veh_id == AV_ID:
                continue
            if (
                "criticality" in veh_command_information
                and "negligence" in veh_command_information["criticality"]
            ):
                criticality = veh_command_information["criticality"]["adversarial"]
                if criticality > 0:
                    vehicle_local_position = list(traci.vehicle.getPosition(veh_id))
                    vehicle_relative_position = [
                        vehicle_local_position[0] - AV_global_position[0],
                        vehicle_local_position[1] - AV_global_position[1],
                    ]
                    vehicle_speed = traci.vehicle.getSpeed(veh_id)
                    vehicle_heading = traci.vehicle.getAngle(veh_id)
                    vehicle_info_list.extend(
                        vehicle_relative_position + [vehicle_speed] + [vehicle_heading]
                    )
                    break

        if not vehicle_info_list:
            vehicle_info_list.extend([-100, -100, 0, 0])

        velocity_lb, velocity_ub = 0, 10
        AV_position_lb, AV_position_ub = [0, 0], [240, 400]
        driving_distance_lb, driving_distance_ub = 0, 1000
        heading_lb, heading_ub = 0, 360
        vehicle_info_lb, vehicle_info_ub = [-20, -20, 0, 0], [20, 20, 10, 360]

        lb_array = np.array(
            AV_position_lb
            + [velocity_lb]
            + [driving_distance_lb]
            + [heading_lb]
            + vehicle_info_lb
        )
        ub_array = np.array(
            AV_position_ub
            + [velocity_ub]
            + [driving_distance_ub]
            + [heading_ub]
            + vehicle_info_ub
        )
        total_obs_for_DRL_ori = np.array(
            AV_global_position
            + [AV_speed]
            + [AV_driving_distance]
            + [AV_heading]
            + vehicle_info_list
        )

        total_obs_for_DRL = (
            2 * (total_obs_for_DRL_ori - lb_array) / (ub_array - lb_array) - 1
        )
        total_obs_for_DRL = np.clip(total_obs_for_DRL, -5, 5)
        return np.array(total_obs_for_DRL).astype(float)
