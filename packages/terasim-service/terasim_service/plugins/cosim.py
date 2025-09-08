import json
import logging
from logging.handlers import RotatingFileHandler
import numpy as np
import redis
from redis.exceptions import RedisError
import time
import subprocess
from pathlib import Path

from terasim.overlay import traci
from terasim.simulator import Simulator

from terasim_nde_nade.adversity import ConstructionAdversity

from .base import BasePlugin, DEFAULT_REDIS_CONFIG

from ..utils import SimulationState, AgentStateSimplified, SUMOSignal, AgentCommand


def interpolate_by_distance(points, step):
    """
    Interpolate a tuple of tuples so that the distance between each point is equal to 'step'.

    Args:
        points (tuple of tuple): Original shape, e.g., ((x1, y1), (x2, y2), ...)
        step (float): Desired distance between points.

    Returns:
        list of list: Interpolated points as [[x, y], ...] with equal spacing.
    """
    points = np.array(points, dtype=np.float32)
    # Compute distances between consecutive points
    deltas = np.diff(points, axis=0)
    seg_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
    cumulative = np.insert(np.cumsum(seg_lengths), 0, 0)
    total_length = cumulative[-1]
    if total_length == 0:
        return [points[0].tolist()]
    # Generate equally spaced distances
    num_points = int(np.floor(total_length / step)) + 1
    distances = np.linspace(0, total_length, num_points)
    # Interpolate x and y separately
    x_interp = np.interp(distances, cumulative, points[:, 0])
    y_interp = np.interp(distances, cumulative, points[:, 1])
    return [[float(x), float(y)] for x, y in zip(x_interp, y_interp)]


def generate_construction_zone_shape(lane_shape, lane_width, direction):
    """
    Generate a construction zone shape based on the lane shape and lane width.
    The first ten points of the lane_shape are offset laterally, with the offset
    gradually changing from direction * lane_width/2 to -direction * lane_width/2.
    The remaining points are offset by a constant -direction * lane_width/2.

    Args:
        lane_shape (list of list): The lane shape as a list of [x, y] points.
        lane_width (float): The width of the lane.
        direction (int): -1 for from left to right, 1 for from right to left.

    Returns:
        list of list: The offset lane shape.
    """
    n = min(10, len(lane_shape))
    construction_zone_shape = []
    for i, pt in enumerate(lane_shape):
        pt = np.array(pt)
        # Compute tangent direction
        if i < len(lane_shape) - 1:
            next_pt = np.array(lane_shape[i + 1])
            dir_vec = next_pt - pt
        else:
            prev_pt = np.array(lane_shape[i - 1])
            dir_vec = pt - prev_pt
        norm = np.linalg.norm(dir_vec)
        if norm == 0:
            dir_vec = np.array([1.0, 0.0])
        else:
            dir_vec = dir_vec / norm
        # Normal vector (perpendicular)
        normal = np.array([-dir_vec[1], dir_vec[0]]) * direction * -1

        # Compute offset
        if i < n:
            # Linear interpolation from +lane_width/2 to -lane_width/2
            alpha = i / (n - 1) if n > 1 else 0
            offset_val = (1 - alpha) * (lane_width / 2) + alpha * (-lane_width / 2)
        else:
            offset_val = - lane_width / 2

        offset_pt = pt + normal * offset_val
        construction_zone_shape.append(offset_pt.tolist())
    return construction_zone_shape


DEFAULT_COSIM_PLUGIN_CONFIG = {
    "name": "terasim_cosim_plugin",
    "priority": {
        "before_env": {
            "start": -90,
            "step": -90,
            "stop": -90,
        },
        "after_env": {
            "start": 90,
            "step": 90,
            "stop": 90,
        },
    },
}


class TeraSimCoSimPlugin(BasePlugin):
    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_COSIM_PLUGIN_CONFIG,
        redis_config: dict = DEFAULT_REDIS_CONFIG,
        base_dir: str = "output",
        key_expiry=3600,
        auto_run=False,
        enable_viz=False,
        viz_port=8050,
        viz_update_freq=5,
    ):
        """Initialize the Co-Simulation plugin.

        Args:
            simulation_uuid (str): Unique identifier for the simulation instance.
            plugin_config (dict, optional): Configuration for the plugin. Defaults to DEFAULT_COSIM_PLUGIN_CONFIG.
            redis_config (dict, optional): Configuration for the Redis connection. Defaults to DEFAULT_REDIS_CONFIG.
            base_dir (str, optional): Base directory for the log file. Defaults to "output".
            key_expiry (int, optional): Key expiration time in seconds. Defaults to 3600.
            auto_run (bool, optional): Flag to enable auto-run mode. Defaults to False.
            enable_viz (bool, optional): Enable visualization with Streamlit. Defaults to False.
            viz_port (int, optional): Port for Streamlit server. Defaults to 8050.
            viz_update_freq (int, optional): Visualization update frequency. Defaults to 5.
        """
        super().__init__(simulation_uuid, plugin_config, redis_config)
        # Key expiration time in seconds (default: 1 hour)
        self.key_expiry = key_expiry
        self.auto_run = auto_run
        self.base_dir = base_dir

        # Visualization settings
        self.enable_viz = enable_viz
        self.viz_port = viz_port
        self.viz_update_freq = viz_update_freq
        self.viz_process = None

        # Setup logging
        self.logger = self._setup_logger(base_dir)

        # Maintain controlled agents in each step, assuming each agent can be controlled by only one command
        self.controlled_agents_each_step = set()

        # Cache construction zone shapes
        self.construction_zone_shapes = None

        # Initialize last orientations cache
        self.last_orientations = {}  # {vehicle_id: (last_orientation, last_time)}
        
        # Initialize health monitoring
        self.error_count = 0
        self.last_successful_operation = time.time()

    def _setup_logger(self, base_dir: str) -> logging.Logger:
        """Setup logger for the plugin.

        Args:
            base_dir (str): Base directory for the log file.

        Returns:
            logging.Logger: Logger instance for the plugin.
        """
        logger = logging.getLogger(f"{self.plugin_name}-{self.simulation_uuid}")
        logger.setLevel(logging.DEBUG)

        # Create a rotating file handler
        file_handler = RotatingFileHandler(
            f"{base_dir}/{self.plugin_name}.log",
            maxBytes=10 * 1024 * 1024,  # 10MB
            backupCount=5,
        )
        file_handler.setLevel(logging.DEBUG)

        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Create formatter and add it to the handlers
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
        )
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)

        # Add the handlers to the logger
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        return logger

    def function_before_env_start(self, simulator: Simulator, ctx):
        """Connect to the Redis server and set the simulation status to be 'initializing'.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        try:
            # Initialize Redis connection
            self.redis_client = redis.Redis(**self.redis_config)

            # Clear old data and set initial state with expiration
            self.redis_client.delete(f"simulation:{self.simulation_uuid}:*")
            self.redis_client.set(
                f"simulation:{self.simulation_uuid}:status", "initializing", ex=self.key_expiry
            )

            self.logger.info(
                f"Redis connection established. Simulation UUID: {self.simulation_uuid}, start initialization!"
            )

            # Add this line to write initial simulation state
            # self._write_simulation_state(simulator)

            return True
        except RedisError as e:
            self.logger.error(f"Failed to initialize Redis: {e}")
            return False
        except Exception as e:
            self.logger.exception(f"Unexpected error during initialization: {e}")
            return False
        
    def function_after_env_start(self, simulator: Simulator, ctx):
        """Set the simulation status to 'wait_for_tick' after finishing the intialization.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        try:
            # Set initial state with expiration
            self.redis_client.set(
                f"simulation:{self.simulation_uuid}:status", "wait_for_tick", ex=self.key_expiry
            )

            self.logger.info(
                f"Redis connection established. Simulation UUID: {self.simulation_uuid}, finish initialization!"
            )

            # Extract map data and start visualization if enabled
            if self.enable_viz:
                self.logger.info("Visualization enabled, extracting map data...")
                map_data = self._extract_map_geometry(simulator.sumo_net)
                
                # Store map data in Redis
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:map_data",
                    json.dumps(map_data),
                    ex=self.key_expiry
                )
                
                self.logger.info(
                    f"Map data extracted: {len(map_data['lanes'])} lanes, "
                    f"{len(map_data['junctions'])} junctions, "
                    f"{len(map_data['traffic_lights'])} traffic lights"
                )
                
                # Start Streamlit visualization
                self._start_streamlit_service()

            return True
        except RedisError as e:
            self.logger.error(f"Failed to initialize Redis: {e}")
            return False
        except Exception as e:
            self.logger.exception(f"Unexpected error during initialization: {e}")
            return False

    def function_before_env_step(self, simulator: Simulator, ctx):
        """Handle simulation step logic, including handling simulation level commands, handling agent-level command, and retrieving simulation states.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        
        Returns:
            bool: True if the simulation step was successful, False otherwise.
        """
        idle_start_time = time.time()
        
        while True:
            # Auto-stop if no commands for 10 minutes
            if time.time() - idle_start_time > 600:  # 10 minutes
                self.logger.warning("No activity for 10 minutes, auto-stopping")
                return False
                
            # Handle simulation control commands
            command = self._get_and_handle_command(simulator)
            if command == "stop":
                return False

            if command:
                idle_start_time = time.time()  # Reset idle timer

            # Handle all pending vehicle commands
            self.controlled_agents_each_step.clear()
            self._handle_pending_agent_commands()

            # Write current simulation state
            state_write_success = self._write_simulation_state(simulator)
            if not state_write_success:
                return False

            if self._is_simulation_paused():
                time.sleep(0.1)  # Wait while paused
                continue

            if not self.auto_run:
                if command == "tick":
                    break  # Proceed with the simulation step
                else:
                    time.sleep(0.005)  # Short sleep to prevent busy waiting
                    continue

            break  # Proceed with the simulation step in auto_run mode
        self.redis_client.set(
            f"simulation:{self.simulation_uuid}:status", "running", ex=self.key_expiry
        )
        self.logger.info("Simulation step started")
        return True
    
    def function_after_env_step(self, simulator: Simulator, ctx):
        """Handle post-simulation step logic, including updating simulation status.
        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        Returns:
            bool: True if the simulation step was successful, False otherwise.
        """
        self.redis_client.set(
            f"simulation:{self.simulation_uuid}:status", "ticked", ex=self.key_expiry
        )
        self.logger.info("Simulation step finished!")
        return True

    def function_before_env_stop(self, simulator: Simulator, ctx):
        """Handle simulation stopping logic. Default implementation does nothing.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        pass

    def function_after_env_stop(self, simulator: Simulator, ctx):
        """Handle post-simulation stopping logic, including updating simulation status.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        try:
            # Stop visualization if enabled
            if self.enable_viz and self.viz_process:
                self.logger.info("Stopping visualization service...")
                try:
                    self.viz_process.terminate()
                    self.viz_process.wait(timeout=5)
                    self.logger.info("Visualization service stopped")
                except Exception as e:
                    self.logger.error(f"Error stopping visualization: {e}")
                    
            if self.redis_client:
                finish_string = f"Simulation {self.simulation_uuid} finished!"
                # Set simulation end status briefly before cleanup
                results_dict = {
                    "finish_reason": simulator.env.record.get("finish_reason",""),
                    "collider": simulator.env.record.get("collider",""),
                    "victim": simulator.env.record.get("victim",""),
                }
                results_str = json.dumps(results_dict)
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:status",
                    "finished",
                    ex=1800,  # Keep status for 10 seconds only
                )
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:result",
                    results_str,
                    ex=1800,  # Keep status for 30 minutes
                )

                # Clean up visualization data if enabled
                if self.enable_viz:
                    self.redis_client.delete(f"simulation:{self.simulation_uuid}:map_data")

                # Close Redis connection
                self.redis_client.close()
                self.logger.info(finish_string)
        except RedisError as e:
            self.logger.error(f"Error during Redis cleanup: {e}")
        except Exception as e:
            self.logger.exception(f"Unexpected error during cleanup: {e}")
    
    def inject(self, simulator: Simulator, ctx):
        """Inject the plugin into the simulation.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook(f"{self.plugin_name}_before_env_start", self.function_before_env_start, priority=self.plugin_priority["before_env"]["start"])
        simulator.start_pipeline.hook(f"{self.plugin_name}_after_env_start", self.function_after_env_start, priority=self.plugin_priority["after_env"]["start"])
        simulator.step_pipeline.hook(f"{self.plugin_name}_before_env_step", self.function_before_env_step, priority=self.plugin_priority["before_env"]["step"])
        simulator.step_pipeline.hook(f"{self.plugin_name}_after_env_step", self.function_after_env_step, priority=self.plugin_priority["after_env"]["step"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_before_env_stop", self.function_before_env_stop, priority=self.plugin_priority["before_env"]["stop"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_after_env_stop", self.function_after_env_stop, priority=self.plugin_priority["after_env"]["stop"])
    
    def _check_simulation_status(self) -> bool:
        """Check if simulation is still running.

        Returns:
            bool: True if simulation is running, False if stopped or doesn't exist
        """
        status = self.redis_client.get(f"simulation:{self.simulation_uuid}:status")
        if not status or status.decode("utf-8") == "finished":
            self.logger.warning(
                f"Simulation {self.simulation_uuid} is stopped or doesn't exist"
            )
            return False
        return True

    def _get_and_handle_command(self, simulator: Simulator) -> str | None:
        """Get and handle simulation control commands.

        Args:
            simulator (Simulator): The simulator object.

        Returns:
            str | None: The control command to execute, or None if no command is present.
        """
        if not self._check_simulation_status():
            return "stop"
        command = self.redis_client.get(f"simulation:{self.simulation_uuid}:control")
        if command:
            command = command.decode("utf-8")
            self._handle_control_command(command, simulator)
            if command != "stop":
                self.redis_client.delete(f"simulation:{self.simulation_uuid}:control")
        return command

    def _is_simulation_paused(self) -> bool:
        """Check if the simulation is paused.

        Returns:
            bool: True if simulation is paused, False otherwise.
        """
        if not self._check_simulation_status():
            return False
        return bool(self.redis_client.exists(f"simulation:{self.simulation_uuid}:paused"))

    def _handle_control_command(self, command, simulator):
        """Handle simulation control commands.
        
        Args:
            command (str): The control command to execute.
            simulator (Simulator): The simulator object.
        """
        if command == "pause":
            self.redis_client.set(f"simulation:{self.simulation_uuid}:paused", "1")
            self.logger.info("Simulation paused")
        elif command == "resume":
            self.redis_client.delete(f"simulation:{self.simulation_uuid}:paused")
            self.logger.info("Simulation resumed")
        elif command == "stop":
            self.logger.info("Stopping simulation")
            simulator.running = False
        # Add more control command handling logic as needed

    def get_vehicle_vru_ids(self):
        """Get all vehicle and VRU IDs in the simulation."""
        all_ids = list(set(traci.vehicle.getIDList() + traci.person.getIDList()))
        # Separate by type: construction objects, VRUs, and regular vehicles
        construction_ids = [id for id in all_ids if id.startswith("CONSTRUCTION_")]
        vru_ids = [id for id in all_ids if "VRU" in id and id not in construction_ids]
        vehicle_ids = [id for id in all_ids if id not in vru_ids and id not in construction_ids]
        return vehicle_ids, vru_ids, construction_ids

    def _write_simulation_state(self, simulator):
        """Write the current simulation state to Redis.

        Args:
            simulator (Simulator): The simulator object.
        """
        if not self._check_simulation_status():
            return False
        try:
            simulation_state = SimulationState()
            simulation_state.simulation_time = traci.simulation.getTime()

            # Get all interested agent IDs
            vehicle_ids, vru_ids, construction_ids = self.get_vehicle_vru_ids()
            simulation_state.agent_count = {
                "vehicle": len(vehicle_ids),
                "vru": len(vru_ids),
                "construction": len(construction_ids),
            }

            # Add vehicle states
            vehicles = {}
            for vid in vehicle_ids:
                vehicle_state = AgentStateSimplified()
                vehicle_state.x,vehicle_state.y,vehicle_state.z = traci.vehicle.getPosition3D(vid)
                vehicle_state.lon,vehicle_state.lat = traci.simulation.convertGeo(vehicle_state.x, vehicle_state.y)
                vehicle_state.sumo_angle = traci.vehicle.getAngle(vid)
                vehicle_state.orientation = np.radians((90 - vehicle_state.sumo_angle) % 360)
                vehicle_state.speed = traci.vehicle.getSpeed(vid)
                vehicle_state.acceleration = traci.vehicle.getAcceleration(vid)
                vehicle_state.length = traci.vehicle.getLength(vid)
                vehicle_state.width = traci.vehicle.getWidth(vid)
                vehicle_state.height = traci.vehicle.getHeight(vid)
                vehicle_state.type = traci.vehicle.getTypeID(vid)
                vehicle_state.angular_velocity = 0.0  # rad/s
                now_time = simulation_state.simulation_time
                now_orientation = vehicle_state.orientation
                last_orientation, last_time = self.last_orientations.get(vid, (now_orientation, now_time))
                dt = now_time - last_time
                if dt > 0:
                    dtheta = np.arctan2(np.sin(now_orientation - last_orientation), np.cos(now_orientation - last_orientation))
                    vehicle_state.angular_velocity = dtheta / dt
                else:
                    vehicle_state.angular_velocity = 0.0
                self.last_orientations[vid] = (now_orientation, now_time)
                vehicles[vid] = vehicle_state

            simulation_state.agent_details["vehicle"] = vehicles

            # Add VRU states
            # Get current vehicle and person lists to determine actual object type
            current_vehicle_list = traci.vehicle.getIDList()
            current_person_list = traci.person.getIDList()
            
            vrus = {}
            for vru_id in vru_ids:
                vru_state = AgentStateSimplified()
                
                # Determine if this VRU is actually a vehicle or person
                if vru_id in current_vehicle_list:
                    # VRU is actually a vehicle (disguised as pedestrian)
                    vru_state.x, vru_state.y, vru_state.z = traci.vehicle.getPosition3D(vru_id)
                    vru_state.lon, vru_state.lat = traci.simulation.convertGeo(vru_state.x, vru_state.y)
                    vru_state.sumo_angle = traci.vehicle.getAngle(vru_id)
                    vru_state.speed = traci.vehicle.getSpeed(vru_id)
                    vru_state.acceleration = traci.vehicle.getAcceleration(vru_id)
                    vru_state.length = traci.vehicle.getLength(vru_id)
                    vru_state.width = traci.vehicle.getWidth(vru_id)
                    vru_state.height = traci.vehicle.getHeight(vru_id)
                    vru_state.type = traci.vehicle.getTypeID(vru_id)
                    vru_state.angular_velocity = 0.0  # rad/s
                    now_time = simulation_state.simulation_time
                    now_orientation = np.radians((90 - vru_state.sumo_angle) % 360)
                    last_orientation, last_time = self.last_orientations.get(vru_id, (now_orientation, now_time))
                    dt = now_time - last_time
                    if dt > 0:
                        dtheta = np.arctan2(np.sin(now_orientation - last_orientation), np.cos(now_orientation - last_orientation))
                        vru_state.angular_velocity = dtheta / dt
                    else:
                        vru_state.angular_velocity = 0.0
                    self.last_orientations[vru_id] = (now_orientation, now_time)
                    vru_state.orientation = now_orientation
                elif vru_id in current_person_list:
                    # VRU is actually a person
                    vru_state.x, vru_state.y, vru_state.z = traci.person.getPosition3D(vru_id)
                    vru_state.lon, vru_state.lat = traci.simulation.convertGeo(vru_state.x, vru_state.y)
                    vru_state.sumo_angle = traci.person.getAngle(vru_id)
                    vru_state.speed = traci.person.getSpeed(vru_id)
                    vru_state.acceleration = traci.person.getAcceleration(vru_id)
                    vru_state.length = traci.person.getLength(vru_id)
                    vru_state.width = traci.person.getWidth(vru_id)
                    vru_state.height = traci.person.getHeight(vru_id)
                    vru_state.type = traci.person.getTypeID(vru_id)
                    vru_state.angular_velocity = 0.0  # rad/s
                    vru_state.orientation = np.radians((90 - vru_state.sumo_angle) % 360)
                else:
                    # VRU ID not found in either list, log warning and skip
                    self.logger.warning(f"VRU ID {vru_id} not found in vehicle or person lists, skipping")
                    continue
                    
                vrus[vru_id] = vru_state

            simulation_state.agent_details["vru"] = vrus

            # Add construction objects
            construction_objects = {}
            for cid in construction_ids:
                construction_state = AgentStateSimplified()
                construction_state.x, construction_state.y, construction_state.z = traci.vehicle.getPosition3D(cid)
                construction_state.lon, construction_state.lat = traci.simulation.convertGeo(construction_state.x, construction_state.y)
                construction_state.sumo_angle = traci.vehicle.getAngle(cid)
                construction_state.orientation = np.radians((90 - construction_state.sumo_angle) % 360)
                construction_state.speed = traci.vehicle.getSpeed(cid)
                construction_state.acceleration = traci.vehicle.getAcceleration(cid)
                construction_state.length = traci.vehicle.getLength(cid)
                construction_state.width = traci.vehicle.getWidth(cid)
                construction_state.height = traci.vehicle.getHeight(cid)
                construction_state.type = traci.vehicle.getTypeID(cid)
                construction_state.angular_velocity = 0.0
                construction_objects[cid] = construction_state
                
            simulation_state.construction_objects = construction_objects

            # Add traffic light states
            traffic_lights = {}
            for tl_id in traci.trafficlight.getIDList():
                sumo_signal = SUMOSignal()
                sumo_signal.x, sumo_signal.y = 0,0
                sumo_signal.tls = traci.trafficlight.getRedYellowGreenState(tl_id)
                tls_information = {
                    "programs": {}
                }
                tls = self.simulator.sumo_net.getTLS(tl_id)
                programs = tls.getPrograms()
                for program_id, program in programs.items():
                    # Get the program parameters
                    program_parameters = program.getParams()
                    tls_information["programs"][program_id] = {
                        "parameters": program_parameters
                    }
                sumo_signal.information = json.dumps(tls_information)
                traffic_lights[tl_id] = sumo_signal

            simulation_state.traffic_light_details = traffic_lights

            # Add construction zone shapes
            if self.construction_zone_shapes is None and simulator.env.static_adversity is not None and simulator.env.static_adversity.adversities is not None:
                self.construction_zone_shapes = {}
                for adversity in simulator.env.static_adversity.adversities:
                    if isinstance(adversity, ConstructionAdversity):
                        lane_shape = traci.lane.getShape(adversity._lane_id)
                        if lane_shape: # convert to list of lists
                            lane_shape = interpolate_by_distance(lane_shape, 2.0)
                            lane_index = int(adversity._lane_id.split("_")[-1])
                            edge_id = traci.lane.getEdgeID(adversity._lane_id)
                            if lane_index == 0:
                                # From right to left
                                direction = 1
                            elif lane_index == traci.edge.getLaneNumber(edge_id) - 1:
                                # From left to right
                                direction = -1
                            else:
                                # Middle lane, no construction zone
                                continue
                            construction_zone_shape = generate_construction_zone_shape(lane_shape, traci.lane.getWidth(adversity._lane_id), direction)
                            self.construction_zone_shapes[adversity._lane_id] = construction_zone_shape

            simulation_state.construction_zone_details = self.construction_zone_shapes
            
            # Write to Redis with expiration
            self.redis_client.set(
                f"simulation:{self.simulation_uuid}:state", simulation_state.model_dump_json()
            )
            self.redis_client.expire(
                f"simulation:{self.simulation_uuid}:state", self.key_expiry
            )
            
            # If we reach here, TeraSim is working normally
            self.error_count = 0
            self.last_successful_operation = time.time()
            return True

        except Exception as e:
            self.error_count += 1
            error_msg = str(e).lower()
            
            # Check if this is a critical error
            critical_errors = [
                "no network loaded",
                "connection lost", 
                "traci",
                "sumo",
                "simulation crashed"
            ]
            
            is_critical = any(err in error_msg for err in critical_errors)
            
            self.logger.error(f"TeraSim error #{self.error_count}: {e}")
            
            # Stop if critical error or too many consecutive errors
            if is_critical or self.error_count >= 3:
                self.logger.critical(f"TeraSim appears broken, stopping simulation")
                # Set error flag for cleanup task to handle
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:error_stop", 
                    f"terasim_error_{self.error_count}",
                    ex=300  # 5 minutes expiry
                )
                return False
                
            # Also stop if no successful operation for too long
            if time.time() - self.last_successful_operation > 300:  # 5 minutes
                self.logger.critical("TeraSim not responding for 5 minutes, stopping")
                self.redis_client.set(
                    f"simulation:{self.simulation_uuid}:error_stop", 
                    "terasim_timeout",
                    ex=300
                )
                return False
                
            return True

    def _handle_agent_command(self, command_data):
        """Handle agent control commands.
        
        Args:
            command_data (str): The agent command data.
        """
        try:
            command = AgentCommand.model_validate_json(command_data.decode("utf-8"))
            if command.agent_id != '':
                if command.agent_type not in ["vehicle", "vru"]:
                    self.logger.error(f"Invalid agent type: {command.agent_type}")
                    return False
                if command.agent_id in self.controlled_agents_each_step:
                    self.logger.debug(f"Agent {command.agent_id} is already controlled")
                    return True
                self.controlled_agents_each_step.add(command.agent_id)
                if command.command_type == "set_state":
                    # Check that exactly one of position or lonlat is present
                    has_position = "position" in command.data
                    has_lonlat = "lonlat" in command.data
                    if not (has_position ^ has_lonlat):  # XOR operation ensures exactly one is True
                        self.logger.error("Must specify exactly one of position or lonlat")
                        return False
                    if "position" in command.data:
                        x, y = command.data["position"]
                    elif "lonlat" in command.data:
                        lon, lat = command.data["lonlat"]
                        x, y = traci.simulation.convertGeo(lon, lat, fromGeo=True)
                    if command.agent_type == "vehicle":
                        traci.vehicle.moveToXY(
                            command.agent_id, "", 0, x, y, command.data.get("sumo_angle", 0), 2
                        )

                        if "speed" in command.data:
                            traci.vehicle.setPreviousSpeed(command.agent_id, command.data["speed"])
                    else:  # VRU type
                        # Check if VRU is actually a vehicle or person
                        current_vehicle_list = traci.vehicle.getIDList()
                        current_person_list = traci.person.getIDList()
                        
                        if command.agent_id in current_vehicle_list:
                            # VRU is actually a vehicle (disguised as pedestrian)
                            traci.vehicle.moveToXY(
                                command.agent_id, "", 0, x, y, command.data.get("sumo_angle", 0), 2
                            )
                            if "speed" in command.data:
                                traci.vehicle.setPreviousSpeed(command.agent_id, command.data["speed"])
                        elif command.agent_id in current_person_list:
                            # VRU is actually a person
                            traci.person.moveToXY(
                                command.agent_id, "", x, y, command.data.get("sumo_angle", 0), 2
                            )
                            if "speed" in command.data:
                                traci.person.setSpeed(command.agent_id, command.data["speed"])
                        else:
                            self.logger.error(f"VRU ID {command.agent_id} not found in vehicle or person lists")
                            return False
            

                self.logger.info(f"Agent command executed: {command_data}")
                return True

        except Exception as e:
            self.logger.error(f"Error handling agent command: {e}")
            return False

    def _reconnect_redis(self):
        """Reconnect to Redis server.

        Returns:
            bool: True if reconnection was successful, False otherwise.
        """
        try:
            self.logger.info("Attempting to reconnect to Redis...")
            self.redis_client = redis.Redis(**self.redis_config)
            self.logger.info("Successfully reconnected to Redis")
            return True
        except RedisError as e:
            self.logger.error(f"Failed to reconnect to Redis: {e}")
            return False

    def _handle_pending_agent_commands(self):
        """Handle all pending agent commands in the queue."""
        if not self._check_simulation_status():
            return
        """Handle all pending agent commands in the queue"""
        try:
            # Process up to 100 commands per step to prevent infinite loops
            for _ in range(100):
                command_data = self.redis_client.lpop(
                    f"simulation:{self.simulation_uuid}:agent_commands"
                )
                if not command_data:
                    break

                self._handle_agent_command(command_data)
        except Exception as e:
            self.logger.error(f"Error handling pending agent commands: {e}")

    def _extract_map_geometry(self, sumo_net):
        """Extract static map geometry from SUMO network."""
        map_data = {
            "lanes": [],
            "edges": [],  # Add edges for boundary calculation
            "junctions": [],
            "traffic_lights": [],
            "bounds": {
                "min_x": float('inf'),
                "max_x": float('-inf'),
                "min_y": float('inf'),
                "max_y": float('-inf')
            }
        }
        
        # Extract lane data
        for edge in sumo_net.getEdges():
            for lane in edge.getLanes():
                lane_shape = lane.getShape()
                if lane_shape:
                    # Convert to list of lists and update bounds
                    shape_list = []
                    for x, y in lane_shape:
                        shape_list.append([float(x), float(y)])
                        map_data["bounds"]["min_x"] = min(map_data["bounds"]["min_x"], x)
                        map_data["bounds"]["max_x"] = max(map_data["bounds"]["max_x"], x)
                        map_data["bounds"]["min_y"] = min(map_data["bounds"]["min_y"], y)
                        map_data["bounds"]["max_y"] = max(map_data["bounds"]["max_y"], y)
                    
                    map_data["lanes"].append({
                        "id": lane.getID(),
                        "shape": shape_list,
                        "width": float(lane.getWidth()),
                        "speed_limit": float(lane.getSpeed()),
                        "length": float(lane.getLength()),
                        "edge_id": edge.getID()
                    })
        
        # Calculate all lane boundaries for each edge
        edges_dict = {}
        for lane_data in map_data["lanes"]:
            edge_id = lane_data["edge_id"]
            if edge_id not in edges_dict:
                edges_dict[edge_id] = []
            edges_dict[edge_id].append(lane_data)
        
        # For each edge, calculate all lane boundaries
        for edge_id, lanes in edges_dict.items():
            if not lanes:
                continue
            
            # Sort lanes by their index (assuming lane IDs end with _0, _1, etc.)
            lanes.sort(key=lambda l: int(l["id"].split("_")[-1]))
            
            # Calculate boundary for each lane
            lane_boundaries = []
            
            # Helper function to calculate boundary line
            def calculate_boundary(lane_shape, lane_width, side):
                """Calculate left or right boundary of a lane.
                side: -1 for left boundary, 1 for right boundary
                """
                boundary = []
                for i, point in enumerate(lane_shape):
                    # Calculate perpendicular vector
                    if i < len(lane_shape) - 1:
                        # Use next point for direction
                        dx = lane_shape[i+1][0] - point[0]
                        dy = lane_shape[i+1][1] - point[1]
                    else:
                        # For last point, use previous point for direction
                        dx = point[0] - lane_shape[i-1][0]
                        dy = point[1] - lane_shape[i-1][1]
                    
                    length = (dx**2 + dy**2)**0.5
                    if length > 0:
                        # Perpendicular vector (left: -dy,dx; right: dy,-dx)
                        if side < 0:  # left
                            perp_x = -dy / length
                            perp_y = dx / length
                        else:  # right
                            perp_x = dy / length
                            perp_y = -dx / length
                        
                        # Offset by half lane width
                        offset = lane_width / 2
                        boundary.append([
                            point[0] + perp_x * offset,
                            point[1] + perp_y * offset
                        ])
                return boundary
            
            # Calculate boundaries for all lanes
            all_boundaries = []
            
            # Right boundary of the rightmost lane (road edge)
            if lanes:
                right_edge = calculate_boundary(lanes[0]["shape"], lanes[0]["width"], 1)
                if right_edge:
                    all_boundaries.append({
                        "points": right_edge,
                        "type": "edge",
                        "side": "right"
                    })
            
            # Boundaries between lanes
            for i in range(len(lanes) - 1):
                # Left boundary of current lane = right boundary of next lane
                lane_divider = calculate_boundary(lanes[i]["shape"], lanes[i]["width"], -1)
                if lane_divider:
                    all_boundaries.append({
                        "points": lane_divider,
                        "type": "divider",
                        "between": [lanes[i]["id"], lanes[i+1]["id"]]
                    })
            
            # Left boundary of the leftmost lane (road edge)
            if lanes:
                left_edge = calculate_boundary(lanes[-1]["shape"], lanes[-1]["width"], -1)
                if left_edge:
                    all_boundaries.append({
                        "points": left_edge,
                        "type": "edge",
                        "side": "left"
                    })
            
            if all_boundaries:
                map_data["edges"].append({
                    "id": edge_id,
                    "boundaries": all_boundaries,
                    "lanes": [l["id"] for l in lanes]
                })
        
        # Extract junction data
        for node in sumo_net.getNodes():
            if node.getType() not in ["dead_end", "rail_crossing"]:
                shape = node.getShape()
                if shape:
                    shape_list = [[float(x), float(y)] for x, y in shape]
                    coord = node.getCoord()
                    map_data["junctions"].append({
                        "id": node.getID(),
                        "shape": shape_list,
                        "position": [float(coord[0]), float(coord[1])],
                        "type": node.getType()
                    })
        
        # Extract traffic light data with actual positions
        # TODO: Fix TLS API - getNodes() doesn't exist, need to find correct method
        # Temporarily commented out to allow visualization to start
        '''
        for tls in sumo_net.getTrafficLights():
            tls_id = tls.getID()
            
            # Get controlled nodes to find traffic light positions
            controlled_nodes = tls.getNodes()
            
            for node in controlled_nodes:
                coord = node.getCoord()
                
                # Get controlled lanes for this traffic light
                controlled_lanes = []
                for connection in tls.getConnections():
                    from_lane_id = connection.getFromLane().getID()
                    to_lane_id = connection.getToLane().getID()
                    controlled_lanes.append({
                        "from": from_lane_id,
                        "to": to_lane_id
                    })
                
                map_data["traffic_lights"].append({
                    "id": tls_id,
                    "position": [float(coord[0]), float(coord[1])],
                    "node_id": node.getID(),
                    "controlled_lanes": controlled_lanes
                })
        '''
        
        return map_data

    def _start_streamlit_service(self):
        """Start the Dash visualization service."""
        try:
            # Path to Dash app (now using Dash by default)
            viz_app_path = Path(__file__).parent / "dash_viz_app.py"
            
            if not viz_app_path.exists():
                self.logger.error(f"Dash app not found at {viz_app_path}")
                return
            
            # Start Dash process
            cmd = [
                "python", str(viz_app_path),
                "--simulation_uuid", self.simulation_uuid,
                "--redis_host", self.redis_config.get("host", "localhost"),
                "--redis_port", str(self.redis_config.get("port", 6379)),
                "--port", str(self.viz_port),
                "--update_interval", str(1.0 / self.viz_update_freq)  # Convert frequency to interval
            ]
            
            self.viz_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.logger.info(
                f"🎨 Dash visualization started at http://localhost:{self.viz_port}"
            )
            
            # Give it a moment to start
            time.sleep(2)
            
            # Check if process started successfully
            if self.viz_process.poll() is not None:
                stdout, stderr = self.viz_process.communicate()
                self.logger.error(f"Dash process failed to start: {stderr}")
                raise RuntimeError(f"Dash process failed to start: {stderr}")
                
        except Exception as e:
            self.logger.error(f"Failed to start visualization service: {e}")


