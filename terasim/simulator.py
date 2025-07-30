from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Optional

import numpy as np
import sumolib
from loguru import logger
from traci import constants as tc

import terasim
import terasim.utils as utils
from terasim.agent.agent import Agent, AgentInitialInfo

from .overlay import has_libsumo, traci
from .pipeline import Pipeline, PipelineElement


class Context:
    def __init__(self) -> None:
        self.plugins = []


class Simulator(object):
    """
    Simulator deals everything about synchronization of states between SUMO and python script
    """

    def __init__(
        self,
        sumo_config_file_path: str or Path,
        sumo_net_file_path: str or Path,
        num_tries: int = 10,
        gui_flag: bool = False,
        output_path: str or Path = None,
        sumo_output_file_types: list = None,
        step_length: float = None,
        realtime_flag: bool = False,
        seed: int = None,
        additional_sumo_args: str or list = None,
        traffic_scale: float = 1.0,
    ):
        """Initialize the simulator.
        
        Args:
            sumo_config_file_path (str or Path): SUMO configuration file path.
            sumo_net_file_path (str or Path): SUMO network file path.
            num_tries (int, optional): Number of tries. Defaults to 10.
            gui_flag (bool, optional): GUI flag. Defaults to False.
            output_path (str or Path, optional): Output path. Defaults to None.
            sumo_output_file_types (list, optional): SUMO output file types. Defaults to None.
            step_length (float, optional): Step length. Defaults to None.
            realtime_flag (bool, optional): Realtime flag. Defaults to False.
            additional_sumo_args (str or list, optional): Additional SUMO arguments. Defaults to None.
            seed (int, optional): Seed. Defaults to None.
            traffic_scale (float, optional): Traffic scale factor. Values > 1.0 increase traffic, < 1.0 decrease traffic. Defaults to 1.0.
        """
        self.sumo_net_file_path = Path(sumo_net_file_path)
        self.sumo_config_file_path = Path(sumo_config_file_path)
        assert self.sumo_net_file_path.exists(), "sumo_net_file_path does not exist"
        assert self.sumo_config_file_path.exists(), "sumo_config_file_path does not exist"
        self.sumo_net = sumolib.net.readNet(str(self.sumo_net_file_path), withInternal=True, withPrograms=True)

        self.gui_flag = gui_flag
        self.sumo_binary = "sumo-gui" if gui_flag else "sumo"
        self.running = False
        self.num_tries = num_tries
        self.sublane_flag = False
        self.step_length = step_length
        self.realtime_flag = realtime_flag
        self.traffic_scale = traffic_scale
        assert (
            isinstance(additional_sumo_args, list)
            or isinstance(additional_sumo_args, str)
            or additional_sumo_args is None
        ), "additional_sumo_args should be a list of strings or a string"
        self.additional_sumo_args = (
            [additional_sumo_args]
            if isinstance(additional_sumo_args, str)
            else additional_sumo_args
        )
        if seed is not None:
            self.additional_sumo_args += ["--seed", str(seed)]
        if output_path is not None:
            self.output_path = Path(output_path)
            self.output_path.mkdir(parents=True, exist_ok=True)
        else:
            self.output_path = None
            print("Warning: output_path is not specified. No output will be generated.")
        self.sumo_output_file_types = (
            sumo_output_file_types if sumo_output_file_types is not None else []
        )
        self._plugin_list = []
        self.ctx = {}  # context for the execution pipeline

        # pipelines
        self.start_pipeline = Pipeline("start_pipeline", [])  # params: simulator, ctx
        self.step_pipeline = Pipeline(
            "step_pipeline",  # params: simulator, ctx
            [
                PipelineElement(
                    "record_step_start_time",
                    self.record_step_start_time,
                    priority=-10000,
                ),
                PipelineElement("sumo_step", self.sumo_step, priority=10),
                PipelineElement(
                    "compensate_step_end_time",
                    self.compensate_step_end_time,
                    priority=10000,
                ),
            ],
        )
        self.stop_pipeline = Pipeline("stop_pipeline", [])  # params: simulator, ctx
        self._add_vehicle_to_sim = Pipeline(
            "add_vehicle_pipeline",  # params: agent, init_info. If init_info is None, then the initial info should be inferred from traci.
            [PipelineElement("sumo_add", self._add_vehicle_to_sumo)],
        )
        self._remove_vehicle_from_sim = Pipeline(
            "remove_vehicle_pipeline",  # params: agent
            [PipelineElement("sumo_remove", self._remove_vehicle_from_sumo)],
        )

    def bind_env(self, env: terasim.envs.base.BaseEnv):
        """Combine the environment with the simulator

        Args:
            env (BaseEnv): Simulation environment
        """
        self.env = env
        self.env.simulator = self
        self.start_pipeline.hook("env_start", env._start, priority=0)
        self.step_pipeline.hook("env_step", env._step, priority=0)
        self.stop_pipeline.hook("env_stop", env._stop, priority=0)

    def start(self):
        """Start SUMO simulation or initialize environment.
        """
        sumo_cmd = [
            self.sumo_binary,
            "-c",
            str(self.sumo_config_file_path),
        ]
        if self.step_length is not None:
            sumo_cmd += ["--step-length", str(self.step_length)]

        if self.sumo_output_file_types is not None:
            if "traj" in self.sumo_output_file_types:
                filename = str(self.output_path / "traj.xml")
                sumo_cmd += ["--amitran-output", filename]
            if "fcd_all" in self.sumo_output_file_types:
                filename = str(self.output_path / "fcd_all.xml")
                sumo_cmd += ["--fcd-output", filename, "--fcd-output.acceleration"]
            if "fcd" in self.sumo_output_file_types:
                filename = str(self.output_path / "fcd.xml")
                sumo_cmd += [
                    "--fcd-output",
                    filename,
                    "--fcd-output.acceleration",
                    "--device.fcd.explicit",
                    "AV",
                    "--device.fcd.radius",
                    "200",
                ]
            if "lc" in self.sumo_output_file_types:
                filename = str(self.output_path / "lc.xml")
                sumo_cmd += ["--lanechange-output", filename]
            if "collision" in self.sumo_output_file_types:
                filename = str(self.output_path / "collision.xml")
                sumo_cmd += ["--collision-output", filename]
            if "tripinfo" in self.sumo_output_file_types:
                filename = str(self.output_path / "tripinfo.xml")
                sumo_cmd += [
                    "--tripinfo-output",
                    filename,
                    "--tripinfo-output.write-unfinished",
                ]
        # log file
        if self.output_path is not None:
            sumo_cmd += ["-l", str(self.output_path / "run.log")]
        # traffic scale
        if self.traffic_scale != 1.0:
            sumo_cmd += ["--scale", str(self.traffic_scale)]
        sumo_cmd += self.additional_sumo_args if self.additional_sumo_args is not None else []
        self.sumo_cmd = sumo_cmd
        if has_libsumo:
            traci.start(sumo_cmd)
        else:
            traci.start(sumo_cmd, numRetries=self.num_tries)

        self.start_pipeline(self, self.ctx)
        self.running = True

    def get_vehicle_route_lanes(self, vehID: str):
        """Get list of lanes in the route of the vehicle.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            list(str): A list of list(sumo lane).
        """
        route = traci.vehicle.getRoute(vehID)
        list_lanes = []
        for edgeID in route:
            sumo_edge = self.sumo_net.getEdge(edgeID)
            list_lanes.append(sumo_edge.getLanes())
        return list_lanes

    def add_plugin(self, plugin):
        """Plugin a plugin to the simulator.

        Args:
            plugin (Plugin): Plugin object.
        """
        self._plugin_list.append(plugin)
        plugin.inject(self, self.ctx)

    @property
    def plugins(self):
        """Get the plugin list.

        Returns:
            list: List of plugins.
        """
        return self._plugin_list

    def step(self):
        """Make a simulation step.
        """
        self.step_pipeline(self, self.ctx)

    def get_all_vehicle_information(self, simulator, ctx):
        """Fetch all vehicle information.

        Args:
            simulator (Simulator): Simulator object.
            ctx (dict): Context information.

        Returns:
            dict: Vehicle information dictionary.
        """
        raise NotImplementedError()

    def sumo_step(self, simulator, ctx):
        """Make a simulation step.
        
        Args:
            simulator (Simulator): Simulator object.
            ctx (dict): Context information.
        """
        traci.simulationStep()

    def record_step_start_time(self, simulator, ctx):
        """Record the starting time of the step.
        
        Args:
            simulator (Simulator): Simulator object.
            ctx (dict): Context information.
        """
        self.step_start_time = time.time()

    def compensate_step_end_time(self, simulator, ctx):
        """Compensate for real world time.
        
        Args:
            simulator (Simulator): Simulator object.
            ctx (dict): Context information.
        """
        if self.realtime_flag:
            step_time = time.time() - self.step_start_time
            sim_step_size = utils.get_step_size()
            if step_time < sim_step_size:
                time.sleep(sim_step_size - step_time)
            else:
                logger.critical(step_time)

    def run(self):
        """Run the specific episode.
        """
        self.start()
        while True:
            self.step()
            if self.running is False:
                break
        self.stop()

    def stop(self):
        """Close SUMO simulation.
        """
        self.stop_pipeline(self, self.ctx)
        traci.close()

    def _add_vehicle_to_sumo(self, veh: Agent, init_info: Optional[AgentInitialInfo]):
        """Generate a vehicle in SUMO network.
        
        Args:
            veh (Agent): Vehicle object.
            init_info (AgentInitialInfo, optional): Initial information of the vehicle. Defaults to None.
        """
        if init_info is None:
            return

        if has_libsumo:
            traci.vehicle.add(
                veh.id,
                init_info.route,
                typeID=init_info.type,
                departSpeed=str(init_info.depart.speed),
            )
        else:
            traci.vehicle.add(
                veh.id,
                init_info.route,
                typeID=init_info.type,
                depart=init_info.depart.time,
                departLane=init_info.depart.lane,
                departPos=init_info.depart.position,
                departSpeed=init_info.depart.speed,
                arrivalLane=init_info.arrive.lane,
                arrivalPos=init_info.arrive.position,
                arrivalSpeed=init_info.arrive.speed,
                fromTaz=init_info.from_taz,
                toTaz=init_info.to_taz,
                line=init_info.line,
                personCapacity=init_info.person_capacity,
                personNumber=init_info.person_number,
            )
        if init_info.depart.lane_id is not None:
            traci.vehicle.moveTo(veh.id, init_info.depart.lane_id, init_info.depart.position)

    def _delete_all_vehicles_in_sumo(self):
        """Delete all vehicles in the network.
        """
        for vehID in traci.vehicle.getIDList():
            traci.vehicle.remove(vehID)

    def track_vehicle_gui(self, vehID="AV"):
        """Track specific vehicle in GUI.

        Args:
            vehID (str, optional): Vehicle ID. Defaults to "AV".
        """
        traci.gui.trackVehicle(viewID="View #0", vehID=vehID)

    def get_road_ID(self, vehID):
        """Get ID of the road where the vehicle drives on.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            str: Road ID.
        """
        return traci.vehicle.getRoadID(vehID)

    def changeTarget(self, vehID, edgeID):
        """Change the target edge of the vehicle.

        Args:
            vehID (str): Vehicle ID.
            edgeID (str): Edge ID.
        """
        traci.vehicle.changeTarget(vehID, edgeID)

    def detected_crash(self):
        """Detect the crash happened in the last time step.

        Returns:
            bool: True if a collision happenes in the simulation. False if no collision happens.
        """
        colli = traci.simulation.getCollidingVehiclesIDList()
        return colli

    def detect_vehicle_num(self):
        """Determine the vehicle number in the simulation.

        Returns:
            int: Number of vehicles.
        """
        return traci.simulation.getMinExpectedNumber()

    def get_available_lanes_id(self, edge_id, veh_type="passenger"):
        """Get available lanes for the specific edge and vehicle type.

        Args:
            edge_id (str): Edge ID.
            veh_type (str, optional): Vehicle type. Defaults to 'passenger'.

        Returns:
            list(str): List of lane ID.
        """
        lane_num = self.get_edge_lane_number(edge_id)
        lanes_id = []
        for i in range(lane_num):
            lane_id = edge_id + "_" + str(i)
            if veh_type not in self.get_lane_disallowed(lane_id):
                lanes_id.append(lane_id)
        return lanes_id

    def get_available_lanes(self, edge_id=None):
        """Get the available lanes in the sumo network.

        Args:
            edge_id (str, optional): Edge ID. Defaults to None.

        Returns:
            list(sumo lane object): Possible lanes to insert vehicles
        """
        if edge_id == None:
            sumo_edges = self.sumo_net.getEdges()
        else:
            sumo_edges = [self.sumo_net.getEdge(edge_id)]
        available_lanes = []
        for edge in sumo_edges:
            for lane in edge.getLanes():
                available_lanes.append(lane)
        return available_lanes

    def get_av_travel_distance(self):
        """Get the travel distance of AV.

        Returns:
            float: Travel distance.
        """
        return traci.vehicle.getDistance("AV")

    def get_edge_dist(
        self, first_edge_id, first_lane_position, second_edge_id, second_lane_position
    ):
        """Get distance between two edge position.

        Args:
            first_edge_id (str): Edge ID of the first edge.
            first_lane_position (float): Lane position on the first edge.
            second_edge_id (str): Edge ID of the second edge.
            second_lane_position (float): Lane position on the second edge.

        Returns:
            float: Distance between two positions.
        """
        return traci.simulation.getDistanceRoad(
            first_edge_id,
            first_lane_position,
            second_edge_id,
            second_lane_position,
            True,
        )

    def get_edge_length(self, edgeID):
        """Get the length of the edge.

        Args:
            edgeID (str): Edge ID.

        Returns:
            float: Edge length.
        """
        lane_id = edgeID + "_0"
        lane_length = self.get_lane_length(lane_id)
        return lane_length

    def get_lane_length(self, laneID):
        """Get the lane length.

        Args:
            laneID (str): Lane ID.

        Returns:
            float: Lane length.
        """
        return traci.lane.getLength(laneID)

    def get_lane_width(self, laneID):
        """Get lane width.

        Args:
            laneID (str): Lane ID.

        Returns:
            float: Lane width in m.
        """
        return traci.lane.getWidth(laneID)

    def get_lane_links(self, laneID):
        """Get successor lanes together with priority, open and foe for each link.

        Args:
            laneID (str): Lane ID.

        Returns:
            list: Successor lane information.
        """
        return traci.lane.getLinks(laneID)

    @staticmethod
    def get_vehicle_lane_adjacent(vehID, direction):
        """Get whether the vehicle is allowed to drive on the adjacent lane.

        Args:
            vehID (str): Vehicle ID.
            direction (int): 1 represents left, while -1 represents right.

        Returns:
            bool: Whether the vehicle can drive on the specific lane.
        """
        if direction not in [-1, 1]:
            raise ValueError("Unknown direction input:" + str(direction))
        lane_index = Simulator.get_vehicle_lane_index(vehID)
        new_lane_index = lane_index + direction
        edge_id = Simulator.get_vehicle_roadID(vehID)
        lane_num = Simulator.get_edge_lane_number(edge_id)
        if new_lane_index < 0 or new_lane_index >= lane_num:
            # Adjacent lane does not exist.
            return False
        new_lane_id = edge_id + "_" + str(new_lane_index)
        veh_class = traci.vehicle.getVehicleClass(vehID)
        disallowed = Simulator.get_lane_disallowed(new_lane_id)
        return not veh_class in disallowed

    @staticmethod
    def get_vehicle_lane_index(vehID):
        """Get vehicle lane index.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            int: Lane index.
        """
        return traci.vehicle.getLaneIndex(vehID)

    @staticmethod
    def get_lane_disallowed(laneID):
        """Get disallowed vehicle class of the lane.

        Args:
            laneID (str): Lane ID.

        Returns:
            list(str): Disallowed vehicle class, such as "passenger".
        """
        return traci.lane.getDisallowed(laneID)

    def get_route_edges(self, routeID):
        """Get edges of the route.

        Args:
            routeID (str): Route ID.

        Returns:
            list(str): A list of edge ID.
        """
        return traci.route.getEdges(routeID)

    def _remove_vehicle_from_sumo(self, vehicle):
        """Remove the vehicle from the simulation. Unsubscribe all information as well

        Args:
            vehicle (Vehicle): Vehicle object.
        """
        # unsubscribe the vehicle
        vehID = vehicle.id
        traci.vehicle.unsubscribe(vehID)
        # remove the vehicle from the simulation
        traci.vehicle.remove(vehID)

    def set_vehicle_emegency_deceleration(self, vehID, decel):
        """Set the emergency deceleration of the vehicle.

        Args:
            vehID (str): Vehicle ID.
            decel (float): Deceleration value.
        """
        traci.vehicle.setEmergencyDecel(vehID, decel)

    @staticmethod
    def subscribe_vehicle_ego(vehID):
        """Subscribe to store vehicle's ego information.

        Args:
            vehID (str): Vehicle ID.
        """
        traci.vehicle.subscribe(
            vehID,
            [
                tc.VAR_LENGTH,
                tc.VAR_POSITION,
                tc.VAR_SPEED,
                tc.VAR_LANE_INDEX,
                tc.VAR_ANGLE,
                tc.VAR_POSITION3D,
                tc.VAR_EDGES,
                tc.VAR_LANEPOSITION,
                tc.VAR_LANEPOSITION_LAT,
                tc.VAR_SPEED_LAT,
                tc.VAR_ROAD_ID,
                tc.VAR_ACCELERATION,
            ],
        )

    @staticmethod
    def subscribe_vehicle_surrounding(vehID, max_obs_range=120):
        """Subscribe to store vehicle's ego and surrounding information.

        Args:
            vehID (str): Vehicle ID.
        """
        Simulator.subscribe_vehicle_ego(vehID)
        traci.vehicle.subscribeContext(
            vehID,
            tc.CMD_GET_VEHICLE_VARIABLE,
            max_obs_range,
            [
                tc.VAR_LENGTH,
                tc.VAR_POSITION,
                tc.VAR_SPEED,
                tc.VAR_LANE_INDEX,
                tc.VAR_ANGLE,
                tc.VAR_POSITION3D,
                tc.VAR_ROAD_ID,
                tc.VAR_ACCELERATION,
                tc.VAR_LANE_ID,
            ],
        )
        traci.vehicle.addSubscriptionFilterLanes(
            [-2, -1, 0, 1, 2],
            noOpposite=True,
            downstreamDist=max_obs_range,
            upstreamDist=max_obs_range,
        )

    @staticmethod
    def subscribe_vehicle_all_information(vehID, max_obs_range=120):
        """Subscribe to store vehicle's complete information.

        Args:
            vehID (str): Vehicle ID.
        """
        Simulator.subscribe_vehicle_ego(vehID)
        traci.vehicle.subscribeContext(
            vehID,
            tc.CMD_GET_VEHICLE_VARIABLE,
            max_obs_range,
            [
                tc.VAR_LENGTH,
                tc.VAR_POSITION,
                tc.VAR_SPEED,
                tc.VAR_LANE_INDEX,
                tc.VAR_ANGLE,
                tc.VAR_POSITION3D,
                tc.VAR_EDGES,
                tc.VAR_LANEPOSITION,
                tc.VAR_LANEPOSITION_LAT,
                tc.VAR_SPEED_LAT,
                tc.VAR_ROAD_ID,
                tc.VAR_ACCELERATION,
                tc.VAR_LANE_ID,
            ],
        )
        traci.vehicle.addSubscriptionFilterLanes(
            [-2, -1, 0, 1, 2],
            noOpposite=True,
            downstreamDist=max_obs_range,
            upstreamDist=max_obs_range,
        )

    def unsubscribe_vehicle(self, vehID):
        """Unsubscribe the vehicle information.

        Args:
            vehID (str): Vehicle ID.
        """
        traci.vehicle.unsubscribe(vehID)

    def get_vehicle_context_subscription_results(self, vehID):
        """Get subscription results of the context information.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            dict: Context subscription results.
        """
        return traci.vehicle.getContextSubscriptionResults(vehID)

    def get_vehicle_min_expected_number(self):
        """Get the vehicle number in the simulation plus the number of vehicles waiting to start.

        Returns:
            int: Number of vehicles in the simulation and vehicles waiting to start.
        """
        return traci.simulation.getMinExpectedNumber()

    def get_vehicle_speedmode(self, vehID):
        """Get speed mode of the vehicle.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            int: Speed mode.
        """
        return traci.vehicle.getSpeedMode(vehID)

    def get_colliding_vehicle_number(self):
        """Get the number of cars involved in a collision during the simulation.

        Returns:
            int: Return number of vehicles involved in a collision (typically 2 per collision).
        """
        return traci.simulation.getCollidingVehiclesNumber()

    def get_colliding_vehicles(self):
        """Get the IDs of cars involved in a collision during the simulation.

        Returns:
            list(str): Return the first list of vehicle IDs involved in a collision (typically 2 per collision).
        """
        return traci.simulation.getCollidingVehiclesIDList()

    def get_vehicle_length(self, vehID):
        """Get vehicle length.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Vehicle length in m.
        """
        return traci.vehicle.getLength(vehID)

    def get_vehicle_mingap(self, vehID):
        """Get vehicle minimum gap.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Vehicle minimum gap in m.
        """
        return traci.vehicle.getMinGap(vehID)

    def get_vehicle_acc(self, vehID):
        """Get the acceleration in m/s^2 of the named vehicle within the last step.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Vehicle acceleration [m/s^2].
        """
        return traci.vehicle.getAcceleration(vehID)

    def get_vehicle_maxacc(self, vehID):
        """Get vehicle maximum acceleration.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Maximum acceleration in m/s^2.
        """
        return traci.vehicle.getAccel(vehID)

    def get_vehicle_could_change_lane(self, vehID, direction):
        """Check whehther the vehicle could change lane in the specific direction.

        Args:
            vehID (str): Vehicle ID.
            direction (int): 1 represents "left" and -1 represents "right".

        Returns:
            bool: Whehther the vehicle chould change lane.
        """
        return traci.vehicle.couldChangeLane(vehID, direction)

    def get_vehicle_lane_position(self, vehID):
        """Get the lane position of the vehicle.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Lane position in m.
        """
        return traci.vehicle.getLanePosition(vehID)

    @staticmethod
    def get_edge_lane_number(edgeID):
        """Get lane number of the edge.

        Args:
            edgeID (str): Edge ID.

        Returns:
            int: Lane number.
        """
        return traci.edge.getLaneNumber(edgeID)

    def get_vehicle_lane_number(self, vehID):
        """Get lane number of the edge where the vehicle is driving on.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            int: Lane number.
        """
        return self.get_edge_lane_number(self.get_vehicle_roadID(vehID))

    def get_vehicle_maxdecel(self, vehID):
        """Get vehicle maximum deceleration.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Maximum deceleration.
        """
        return traci.vehicle.getDecel(vehID)

    def get_vehicle_maneuver_pdf(self, vehID):
        """Get vehicle maneuver possibility distribution.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            array(float): Possibility distribution of vehicle maneuver.
        """
        pdf_tuple = traci.vehicle.getNDDProb(vehID)  # tuple of strings
        pdf_array = np.zeros(len(pdf_tuple), dtype=float)  # array of floats
        for i in range(len(pdf_tuple)):
            pdf_array[i] = float(pdf_tuple[i])
        return pdf_array

    @staticmethod
    def get_vehicle_roadID(vehID):
        """Get road ID where the vehicle is driving on.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            str: Road ID.
        """
        return traci.vehicle.getRoadID(vehID)

    def get_vehicle_type(self, vehID):
        """Get vehicle type ID.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            str: Type ID.
        """
        return traci.vehicle.getTypeID(vehID)

    def get_vehicle_laneID(self, vehID):
        """Get lane ID where the vehicle is driving on.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            str: Lane ID.
        """
        return traci.vehicle.getLaneID(vehID)

    def get_vehicle_lane_width(self, vehID):
        """Get lane width where the vehicle is driving on.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Lane width in m.
        """
        laneID = self.get_vehicle_laneID(vehID)
        return self.get_lane_width(laneID)

    def get_vehicle_distance_to_edge(self, veh_id, edge_id, edge_position):
        """Get distance from the vehicle to the edge.

        Args:
            veh_id (str): Vehicle ID.
            edge_id (str): Edge ID.
            edge_position (float): Edge position.

        Returns:
            float: Distance between vehicle and edge.
        """
        second_edge_id = self.get_vehicle_roadID(veh_id)
        first_edge_id = edge_id
        second_lane_position = min(
            self.get_vehicle_lane_position(veh_id), self.get_edge_length(second_edge_id)
        )
        first_lane_position = 0
        return traci.simulation.getDistanceRoad(
            first_edge_id,
            first_lane_position,
            second_edge_id,
            second_lane_position,
            True,
        )

    def get_vehicles_dist_road(self, first_veh_id, second_veh_id):
        """Get the distance between two vehicles along the network.

        Args:
            first_veh_id (str): First vehicle ID.
            second_veh_id (str): Second vehicle ID.

        Returns:
            float: Vehicles distance.
        """

        first_edge_id = self.get_vehicle_roadID(first_veh_id)
        second_edge_id = self.get_vehicle_roadID(second_veh_id)
        first_lane_position = min(
            self.get_vehicle_lane_position(first_veh_id),
            self.get_edge_length(first_edge_id),
        )
        second_lane_position = min(
            self.get_vehicle_lane_position(second_veh_id),
            self.get_edge_length(second_edge_id),
        )
        return traci.simulation.getDistanceRoad(
            first_edge_id,
            first_lane_position,
            second_edge_id,
            second_lane_position,
            True,
        )

    def get_vehicles_dist(self, first_veh_pos, second_veh_pos):
        """Get distance between two vehicles.

        Args:
            first_veh_pos (tuple(float,float)): 2D position of the first vehicle.
            second_veh_pos (tuple(float,float)): 2D position of the second vehicle.

        Returns:
            float: Longitudinal distance between two vehicles.
        """
        return traci.simulation.getDistance2D(
            first_veh_pos[0],
            first_veh_pos[1],
            second_veh_pos[0],
            second_veh_pos[1],
            False,
            True,
        )

    def get_vehicles_relative_lane_index(self, ego_vehID, front_vehID):
        """Get relative lane index for two vehicles.

        Args:
            ego_vehID (str): Ego vehicle ID.
            front_vehID (str): Front vehicle ID.

        Returns:
            int: Relative lane index.
        """
        ego_laneID = self.get_vehicle_laneID(ego_vehID)
        ego_roadID = self.get_vehicle_roadID(ego_vehID)
        laneID = ego_laneID
        roadID = ego_roadID
        front_laneID = self.get_vehicle_laneID(front_vehID)
        front_roadID = self.get_vehicle_roadID(front_vehID)
        if front_roadID[0] == ":":
            links = self.get_lane_links(front_laneID)
            if len(links) > 1:
                print(
                    "WARNING: Can't locate vehicles " + str(ego_vehID) + " and " + str(front_vehID)
                )
                return 3
            front_laneID = links[0][0]
            front_roadID = traci.lane.getEdgeID(front_laneID)
        front_lane_index = int(front_laneID.split("_")[-1])
        it = 0
        while it < 10:
            if front_roadID == roadID:
                lane_index = int(laneID.split("_")[-1])
                return front_lane_index - lane_index
            links = self.get_lane_links(laneID)
            # print(links)
            if len(links) > 1:
                print(
                    "WARNING: Can't locate vehicles " + str(ego_vehID) + " and " + str(front_vehID)
                )
                return 3
            else:
                laneID = links[0][0]
                roadID = traci.lane.getEdgeID(laneID)
            it += 1
        print(
            "WARNING: Can't find relative lane index for vehicles "
            + str(ego_vehID)
            + " and "
            + str(front_vehID)
        )
        return 3

    def get_departed_vehID_list(self):
        """Get ID list of vehicles entering the network in the last time step.

        Returns:
            list(str): List of ID of vehicles entering the network in the last time step.
        """
        return traci.simulation.getDepartedIDList()

    def get_vehicle_type_id(self, veh_id):
        """Get the type ID of the vehicle.

        Args:
            veh_id (str): Vehicle ID.

        Returns:
            str: Type ID.
        """
        return traci.vehicle.getTypeID(veh_id)

    def get_arrived_vehID_list(self):
        """Get ID list of vehicles arriving at the final edge in the last time step.

        Returns:
            list(str): List of ID of vehicles leaving the network in the last time step.
        """
        return traci.simulation.getArrivedIDList()

    def get_vehID_list(self):
        """Get ID list of vehicles currently in the network.

        Returns:
            list(str): List of ID of vehicles in the sumo network in the current time step.
        """
        return traci.vehicle.getIDList()

    def get_vehicle_speed(self, vehID):
        """Get the vehicle speed within the last step.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Vehicle speed in m/s.
        """
        return traci.vehicle.getSpeed(vehID)

    def get_vehicle_lateral_speed(self, vehID):
        """Get the lateral speed of the vehicle.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            float: Later speed of the specified vehicle.
        """
        return traci.vehicle.getLateralSpeed(vehID)

    def get_vehicle_position(self, vehID):
        """Get the position of the vehicle.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            tuple(float, float): Position in X,Y coordinates.
        """
        return traci.vehicle.getPosition(vehID)

    def get_vehicle_lane_number(self, vehID):
        """Get the number of lanes in the edge of the vehicle.

        Args:
            vehID (str): Vehicle ID.

        Returns:
            int: Number of lanes.
        """
        return traci.edge.getLaneNumber(traci.vehicle.getRoadID(vehID))

    def set_zoom(self, zoom):
        """Set the current zoom factor for the given view.

        Args:
            zoom (float): Zoom factor.
        """
        traci.gui.setZoom(viewID="View #0", zoom=zoom)

    def set_vehicle_color(self, vehID, rgb):
        """Set the color of a vehicle to separate it from others.

        Args:
            vehID (str): Vehicle ID.
            rgb (tuple(int, int, int, int)): RGB code of the color, i.e. (255,0,0) for the color red. The fourth component (alpha) is optional.
        """
        traci.vehicle.setColor(vehID, rgb)

    def set_vehicle_max_lateralspeed(self, vehID, lat_max_v):
        """Set the maximum lateral speed of vehicle.

        Args:
            vehID (str): Vehicle ID.
            lat_max_v (float): Maximum lateral speed.
        """
        traci.vehicle.setMaxSpeedLat(vehID, lat_max_v)

    def change_vehicle_speed(self, vehID, acceleration, duration=1.0):
        """Fix the acceleration of a vehicle to be a specified value in the specified duration.

        Args:
            vehID (str): Vehicle ID
            acceleration (float): Specified acceleration of vehicle.
            duration (float, optional): Specified time interval to fix the acceleration in s. Defaults to 1.0.
        """
        # traci.vehicle.slowDown take duration + deltaT to reach the desired speed
        init_speed = traci.vehicle.getSpeed(vehID)
        final_speed = init_speed + acceleration * (utils.get_step_size() + duration)
        if final_speed < 0:
            final_speed = 0
        traci.vehicle.slowDown(vehID, final_speed, duration)

    def _cal_lateral_maxSpeed(self, vehID, lane_width, time=1.0):
        """Calculate the maximum lateral speed for lane change maneuver.

        Args:
            vehID (str): Vehicle ID.
            lane_width (float): Width of the lane.
            time (float, optional): Specified time interval to complete the lane change maneuver in s. Defaults to 1.0.

        Raises:
            ValueError: If the maximum lateral acceleration of the vehicle is too small, it is impossible to complete the lane change maneuver in the specified duration.

        Returns:
            float: Maximum lateral speed aiming to complete the lane change behavior in the specified time duration.
        """
        # accelerate laterally to the maximum lateral speed and maintain
        # v^2 - b*v + c = 0
        lat_acc = float(traci.vehicle.getParameter(vehID, "laneChangeModel.lcAccelLat"))
        b, c = lat_acc * (time), lat_acc * lane_width
        delta_power = b**2 - 4 * c
        if delta_power >= 0:
            lat_max_v = (-math.sqrt(delta_power) + b) / 2
        else:
            raise ValueError("The lateral maximum acceleration is too small.")
        return lat_max_v

    def _cal_lateral_distance(self, vehID, direction):
        """Calculate lateral distance to the target lane for a complete lane change maneuver.

        Args:
            vehID (str): Vehicle ID.
            direction (str): Direction, i.e. "left" and "right".

        Raises:
            ValueError: Unknown lane id.
            ValueError: Unknown direction.

        Returns:
            float: Distance in m.
        """
        origin_lane_id = traci.vehicle.getLaneID(vehID)
        edge_id = traci.vehicle.getRoadID(vehID)
        lane_index = int(origin_lane_id.split("_")[-1])
        origin_lane_width = traci.lane.getWidth(origin_lane_id)
        if direction == "left":
            target_lane_id = edge_id + "_" + str(lane_index + 1)
            try:
                target_lane_width = traci.lane.getWidth(target_lane_id)
            except:
                raise ValueError(
                    "Unknown lane id: " + target_lane_id + " in the lane change maneuver."
                )
            latdist = (origin_lane_width + target_lane_width) / 2
        elif direction == "right":
            target_lane_id = edge_id + "_" + str(lane_index - 1)
            try:
                target_lane_width = traci.lane.getWidth(target_lane_id)
            except:
                raise ValueError(
                    "Unknown lane id: " + target_lane_id + " in the lane change maneuver."
                )
            latdist = -(origin_lane_width + target_lane_width) / 2
        else:
            raise ValueError("Unknown direction for lane change command")
        return latdist

    def change_vehicle_sublane_dist(self, vehID, latdist, duration):
        """Change the lateral position of the vehicle.

        Args:
            vehID (str): Vehicle ID.
            latdist (float): Desired lateral distance.
            duration (float): Change duration.
        """
        lat_max_v = self._cal_lateral_maxSpeed(vehID, abs(latdist), duration)
        traci.vehicle.setMaxSpeedLat(vehID, lat_max_v)
        traci.vehicle.changeSublane(vehID, latdist)

    def change_vehicle_lane(self, vehID, direction, duration=1.0):
        """Force a vehicle to complete the lane change maneuver in the duration.

        Args:
            vehID (str): Vehicle ID.
            direction (str): Choose from "LANE_LEFT" and "LANE_RIGHT".
            duration (float, optional): Specified time interval to complete the lane change behavior. Defaults to 1.0.

        Raises:
            ValueError: Direction is neither "LANE_LEFT" nor "LANE_RIGHT".
        """
        if self.sublane_flag:
            latdist = self._cal_lateral_distance(vehID, direction)
            lat_max_v = self._cal_lateral_maxSpeed(vehID, abs(latdist), duration)
            traci.vehicle.setMaxSpeedLat(vehID, lat_max_v)
            traci.vehicle.changeSublane(vehID, latdist)
        else:
            if direction == "left":
                indexOffset = 1
            elif direction == "right":
                indexOffset = -1
            else:
                raise ValueError("Unknown direction for lane change command")
            traci.vehicle.changeLaneRelative(vehID, indexOffset, utils.get_step_size())

    def change_vehicle_position(
        self, vehID, position, edgeID="", lane=-1, angle=-1073741824.0, keepRoute=1
    ):
        """Move the vehicle to the given coordinates and force it's angle to the given value (for drawing).

        Args:
            vehID (str): Vehicle ID.
            position (tuple(float, float)): The specified x,y coordinates.
            edgeID (str, optional): Edge ID. Defaults to "".
            lane (int, optional): Lane index. Defaults to -1.
            angle (float, optional): angle (float, optional): Specified angle of vehicle. If the angle is set to INVALID_DOUBLE_VALUE, the vehicle assumes the natural angle of the edge on which it is driving. Defaults to -1073741824.0.
            keepRoute (int, optional): If keepRoute is set to 1, the closest position within the existing route is taken. If keepRoute is set to 0, the vehicle may move to any edge in the network but it's route then only consists of that edge. If keepRoute is set to 2, the vehicle has all the freedom of keepRoute=0 but in addition to that may even move outside the road network. Defaults to 1.
        """
        x = position[0]
        y = position[1]
        traci.vehicle.moveToXY(vehID, edgeID, lane, x, y, angle, keepRoute)

    def subscribe_signal(self, tlsID):
        """Subscribe to the specified traffic light.
            TL_BLOCKING_VEHICLES = 37
            TL_COMPLETE_DEFINITION_RYG = 43
            TL_COMPLETE_PROGRAM_RYG = 44
            TL_CONTROLLED_JUNCTIONS = 42
            TL_CONTROLLED_LANES = 38
            TL_CONTROLLED_LINKS = 39
            TL_CURRENT_PHASE = 40
            TL_CURRENT_PROGRAM = 41
            TL_EXTERNAL_STATE = 46
            TL_NEXT_SWITCH = 45
            TL_PHASE_DURATION = 36
            TL_PHASE_INDEX = 34
            TL_PRIORITY_VEHICLES = 49
            TL_PROGRAM = 35
            TL_RED_YELLOW_GREEN_STATE = 32
            TL_RIVAL_VEHICLES = 48

        Args:
            tlsID (str): Signal ID.
        """
        traci.trafficlight.subscribe(
            tlsID,
            [tc.TL_CURRENT_PHASE, tc.TL_PHASE_DURATION, tc.TL_RED_YELLOW_GREEN_STATE],
        )

    def get_signal_information(self, tlsID):
        """Get the subscribed information of traffic signal in the last time step.

        Args:
            tlsID (str): Signal ID.

        Returns:
            dict: Subscribed information of the specified traffic light.
        """
        return traci.trafficlight.getSubscriptionResults(tlsID)

    def get_tlsID_list(self):
        """Get a list of traffic light ID.

        Returns:
            list(str): List of all traffic light in the network.
        """
        return traci.trafficlight.getIDList()

    def get_signal_state(self, tlsID):
        """Returns the named tl's state as a tuple of light definitions from rugGyYoO, for red, yed-yellow, green, yellow, off, where lower case letters mean that the stream has to decelerate.

        Args:
            tlsID (str): Signal ID.

        Returns:
            str: Current state of the specified traffic light.
        """
        return traci.trafficlight.getRedYellowGreenState(tlsID)

    def set_signal_logic(self, tlsID, newlogic):
        """Sets a new program for the given tlsID from a Logic object.

        Args:
            tlsID (str): Signal ID.
            newlogic (Logic): New traffic light logic.
        """
        traci.trafficlight.setProgramLogic(tlsID, newlogic)

    def get_vruID_list(self):
        """Get a list of VRU ID.

        Returns:
            list(str): List of all VRU in the network.
        """
        return traci.person.getIDList()
