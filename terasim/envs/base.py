from abc import ABC, abstractmethod
from typing import Union

from loguru import logger

from terasim.agent.agent import AgentDepartureInfo, AgentInitialInfo
from terasim.overlay import traci
from terasim.simulator import Simulator
import terasim.utils as utils
from terasim.vehicle.vehicle import VehicleList


class BaseEnv(ABC):
    def __init__(self, vehicle_factory, info_extractor):
        """Initialize the base testing environment.

        Args:
            vehicle_factory (VehicleFactory): The vehicle factory.
            info_extractor (InfoExtractor): The info extractor.
        """
        self.episode_info = {"start_time": None, "end_time": None}
        self.vehicle_list = VehicleList({})
        self.vehicle_factory = vehicle_factory
        self.info_extractor = info_extractor(self)
        self.simulator: Simulator = None  # to be assigned by the simulator

    ########## Abstract methods that must be overwritten by custom envs ##########

    @abstractmethod
    def on_start(self, ctx) -> bool:
        """Functions to be called when the simulation starts.
        """
        pass

    @abstractmethod
    def on_step(self, ctx) -> Union[bool, dict]:
        """Functions to be called in each simulation step.
        If returned value is boolean, then:
            - True means the simulation shall continue
            - False means the simulation is finished normally.
        If returned value is a dictionary, then:
            - The simulation should be stopped and the related information
              is stored in the dictionary
        """
        pass

    @abstractmethod
    def on_stop(self, ctx) -> bool:
        """Functions to be called when the simulation stops.
        """
        pass

    ########## Utility methods that can be called by custom env ##########

    def add_vehicle_type(
        self,
        type_id: str,
        length: float = 5.0,
        width: float = 2.0,
        height: float = 1.5,
        veh_class: str = "passenger",
        **kwargs,
    ):
        """Add a new vehicle type to the simulation.

        Args:
            type_id (str): The ID of the vehicle type.
            length (float, optional): The length of the vehicle. Defaults to 5.0.
            width (float, optional): The width of the vehicle. Defaults to 2.0.
            height (float, optional): The height of the vehicle. Defaults to 1.5.
            veh_class (str, optional): The class of the vehicle. Defaults to "passenger".
        """
        if type_id in traci.vehicletype.getIDList():
            logger.warning(f"Cannot add new vehicle type: Vehicle type {type_id} already exists.")
        else:
            traci.vehicletype.copy("DEFAULT_VEHTYPE", type_id)
            traci.vehicletype.setLength(type_id, length)
            traci.vehicletype.setWidth(type_id, width)
            traci.vehicletype.setHeight(type_id, height)
            traci.vehicletype.setVehicleClass(type_id, veh_class)

    def add_vehicle(
        self,
        veh_id,
        route_id,
        route=None,
        lane=None,
        lane_id=None,
        position=0,
        speed=-1,
        type_id="DEFAULT_VEHTYPE",
        **kwargs,
    ):
        """Add a new vehicle to the simulation.

        Args:
            veh_id (str): The ID of the vehicle.
            route_id (str): The ID of the route.
            route (list, optional): The route of the vehicle. Defaults to None.
            lane (int, optional): The lane of the vehicle. Defaults to None.
            lane_id (str, optional): The ID of the lane. Defaults to None.
            position (int, optional): The position of the vehicle. Defaults to 0.
            speed (int, optional): The speed of the vehicle. Defaults to -1.
            type_id (str, optional): The ID of the vehicle type. Defaults to "DEFAULT_VEHTYPE".
        """
        assert (
            type(route) == list or route is None
        ), "route should be a list or None, route_id is now used for route id assignment"
        if type_id not in traci.vehicletype.getIDList():
            self.add_vehicle_type(type_id, **kwargs)
        # create the vehicle object from the vehicle factory and add the vehicle to the vehicle list
        vehicle = self._add_vehicle_to_env(veh_id)

        if route_id not in traci.route.getIDList() and route is not None:
            traci.route.add(route_id, route)
        # add the vehicle to the simulators
        self.simulator._add_vehicle_to_sim(
            vehicle,
            AgentInitialInfo(
                route=route_id,
                type=type_id,
                depart=AgentDepartureInfo(
                    position=position, speed=speed, lane=lane, lane_id=lane_id
                ),
            ),
        )

        # setup the attached sensors, controllers, etc
        vehicle._install()

    @property
    def sumo_net(self):
        """Return the sumo network object.

        Raises:
            Exception: If the simulator is not connected yet.
            Exception: If the sumo_net is not initialized yet.

        Returns:
            Net: The sumo network object.
        """
        if self.simulator is None:
            raise Exception("Simulator is not connected yet.")
        if not hasattr(self.simulator, "sumo_net"):
            raise Exception("sumo_net is not initialized yet.")
        return self.simulator.sumo_net

    def remove_vehicle(self, veh_id):
        """Remove a vehicle from the simulation.

        Args:
            veh_id (str): The ID of the vehicle.
        """
        vehicle = self.vehicle_list[veh_id]
        vehicle._uninstall()
        self.simulator._remove_vehicle_from_sim(vehicle)
        self._remove_vehicle_from_env(veh_id)

    ########## These methods are those hooked into the pipelines ##########

    # TODO: remove the simulator arguments in these hooks
    def _start(self, simulator, ctx) -> bool:
        """Start the simulation.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context.    

        Returns:
            bool: True if the simulation starts successfully.
        """
        # Log down initialization information
        self.episode_info = {"start_time": utils.get_time(), "end_time": None}
        
        # Call info extractor initialization
        self.info_extractor.add_initialization_info()

        return self.on_start(ctx)

    def _step(self, simulator, ctx) -> bool:
        """Run a simulation step.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.

        Returns:
            bool: True if the simulation should continue.
        """
        # First synchronize the vehicle list
        self._maintain_all_vehicles(ctx)

        # Then call custom env defined step
        step_result = self.on_step(ctx)
        
        # Call info extractor snapshot
        self.info_extractor.get_snapshot_info({"step_result": step_result})

        # If custom env requested to stop, log some of the information
        if isinstance(step_result, bool):
            if step_result:
                return True
            else:
                self._request_termination("Simulation ends normally", None)
                return False
        elif isinstance(step_result, dict):
            self._request_termination(step_result["reason"], step_result["info"])
            return False
        else:
            raise TypeError("The output of a step should be a boolean or a dictionary")

    def _stop(self, simulator, ctx):
        """Stop the simulation.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        self.on_stop(ctx)

    ########## Other private utility functions that should not be directly called by custom env

    def _maintain_all_vehicles(self, ctx):
        """Maintain the vehicle list based on the departed vehicle list and arrived vehicle list.
        """

        if "terasim_controlled_vehicle_ids" in ctx:
            terasim_controlled_vehicle_ids = (
                ctx["terasim_controlled_vehicle_ids"]
                if isinstance(ctx["terasim_controlled_vehicle_ids"], list)
                else [ctx["terasim_controlled_vehicle_ids"]]
            )
            logger.trace("Using the controlled vehicle list from ctx")
            realtime_vehID_set = set(terasim_controlled_vehicle_ids) & set(
                self.simulator.get_vehID_list()
            )
        else:
            realtime_vehID_set = set(self.simulator.get_vehID_list())

        vehID_set = set(self.vehicle_list.keys())
        # log the difference between the two sets
        logger.trace(f"Realtime vehID set: {realtime_vehID_set}")
        logger.trace(f"Current vehID set: {vehID_set}")
        if vehID_set != realtime_vehID_set:
            for vehID in realtime_vehID_set:
                if vehID not in vehID_set:
                    vehicle = self._add_vehicle_to_env(vehID)
                    vehicle._install()
            for vehID in vehID_set:
                if vehID not in realtime_vehID_set:
                    self._remove_vehicle_from_env(vehID)

    def _add_vehicle_to_env(self, veh_id_list):
        """Add vehicles from veh_id_list.

        Args:
            veh_id_list (list(str)): List of vehicle IDs needed to be inserted.

        Raises:
            ValueError: If one vehicle is neither "BV" nor "AV", it should not enter the network.
        """
        single_input = not isinstance(veh_id_list, list)
        if single_input:
            veh_id_list = [veh_id_list]

        output = []
        for veh_id in veh_id_list:
            veh = self.vehicle_factory.create_vehicle(veh_id, self.simulator)
            self.vehicle_list.add_vehicles(veh)
            output.append(veh)
        return output[0] if single_input else output

    def _remove_vehicle_from_env(self, veh_id_list):
        """Delete vehicles in veh_id_list.

        Args:
            veh_id_list (list(str)): List of vehicle IDs needed to be deleted.

        Raises:
            ValueError: If the vehicle is neither "BV" nor "AV", it shouldn't enter the network.
        """
        if not isinstance(veh_id_list, list):
            veh_id_list = [veh_id_list]
        for veh_id in veh_id_list:
            if veh_id in self.vehicle_list:
                self.vehicle_list.pop(veh_id)._uninstall()

    def _request_termination(self, reason, info):
        """Request the termination of the simulation.

        Args:
            reason (str): The reason of the termination.
            info (dict): The information of the termination.
        """
        self.episode_info["end_time"] = utils.get_time() - utils.get_step_size()
        self.info_extractor.get_terminate_info(True, reason, info)
        self.simulator.running = False
