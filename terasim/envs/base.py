from terasim.simulator import Simulator
from terasim.vehicle.vehicle import VehicleList
from terasim.agent.agent import AgentInitialInfo, AgentDepartureInfo
from abc import ABC, abstractmethod
import terasim.utils as utils
from typing import Union
from terasim.overlay import traci
from loguru import logger


class BaseEnv(ABC):
    def __init__(self, vehicle_factory, info_extractor):
        self.episode_info = {"start_time": None, "end_time": None}
        self.vehicle_list = VehicleList({})
        self.vehicle_factory = vehicle_factory
        self.info_extractor = info_extractor(self)
        self.simulator: Simulator = None  # to be assigned by the simulator

    ########## Abstract methods that must be overwritten by custom envs ##########

    @abstractmethod
    def on_start(self, ctx) -> bool:
        """Return False if the start stage failed."""
        pass

    @abstractmethod
    def on_step(self, ctx) -> Union[bool, dict]:
        """
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
        """Return False if the stop stage failed"""
        pass

    ########## Utility methods that can be called by custom env ##########

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
    ):
        assert (
            type(route) == list or route is None
        ), "route should be a list or None, route_id is now used for route id assignment"
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
        if self.simulator is None:
            raise Exception("Simulator is not connected yet.")
        if not hasattr(self.simulator, "sumo_net"):
            raise Exception("sumo_net is not initialized yet.")
        return self.simulator.sumo_net

    def remove_vehicle(self, veh_id):
        vehicle = self.vehicle_list[veh_id]
        vehicle._uninstall()
        self.simulator._remove_vehicle_from_sim(vehicle)
        self._remove_vehicle_from_env(veh_id)

    ########## These methods are those hooked into the pipelines ##########

    # TODO: remove the simulator arguments in these hooks
    def _start(self, simulator, ctx) -> bool:
        # Log down initialization information
        self.episode_info = {"start_time": utils.get_time(), "end_time": None}

        return self.on_start(ctx)

    def _step(self, simulator, ctx) -> bool:
        # First synchronize the vehicle list
        self._maintain_all_vehicles(ctx)

        # Then call custom env defined step
        step_result = self.on_step(ctx)

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
        self.on_stop(ctx)

    ########## Other private utility functions that should not be directly called by custom env

    def _maintain_all_vehicles(self, ctx):
        """Maintain the vehicle list based on the departed vehicle list and arrived vehicle list."""

        if "terasim_controlled_vehicle_ids" in ctx:
            logger.trace("Using the controlled vehicle list from ctx")
            realtime_vehID_set = set(ctx["terasim_controlled_vehicle_ids"]) & set(
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
        self.episode_info["end_time"] = utils.get_time() - utils.get_step_size()
        self.info_extractor.get_terminate_info(True, reason, info)
        self.simulator.running = False
