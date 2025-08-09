from loguru import logger

from terasim.envs.base import BaseEnv
from terasim.vulnerable_road_user.vulnerable_road_user import VulnerableRoadUserList


class BaseEnvComplete(BaseEnv):
    def __init__(self, vehicle_factory, vulnerable_road_user_factory, info_extractor):
        """Initialize the complete and base testing environment.

        Args:
            vehicle_factory (VehicleFactory): The vehicle factory.
            vulnerable_road_user_factory (VulnerableRoadUserFactory): The vulnerable road user factory.
            info_extractor (InfoExtractor): The info extractor.
        """
        super().__init__(vehicle_factory, info_extractor)
        self.vulnerable_road_user_list = VulnerableRoadUserList({})
        self.vulnerable_road_user_factory = vulnerable_road_user_factory

    def _step(self, simulator, ctx) -> bool:
        """The main step function of the environment.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context dictionary.

        Returns:
            bool: True if the simulation should continue.
        """
        # First synchronize the vehicle list
        self._maintain_all_vehicles(ctx)
        self._maintain_all_vulnerable_road_users(ctx)

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

    ########## Other private utility functions that should not be directly called by custom env

    def _maintain_all_vulnerable_road_users(self, ctx):
        """Maintain the vulnerable road user list based on the departed list and arrived list.
        """
        if "terasim_controlled_vulnerable_road_user_ids" in ctx:
            terasim_controlled_vulnerable_road_user_ids = (
                ctx["terasim_controlled_vulnerable_road_user_ids"]
                if isinstance(ctx["terasim_controlled_vulnerable_road_user_ids"], list)
                else [ctx["terasim_controlled_vulnerable_road_user_ids"]]
            )
            logger.trace("Using the controlled vulnerable road user list from ctx")
            realtime_vruID_set = set(terasim_controlled_vulnerable_road_user_ids) & set(
                self.simulator.get_vruID_list()
            )
        else:
            realtime_vruID_set = set(self.simulator.get_vruID_list())

        vruID_set = set(self.vulnerable_road_user_list.keys())
        # log the difference between the two sets
        logger.trace(f"Realtime vehID set: {realtime_vruID_set}")
        logger.trace(f"Current vehID set: {vruID_set}")
        if vruID_set != realtime_vruID_set:
            for vruID in realtime_vruID_set:
                if vruID not in vruID_set:
                    vru = self._add_vulnerbale_road_user_to_env(vruID)
                    vru._install()
            for vruID in vruID_set:
                if vruID not in realtime_vruID_set:
                    self._remove_vulnerable_road_user_from_env(vruID)

    def _add_vulnerbale_road_user_to_env(self, vru_id_list):
        """Add VRUs from vru_id_list.

        Args:
            vru_id_list (list(str)): List of vru IDs needed to be inserted.

        Raises:
            ValueError: If one vehicle is neither "BV" nor "AV", it should not enter the network.
        """
        single_input = not isinstance(vru_id_list, list)
        if single_input:
            vru_id_list = [vru_id_list]

        output = []
        for vru_id in vru_id_list:
            vru = self.vulnerable_road_user_factory.create_vulnerable_road_user(
                vru_id, self.simulator
            )
            self.vulnerable_road_user_list.add_vulnerable_road_users(vru)
            output.append(vru)
        return output[0] if single_input else output

    def _remove_vulnerable_road_user_from_env(self, vru_id_list):
        """Delete vulnerable road users in vru_id_list.

        Args:
            vru_id_list (list(str)): List of vulnerable road user IDs needed to be deleted.

        Raises:
            ValueError: If the vulnerable road user is neither "BV" nor "AV", it shouldn't enter the network.
        """
        if not isinstance(vru_id_list, list):
            vru_id_list = [vru_id_list]
        for vru_id in vru_id_list:
            if vru_id in self.vulnerable_road_user_list:
                self.vulnerable_road_user_list.pop(vru_id)._uninstall()
