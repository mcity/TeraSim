from terasim.overlay import traci
from terasim.vehicle.decision_models.idm_model import IDMModel

from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    get_location,
)


class NDEDecisionModel(IDMModel):
    def __init__(self, reroute=True, dynamically_change_vtype=True, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reroute = reroute
        self.dynamically_change_vtype = dynamically_change_vtype

    @staticmethod
    def change_vehicle_type_according_to_location(veh_id, vehicle_location):
        """Change the vehicle type according to the location.

        Args:
            veh_id (str): Vehicle ID.
            vehicle_location (str): Location of the vehicle.
        """
        if "AV" in veh_id:
            return
        if "highway" in vehicle_location:  # highway/freeway scenario
            traci.vehicle.setType(veh_id, "NDE_HIGHWAY")
        else:  # urban scenario
            traci.vehicle.setType(veh_id, "NDE_URBAN")

    @staticmethod
    def reroute_vehicle_if_necessary(veh_id: str, current_lane_id: str) -> bool:
        """Reroute the vehicle if necessary.

        Args:
            veh_id (str): Vehicle ID.
            current_lane_id (str): Current lane ID.

        Returns:
            bool: Flag to indicate if the vehicle has been rerouted.
        """
        bestlanes = traci.vehicle.getBestLanes(veh_id)
        # get the bestlane with current lane id
        for lane in bestlanes:
            if lane[0] == current_lane_id:
                if not lane[4]:  # the current lane does not allow continuing the route
                    next_links = traci.lane.getLinks(current_lane_id)
                    if next_links:
                        next_lane_id = next_links[0][0]
                        next_edge_id = traci.lane.getEdgeID(next_lane_id)
                        try:
                            traci.vehicle.changeTarget(veh_id, next_edge_id)
                            return True  # reroute the vehicle
                        except:
                            return False
        return False  # do not reroute the vehicle

    def derive_control_command_from_observation(self, obs_dict):
        """Derive the control command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            CommandType: Control command to be executed by the ego agent.
            dict: Dictionary containing the normal and adversarial maneuvers.
        """
        # change the IDM and MOBIL parameters based on the location
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"], sumo_net=self._agent.simulator.sumo_net, highlight_flag=False, obs_dict=obs_dict
        )
        # for highway and urban scenarios, change the vehicle type
        if self.dynamically_change_vtype:
            self.change_vehicle_type_according_to_location(
                obs_dict["ego"]["veh_id"], vehicle_location
            )
            # if the vehicle needs to be rerouted (e.g., emergency lane change caused by adversarial model), reroute it
        if self.reroute:
            self.reroute_vehicle_if_necessary(
                obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
            )
        # let the vehicle to be controlled by SUMO
        return NDECommand(command_type=CommandType.DEFAULT, prob=1), {
            "ndd_command_distribution": {
                "normal": NDECommand(command_type=CommandType.DEFAULT, prob=1)
            },
            "command_cache": NDECommand(command_type=CommandType.DEFAULT, prob=1),
        }
