from terasim.overlay import traci
from terasim_nde_nade.vehicle.nde_decision_model import NDEDecisionModel

from terasim_nde_nade.utils import (
    CommandType,
    NDECommand,
    get_location,
)


class EmergencyVehicleModel(NDEDecisionModel):
    def __init__(self, reroute=True, dynamically_change_vtype=True, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reroute = reroute
        self.dynamically_change_vtype = dynamically_change_vtype

    def derive_control_command_from_observation(self, obs_dict):
        """Derive the control command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            CommandType: Control command to be executed by the ego agent.
            dict: Dictionary containing the normal and adversarial maneuvers.
        """
        vehicle_id = obs_dict["ego"]["veh_id"]
        traci.vehicle.setSpeedMode(vehicle_id, 96)
        traci.vehicle.setLaneChangeMode(vehicle_id, 0)
        traci.vehicle.setSpeedFactor(vehicle_id, 1.5)
        traci.vehicle.setParameter(vehicle_id, "device.bluelight.reactiondist", str(90))
        traci.vehicle.setMaxSpeed(vehicle_id,33)


        # let the vehicle to be controlled by SUMO
        return NDECommand(command_type=CommandType.DEFAULT, prob=1), {
            "ndd_command_distribution": {
                "normal": NDECommand(command_type=CommandType.DEFAULT, prob=1)
            },
            "command_cache": NDECommand(command_type=CommandType.DEFAULT, prob=1),
        }
