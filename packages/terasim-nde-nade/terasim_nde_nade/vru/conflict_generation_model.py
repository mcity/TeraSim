from ..utils import AdversityManager
from ..vehicle.nde_decision_model import NDEDecisionModel

BaseModel = NDEDecisionModel


class ConflictGenerationModel(BaseModel):
    def __init__(
        self, cfg=None, reroute=True, dynamically_change_vtype=True, *args, **kwargs
    ):
        """Initialize the ConflictGenerationModel module.

        Args:
            cfg (dict): Configuration of the model.
            reroute (bool): Flag to indicate if the vehicle should be rerouted.
            dynamically_change_vtype (bool): Flag to indicate if the vehicle type should be changed dynamically.
        """
        super().__init__(reroute, dynamically_change_vtype, *args, **kwargs)
        self.adversity_manager = AdversityManager(
            cfg.adversity_cfg.vulnerable_road_user if hasattr(cfg.adversity_cfg, "vulnerable_road_user") else None
        )

    def derive_control_command_from_observation(self, obs_dict):
        """Derive the control command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            CommandType: Control command to be executed by the ego agent.
            dict: Dictionary containing the normal and adversarial maneuvers.
        """
        for adversity in self.adversity_manager.adversities:
            if adversity.sumo_net is None:
                adversity.sumo_net = self._agent.simulator.sumo_net
        command, command_dict = self.adversity_manager.derive_command(obs_dict)

        return command, {"ndd_command_distribution": command_dict}
