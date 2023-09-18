from mtlsp.vehicle.decision_models.base_decision_model import BaseDecisionModel


class DummySetSUMOTranformDecisionModel(BaseDecisionModel):
    """dummy decision model:
    
        This decision model will constantly move the vehicle to the given x, y coordinates

    """
    def __init__(self):
        super().__init__()

    def derive_control_command_from_observation(self, obs_dict):
        """derive control command from observation

        Args:
            obs_dict (dict): vehicle observation dictionary

        Returns:
            dict: command
        """
        command = {
            "type": "SetSumoTransform",
            "position": (100, 46), # x, y
            "velocity": None, # m/s
            "angle": 0.2, # rad
        }
        return command, None