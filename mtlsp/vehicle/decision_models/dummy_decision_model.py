from mtlsp.vehicle.decision_models.base_decision_model import BaseDecisionModel

class DummyDecisionModel(BaseDecisionModel):
    """dummy decision model:
    
        This decision model will output constant acceleration (0.1m/s^2) and lane change commands, used to test the simulator

    """
    longitudinal_control_command = {
            "longitudinal": 0.1,
            "lateral": "central",
            "type": "lon_lat"
        }
    constant_control_command = {
            "longitudinal": 0,
            "lateral": "central",
            "type": "lon_lat"
        }
    left_control_command = {
        "longitudinal": 0,
        "lateral": "left",
        "type": "lon_lat"
    }
    right_control_command = {
        "longitudinal": 0,
        "lateral": "right",
        "type": "lon_lat"
    }

    def __init__(self, mode="constant"):
        self.mode = mode
        super().__init__()

    def derive_control_command_from_observation(self, obs_dict):
        """derive control command from observation

        Args:
            obs_dict (dict): vehicle observation dictionary

        Returns:
            dict: command
        """
        if self.mode == "constant":
            return self.constant_control_command, None
        else:
            import random
            rand_int = random.randint(0, 2)
            command_list = [self.longitudinal_control_command, self.left_control_command, self.right_control_command]
            return command_list[rand_int], None