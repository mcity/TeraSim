from terasim.agent.agent_decision_model import AgentDecisionModel


class DummySetSUMOTranformDecisionModel(AgentDecisionModel):
    """dummy decision model:
    This decision model will constantly move the vehicle to the given x, y coordinates
    """

    def derive_control_command_from_observation(self, obs_dict):
        command = {
            "type": "SetSumoTransform",
            "position": (100, 46),  # x, y
            "velocity": None,  # m/s
            "angle": 0.2,  # rad
        }
        return command, None
