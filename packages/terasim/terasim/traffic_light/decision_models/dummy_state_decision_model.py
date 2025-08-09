from terasim.agent.agent_decision_model import AgentDecisionModel


class DummyStateDecisionModel(AgentDecisionModel):
    """Dummy decision model:
    This decision model will constantly set the traffic light state as green in all directions
    """

    def derive_control_command_from_observation(self, obs_dict):
        """Derive control command from observation.

        Args:
            obs_dict (dict): The observation dictionary.

        Returns:
            dict: The control command.
        """
        return self.get_decision(), None

    def get_decision(self):
        """Get the decision. Dummy decision is to set the traffic light state as green in all directions.

        Returns:
            str: The decision, which contains the states of all the traffic lights.
        """
        return "ggggggggg"
