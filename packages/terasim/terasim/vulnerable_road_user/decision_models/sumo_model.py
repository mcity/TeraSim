from terasim.agent.agent_decision_model import AgentDecisionModel


class SUMOModel(AgentDecisionModel):
    def derive_control_command_from_observation(self, obs_dict):
        """Derive control command from observation.

        Args:
            obs_dict (dict): The observation dictionary.
        """            
        return None, None
