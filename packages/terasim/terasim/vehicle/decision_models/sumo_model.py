from terasim.agent.agent_decision_model import AgentDecisionModel
from terasim.overlay import traci


class SUMOModel(AgentDecisionModel):
    def derive_control_command_from_observation(self, obs_dict):
        """Derive control command from observation. SUMO Model outputs nothing, which will give the control to SUMO.

        Args:
            obs_dict (dict): The observation dictionary.
        """
        return None, None
