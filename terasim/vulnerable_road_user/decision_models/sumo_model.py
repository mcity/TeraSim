from terasim.agent.agent_decision_model import AgentDecisionModel
from terasim.overlay import traci


class SUMOModel(AgentDecisionModel):

    def derive_control_command_from_observation(self, obs_dict):
        return None, None
