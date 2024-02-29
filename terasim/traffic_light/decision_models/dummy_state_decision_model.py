from terasim.agent.agent_decision_model import AgentDecisionModel

class DummyStateDecisionModel(AgentDecisionModel):
    """dummy decision model:
        This decision model will constantly set the traffic light state as green in all directions
    """

    def derive_control_command_from_observation(self, obs_dict):
        return self.get_decision(), None
    
    def get_decision(self):
        return "ggggggggg"
