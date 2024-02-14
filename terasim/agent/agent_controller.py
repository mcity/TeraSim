from abc import ABC, abstractmethod


class AgentController(ABC):
    params = {}
    def __init__(self, simulator, params=None):
        self.simulator = simulator
        if params:
            self.params.update(params)
    
    def _install(self, agent):
        pass
    
    def _is_command_legal(self, agent_id, control_command):
        return True

    def execute_control_command(self, agent_id, control_command, obs_dict):
        pass