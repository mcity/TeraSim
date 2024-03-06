'''This module defines the interface for agent controllers
'''
from abc import ABC

class AgentController(ABC):
    params = {}
    def __init__(self, simulator, params=None):
        self._agent = None # to be assigned from outside
        self.simulator = simulator
        if params:
            self.params.update(params)
    
    def _install(self, agent):
        self._agent = agent
        pass
    
    def _is_command_legal(self, agent_id, control_command):
        return True

    def execute_control_command(self, agent_id, control_command, obs_dict):
        pass