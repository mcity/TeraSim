"""This module defines the interface for agent decision models
"""

from abc import ABC


class AgentDecisionModel(ABC):
    """DecisionModel class deal with the control of the vehicle based on observation"""

    def __init__(self):
        self._agent = None  # to be assigned from outside
        self.control_log = {}  # This will have the control log result for each controller

    def _reset(self):
        pass

    def _install(self, agent):
        self._agent = agent
        pass

    def derive_control_command_from_observation(self, obs_dict):
        raise NotImplementedError("Decision model decision function not implemented!")
