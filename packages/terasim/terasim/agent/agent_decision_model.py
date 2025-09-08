"""This module defines the interface for agent decision models
"""

from abc import ABC


class AgentDecisionModel(ABC):
    """DecisionModel class deal with the control of the vehicle based on observation"""

    def __init__(self):
        """Initialize the agent decision model.
        """
        self._agent = None  # to be assigned from outside
        self.control_log = {}  # This will have the control log result for each controller

    def _reset(self):
        """Reset the agent decision model.
        """
        pass

    def _install(self, agent):
        """Install the agent decision model.

        Args:
            agent (Agent): The agent object.
        """
        self._agent = agent
        pass

    def derive_control_command_from_observation(self, obs_dict):
        """Derive control command from observation.

        Args:
            obs_dict (dict): The observation dictionary.
        
        Returns:
            dict: The control command.
        """
        raise NotImplementedError("Decision model decision function not implemented!")
