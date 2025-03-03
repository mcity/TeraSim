"""This module defines the interface for agent controllers
"""

from abc import ABC
import json
from pydantic import BaseModel
from typing import Any, Dict, Optional, Type


class AgentController(ABC):
    params = {}

    def __init__(
        self,
        simulator: Any,
        control_command_schema: Type[BaseModel],
        params: Optional[Dict[str, Any]] = None,
    ):
        """Initialize the agent controller.

        Args:
            simulator (Simulator): The simulator object.
            control_command_schema (BaseModel): The schema of the control command.
            params (Dict, optional): The parameters of the agent controller.
        """
        self._agent = None  # to be assigned from outside
        self.simulator = simulator
        self.control_command_schema = control_command_schema
        if params:
            self.params.update(params)

    def _install(self, agent):
        """Install the agent controller.

        Args:
            agent (Agent): The agent object.
        """
        self._agent = agent
        pass

    def is_command_legal(self, agent_id, control_command):
        """Check if the control command is legal.

        Args:
            agent_id (str): The ID of the agent.
            control_command (dict): The control command.

        Returns:
            bool: True if the control command is legal.
        """
        return True

    def _is_command_legal(self, agent_id, control_command):
        """Inner function to check if the control command is legal.

        Args:
            agent_id (str): The ID of the agent.
            control_command (dict): The control command.

        Returns:
            bool: True if the control command is legal.
        """
        if not isinstance(control_command, self.control_command_schema) and not (
            isinstance(control_command, dict)
            and self.control_command_schema.model_validate_json(json.dumps(control_command))
        ):
            return False
        return self.is_command_legal(agent_id, control_command)

    def execute_control_command(self, agent_id, control_command, obs_dict):
        """Execute the control command.

        Args:
            agent_id (str): The ID of the agent.
            control_command (dict): The control command.
            obs_dict (dict): The observation dictionary.
        """
        pass
