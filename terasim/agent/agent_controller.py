"""This module defines the interface for agent controllers
"""

from abc import ABC
from pydantic import BaseModel
from typing import Optional, Dict, Any, Type
from pydantic import BaseModel


class AgentController(ABC):
    params = {}

    def __init__(
        self,
        simulator: Any,
        control_command_schema: Type[BaseModel],
        params: Optional[Dict[str, Any]] = None,
    ):
        self._agent = None  # to be assigned from outside
        self.simulator = simulator
        self.control_command_schema = control_command_schema
        if params:
            self.params.update(params)

    def _install(self, agent):
        self._agent = agent
        pass

    def is_command_legal(self, agent_id, control_command):
        return True

    def _is_command_legal(self, agent_id, control_command):
        if not isinstance(control_command, self.control_command_schema) and not (
            isinstance(control_command, dict)
            and self.control_command_schema.model_validate_json(control_command)
        ):
            return False
        return self.is_command_legal(agent_id, control_command)

    def execute_control_command(self, agent_id, control_command, obs_dict):
        pass
