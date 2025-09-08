from pydantic import BaseModel, validator
from typing import Any, Callable, Dict, List, Optional

from terasim.overlay import traci
from terasim.params import AgentType

from .types import CommandType


class NDECommand(BaseModel):
    """
    Represents a command for an agent in a Non-Deterministic Environment (NDE).
    if the command is "default", the agent will follow the SUMO controlled model, other elements will be ignored
    if the command is "left" or "right", the agent will change lane to the left or right, other elements will be ignored
    if the command is "trajectory", the agent will follow the future trajectory, which will be predicted according to the current acceleration, other elements will be ignored
    if the command is "acceleration", the agent will decelerate to stop using the acceleration element
    """

    agent_type: AgentType = AgentType.VEHICLE
    command_type: CommandType = CommandType.DEFAULT
    acceleration: float = 0.0
    future_trajectory: List[List] = [[]]  # shape: (n, 5) for [x, y, heading, velocity, time]
    prob: float = 1.0
    duration: float = None
    time_resolution: float = None
    info: Dict[str, Any] = {}
    custom_control_command: Dict[str, Any] = None
    custom_execute_control_command: Callable = None
    keep_route_mode: int = 1

    @validator("duration", pre=True, always=True)
    def set_duration(cls, v):
        return v if v is not None else traci.simulation.getDeltaT()

    class Config:
        slots = True
        extra = "forbid"
