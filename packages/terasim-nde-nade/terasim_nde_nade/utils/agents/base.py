from dataclasses import dataclass
from typing import List


@dataclass
class AgentInfo:
    """Base class for agent information."""

    id: str
    position: List[float]
    velocity: float
    heading: float

    def __getitem__(self, item):
        return self.__dict__[item]
