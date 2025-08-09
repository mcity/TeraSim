from pydantic import BaseModel
from typing import Dict

from .AgentStateSimplified import AgentStateSimplified
from .Header import Header
from .SUMOSignal import SUMOSignal


class SimulationState(BaseModel):
    header: Header = Header()

    # Simulation time
    simulation_time: float = 0.0

    # Number of agents, current we support only vehicles and vrus
    agent_count: Dict[str, int] = {}

    # Details of the agents, the outer key is the agent type, the inner key is the agent ID, and the value is the agent state
    agent_details: Dict[str, Dict[str, AgentStateSimplified]] = {}

    # Details of the traffic lights, the key is the traffic light ID, and the value is the traffic light state
    traffic_light_details: Dict[str, SUMOSignal] = {}

    # Details of the construction zones, the key is the construction zone lane ID, and the value is its shape
    construction_zone_details: Dict[str, list[list[float]]] = {}
    
    # Details of construction objects (cones, barriers, signs), the key is the object ID, and the value is the object state
    construction_objects: Dict[str, AgentStateSimplified] = {}
