'''This module defines the interface for agents
'''

import addict
from attrs import define
from typing import NewType, Union, List, Optional, Dict, Any

AgentId = NewType('AgentId', str)

class AgentType:
    def __init__(self) -> None:
        pass

    @staticmethod
    def default() -> 'AgentType':
        return AgentType()

@define
class AgentDepartureInfo:
    time: float = None # Time step at which the vehicle should enter the network. Defaults to None.
    lane: str = 'first' # Lane on which the vehicle should be inserted. Defaults to 'first'.
    lane_id: str = None # specific lane id where vehicle should be inserted
    position: str = 'base' # Position at which the vehicle should enter the net. Defaults to 'base'.
    speed: str = '0' # Speed with which the vehicle should enter the network. Defaults to '0'.

@define
class AgentArrivalInfo:
    lane: str = 'current' # Lane at which the vehicle should leave the network. Defaults to 'current'.
    position: str = 'max' # Position at which the vehicle should leave the network. Defaults to 'max'.
    speed: str = 'current' # Speed with which the vehicle should leave the network. Defaults to 'current'.

@define
class AgentInitialInfo:
    route: str
    type: AgentType = AgentType.default()
    depart: AgentDepartureInfo = AgentDepartureInfo()
    arrive: AgentArrivalInfo = AgentArrivalInfo()

    from_taz: str = '' # Traffic assignment zone where the vehicle should enter the network. Defaults to ''.
    to_taz: str = '' # Traffic assignment zones where the vehicle should leave the network. Defaults to ''.
    line: str = '' # A string specifying the id of a public transport line which can be used when specifying person rides. Defaults to ''.
    person_capacity: int = 0 # Number of all seats of the added vehicle. Defaults to 0.
    person_number: int = 0 # Number of occupied seats when the vehicle is inserted. Defaults to 0.

class Agent:
    '''
    A basic class holds the essential information for agents (vehicles, pedestrians, etc.) in the simulator
    '''

    DEFAULT_PARAMS = {}

    def __init__(self, id: AgentId, simulator: Any, **params) -> None:
        self._id = id
        self._simulator = simulator
        self._params = addict.Dict(self.DEFAULT_PARAMS)
        self._params.update(params)

    @property
    def params(self) -> addict.Dict:
        return self._params
    
    @property
    def id(self):
        return self._id

    @property
    def simulator(self):
        return self._simulator
