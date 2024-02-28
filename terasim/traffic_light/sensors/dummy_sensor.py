from terasim.agent.agent_sensor import AgentSensor
import traci.constants as tc
from terasim.simulator import traci

class DummySensor(AgentSensor):
    ''' A sensor for reporting basic states (position, speed, heading, etc.) '''

    DEFAULT_PARAMS = dict(
        fields = {
            'state': tc.TL_RED_YELLOW_GREEN_STATE,
        }
    )

    def __init__(self, name = "ego", **params):
        super().__init__(name, **params)

    def subscribe(self) -> None:
        pass
    
    def fetch(self) -> dict:
        pass