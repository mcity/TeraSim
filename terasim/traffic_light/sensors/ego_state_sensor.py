import traci.constants as tc

from terasim.agent.agent_sensor import AgentSensor
from terasim.simulator import traci


class EgoStateSensor(AgentSensor):
    """A sensor for reporting basic states (position, speed, heading, etc.)"""

    DEFAULT_PARAMS = dict(
        fields={
            "state": tc.TL_RED_YELLOW_GREEN_STATE,
        }
    )

    def __init__(self, name="ego", **params):
        super().__init__(name, **params)

    def subscribe(self) -> None:
        tls_id = self._agent.id
        tls_ids = list(self.params.fields.values())
        traci.trafficlight.subscribe(tls_id, varIDs=tls_ids)

    def fetch(self) -> dict:
        tls_id = self._agent.id
        sub = traci.trafficlight.getSubscriptionResults(tls_id)
        return {name: sub[id] for name, id in self.params.fields.items()}
