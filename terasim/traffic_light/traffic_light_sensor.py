from terasim.agent.agent_sensor import AgentSensor
import traci.constants as tc
import traci

class BaseTrafficLightSensor(AgentSensor):
    DEFAULT_PARAMS = {
        "fields": {
            "traffic_light_state": tc.TL_RED_YELLOW_GREEN_STATE,
            "traffic_light_state_time": tc.TL_NEXT_SWITCH,
        }
    }

    def __init__(self, name = "base_tls_sensor", **params):
        super().__init__(name, **params)

    def subscribe(self) -> None:
        traci.trafficlight.subscribe(self._agent.id, [tc.TL_RED_YELLOW_GREEN_STATE])

    def fetch(self) -> dict:
        raise NotImplementedError

    def update(self) -> None:
        raise NotImplementedError

    def install(self, agent) -> None:
        super().install(agent)
        self.subscribe()

    def uninstall(self) -> None:
        self.unsubscribe()
        super().uninstall()
