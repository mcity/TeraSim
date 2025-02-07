from terasim.agent.agent_sensor import AgentSensor
from terasim.overlay import traci


class EgoSensor(AgentSensor):
    """A sensor for reporting basic states (position, speed, heading, etc.)"""

    DEFAULT_PARAMS = dict(
        fields={
            "velocity": traci.person.getSpeed,
            "position": traci.person.getPosition,
            "position3d": traci.person.getPosition3D,
            "heading": traci.person.getAngle,
            "edge_id": traci.person.getRoadID,
            "lane_id": traci.person.getLaneID,
        }
    )

    def __init__(self, name="ego", **params):
        super().__init__(name, **params)

    @property
    def length(self):
        return traci.person.getLength(self._agent.id)

    @property
    def width(self):
        return traci.person.getWidth(self._agent.id)

    @property
    def height(self):
        return traci.person.getHeight(self._agent.id)

    def fetch(self) -> dict:
        vru_id = self._agent.id
        data = {"vru_id": vru_id}
        for field, getter in self.params.fields.items():
            data[field] = getter(vru_id)
        return data
