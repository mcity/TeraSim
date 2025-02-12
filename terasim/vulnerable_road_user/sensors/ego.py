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
        self._length = None
        self._width = None
        self._height = None

    @property
    def length(self):
        if self._length is None:
            self._length = traci.person.getLength(self._agent.id)
        return self._length

    @property
    def width(self):
        if self._width is None:
            self._width = traci.person.getWidth(self._agent.id)
        return self._width

    @property
    def height(self):
        if self._height is None:
            self._height = traci.person.getHeight(self._agent.id)
        return self._height

    def fetch(self) -> dict:
        vru_id = self._agent.id
        data = {"vru_id": vru_id}
        for field, getter in self.params.fields.items():
            data[field] = getter(vru_id)
        data["length"] = self.length
        data["width"] = self.width
        data["height"] = self.height
        return data
