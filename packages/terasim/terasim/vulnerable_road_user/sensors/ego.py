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
        """Initialize the ego state sensor for the VRU.

        Args:
            name (str, optional): The name of the sensor. Defaults to "ego".
            params (dict, optional): The parameters of the sensor.
        """
        super().__init__(name, **params)
        self._length = None
        self._width = None
        self._height = None

    @property
    def length(self) -> float:
        """Get the length of the VRU.

        Returns:
            float: The length of the VRU.
        """
        if self._length is None:
            self._length = traci.person.getLength(self._agent.id)
        return self._length

    @property
    def width(self) -> float:
        """Get the width of the VRU.

        Returns:
            float: The width of the VRU.
        """
        if self._width is None:
            self._width = traci.person.getWidth(self._agent.id)
        return self._width

    @property
    def height(self) -> float:
        """Get the height of the VRU.

        Returns:
            float: The height of the VRU.
        """
        if self._height is None:
            self._height = traci.person.getHeight(self._agent.id)
        return self._height

    def fetch(self) -> dict:
        """Fetch the VRU state.

        Returns:
            dict: The VRU state.
        """
        vru_id = self._agent.id
        data = {"vru_id": vru_id}
        for field, getter in self.params.fields.items():
            data[field] = getter(vru_id)
        data["length"] = self.length
        data["width"] = self.width
        data["height"] = self.height
        return data
