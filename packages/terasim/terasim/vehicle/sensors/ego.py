from terasim.agent.agent_sensor import AgentSensor
from terasim.overlay import traci


class EgoSensor(AgentSensor):
    """A sensor for reporting basic states (position, speed, heading, etc.)"""

    DEFAULT_PARAMS = dict(
        fields={
            "velocity": traci.vehicle.getSpeed,
            "position": traci.vehicle.getPosition,
            "position3d": traci.vehicle.getPosition3D,
            "heading": traci.vehicle.getAngle,
            "edge_id": traci.vehicle.getRoadID,
            "lane_id": traci.vehicle.getLaneID,
            "lane_index": traci.vehicle.getLaneIndex,
            "acceleration": traci.vehicle.getAcceleration,
        }
    )

    def __init__(self, name="ego", **params):
        """Initialize the ego state sensor for the vehicle.

        Args:
            name (str, optional): The name of the sensor. Defaults to "ego".
            params (dict, optional): The parameters of the sensor.
        """
        super().__init__(name, **params)
        self._length = None
        self._width = None
        self._height = None

    @property
    def length(self):
        """Get the length of the vehicle.

        Returns:
            float: The length of the vehicle.
        """
        if self._length is None:
            self._length = traci.vehicle.getLength(self._agent.id)
        return self._length

    @property
    def width(self):
        """Get the width of the vehicle.

        Returns:
            float: The width of the vehicle.
        """
        if self._width is None:
            self._width = traci.vehicle.getWidth(self._agent.id)
        return self._width

    @property
    def height(self):
        """Get the height of the vehicle.

        Returns:
            float: The height of the vehicle.
        """
        if self._height is None:
            self._height = traci.vehicle.getHeight(self._agent.id)
        return self._height

    def fetch(self) -> dict:
        """Fetch the vehicle state.

        Returns:
            dict: The vehicle state.
        """
        veh_id = self._agent.id
        data = {"veh_id": veh_id}
        for field, getter in self.params.fields.items():
            data[field] = getter(veh_id)
        data["length"] = self.length
        data["width"] = self.width
        data["height"] = self.height
        return data
