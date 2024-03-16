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
        super().__init__(name, **params)

    def fetch(self) -> dict:
        veh_id = self._agent.id
        data = {"veh_id": veh_id}
        for field, getter in self.params.fields.items():
            data[field] = getter(veh_id)
        return data
