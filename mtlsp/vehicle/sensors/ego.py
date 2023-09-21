from .base import BaseSensor

import traci.constants as tc
from terasim.overlay import traci
from terasim.agent import AgentId

class EgoSensor(BaseSensor):
    ''' A sensor for reporting basic states (position, speed, heading, etc.) '''

    DEFAULT_PARAMS = dict(
        fields = {
            'position': tc.VAR_POSITION,
            'position3d': tc.VAR_POSITION3D,
            'speed': tc.VAR_SPEED,
            'heading': tc.VAR_ANGLE,
            'edge_id': tc.VAR_ROAD_ID,
            'lane_index': tc.VAR_LANE_INDEX,
            'acceleration': tc.VAR_ACCELERATION,
        }
    )

    def __init__(self, name = "ego", **params):
        super().__init__(name, **params)

    def subscribe(self) -> None:
        veh_id = self._agent.id
        var_ids = list(self.params.fields.values())
        traci.vehicle.subscribe(veh_id, varIDs=var_ids)

    def fetch(self) -> dict:
        veh_id = self._agent.id
        sub = traci.vehicle.getSubscriptionResults(veh_id)
        return {name: sub[id] for name, id in self.params.fields.items()}
