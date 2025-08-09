from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity

from .stalled_object import StalledObjectAdversity


class DynamicObjectAdversity(StalledObjectAdversity):

    def set_vehicle_feature(self, vehicle_id: str):
        # traci.vehicle.setSpeedMode(vehicle_id, 39) # [0 1 0 0 1 1 1] safety speed/acc/decce check on[1, 1, 1], road prioroty check off[0], brake to avoid red light running check off[0], disregard traffic rules in the junction[1], speed limit check on[0]
        traci.vehicle.setSpeedMode(vehicle_id, 96)
        # traci.vehicle.setLaneChangeMode(vehicle_id, 0)
        traci.vehicle.setSpeedFactor(vehicle_id, 1.5)
        traci.vehicletype.setParameter(self._object_type, "lcStrategic", "100.0")
        traci.vehicletype.setParameter(self._object_type, "lcCooperative", "0.0")
        traci.vehicletype.setParameter(self._object_type, "lcSpeedGain", "100.0")
        traci.vehicletype.setParameter(self._object_type, "lcKeepRight", "0.0")
        traci.vehicle.setParameter(vehicle_id, "device.bluelight.reactiondist", str(90))
        traci.vehicle.setMaxSpeed(vehicle_id,33)

    def set_vehicle_route(self, vehicle_id):
        route_id = f"r_dynamic_object"
        dynamic_route = self._other_settings.get("route")
        traci.route.add(route_id, dynamic_route)
        return route_id
    
    def add_vehicle(self, vehicle_id):
        route_id = self.set_vehicle_route(vehicle_id)
        traci.vehicle.add(vehicle_id, routeID=route_id, typeID=self._object_type)
        self.set_vehicle_feature(vehicle_id)
        traci.vehicle.moveTo(vehicle_id, self._lane_id, self._lane_position)
        traci.vehicle.setSpeed(vehicle_id, 30)

    def update(self, time: float):
        if self._is_active and self.end_time != -1 and time >= self.end_time:
            try:
                traci.vehicle.remove(self.stalled_object_id)
            except:
                logger.warning(f"Failed to remove the vehicle {self.stalled_object_id}.")
            self._is_active = False


        