from loguru import logger
import math
import random

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


class CollisionAdversity(AbstractStaticAdversity):
    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        if self._lane_id == "":
            logger.warning("Lane ID is not provided.")
            return False
        if self._lane_position == -1:
            logger.warning("Lane position is not provided.")
            return False
        try:
            lane_length = traci.lane.getLength(self._lane_id)
        except:
            logger.warning(f"Failed to get length of the lane {self._lane_id}.")
            return False
        if self._lane_position > lane_length:
            logger.warning(f"Lane position {self._lane_position} is greater than the lane length {lane_length}.")
            return False
        if isinstance(self._object_type, str):
            vehicle_type_list = traci.vehicletype.getIDList()
            if self._object_type not in vehicle_type_list:
                logger.warning(f"Vehicle type {self._object_type} is not available. Using default value 'DEFAULT_VEHTYPE'.")
                self._object_type = ["DEFAULT_VEHTYPE"] * 2
            else:
                self._object_type = [self._object_type] * 2
        elif isinstance(self._object_type, list):
            vehicle_type_list = traci.vehicletype.getIDList()
            for index, object_type in enumerate(self._object_type):
                if object_type not in vehicle_type_list:
                    logger.warning(f"Vehicle type {object_type} is not available. Using default value 'DEFAULT_VEHTYPE'.")
                    self._object_type[index] = "DEFAULT_VEHTYPE"
        else:
            logger.warning("Object type is not provided. Using default value 'DEFAULT_VEHTYPE'.")
            self._object_type = ["DEFAULT_VEHTYPE"] * 2
        if self._other_settings is None:
            logger.warning("Details are not provided.")
            return False
        return True
    
    def initialize(self, time: float):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."
        object1_id = f"BV_1_{self._object_type[0]}_collision"
        object2_id = f"BV_2_{self._object_type[1]}_collision"
        self._static_adversarial_object_id_list.append(object1_id)
        self._static_adversarial_object_id_list.append(object2_id)

        edge_id = traci.lane.getEdgeID(self._lane_id)
        collision_lane_index = int(self._lane_id.split("_")[-1])
        collision_object_route_id = f"r_collision_object"
        traci.route.add(collision_object_route_id, [edge_id])
        traci.vehicle.add(
            object1_id,
            routeID=collision_object_route_id,
            typeID=self._object_type[0],
        )
        traci.vehicle.setSpeedMode(object1_id, 0)
        traci.vehicle.setLaneChangeMode(object1_id, 0)
        object1_position = self._other_settings.get("object_states")[0][0:2]
        object1_angle = self._other_settings.get("object_states")[0][2]
        traci.vehicle.moveToXY(
            object1_id,
            edge_id,
            collision_lane_index,
            object1_position[0],
            object1_position[1],
            angle=object1_angle,
            keepRoute=2,
        )
        traci.vehicle.setSpeed(object1_id, 0)

        traci.vehicle.add(
            object2_id,
            routeID=collision_object_route_id,
            typeID=self._object_type[1],
        )
        traci.vehicle.setSpeedMode(object2_id, 0)
        traci.vehicle.setLaneChangeMode(object2_id, 0)
        object2_position = self._other_settings.get("object_states")[1][0:2]
        object2_angle = self._other_settings.get("object_states")[1][2]
        traci.vehicle.moveToXY(
            object2_id,
            edge_id,
            collision_lane_index,
            object2_position[0],
            object2_position[1],
            angle=object2_angle,
            keepRoute=2,
        )
        traci.vehicle.setSpeed(object2_id, 0)

        self.object1_id = object1_id
        self.object1_edge_id = edge_id
        self.object1_lane_index = collision_lane_index
        self.object1_position = object1_position
        self.object1_angle = object1_angle

        self.object2_id = object2_id
        self.object2_edge_id = edge_id
        self.object2_lane_index = collision_lane_index
        self.object2_position = object2_position
        self.object2_angle = object2_angle
        self._is_active = True

    def update(self, time: float):
        if self._is_active and self.end_time != -1 and time >= self.end_time:
            try:
                traci.vehicle.remove(self.object1_id)
                traci.vehicle.remove(self.object2_id)
            except:
                logger.warning(f"Failed to remove the vehicle {self.object1_id} or {self.object2_id}.")
            self._is_active = False
        if self._is_active: # maintain the position of the vehicles
            traci.vehicle.moveToXY(
                self.object1_id, 
                self.object1_edge_id, 
                self.object1_lane_index,
                self.object1_position[0],
                self.object1_position[1],
                angle=self.object1_angle,
                keepRoute=2
            )
            traci.vehicle.setSpeed(self.object1_id, 0)
            traci.vehicle.moveToXY(
                self.object2_id, 
                self.object2_edge_id, 
                self.object2_lane_index,
                self.object2_position[0],
                self.object2_position[1],
                angle=self.object2_angle,
                keepRoute=2
            )
            traci.vehicle.setSpeed(self.object2_id, 0)
        return