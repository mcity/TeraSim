from loguru import logger

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity

def create_truck_type():
    custom_type_id = "TRUCK"
    if custom_type_id not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", custom_type_id)
        traci.vehicletype.setVehicleClass(custom_type_id, "truck")
        traci.vehicletype.setShapeClass(custom_type_id, "truck")
        traci.vehicletype.setLength(custom_type_id, 10)
        traci.vehicletype.setWidth(custom_type_id, 2.5)
        traci.vehicletype.setHeight(custom_type_id, 4)
        traci.vehicletype.setMaxSpeed(custom_type_id, 10)
        traci.vehicletype.setSpeedFactor(custom_type_id, 1)
        traci.vehicletype.setColor(custom_type_id, (255, 0, 0, 255))
    return custom_type_id

def create_emergency_police_type(subclass="EMERGENCY"):
    """Create a custom vehicle type for emergency vehicles.

    Args:
        subclass (str): The subclass of the emergency vehicle.
        Available subclasses: "ambulance", "firebrigade", "POLICE"

    Returns:
        str: The ID of the custom vehicle type.
    """
    custom_type_id = f"EMERGENCY_{subclass}"
    guiShape = subclass.lower() # emergency stands for ambulance, firebrigade, and police stands for the name

    if custom_type_id not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", custom_type_id)
        traci.vehicletype.setVehicleClass(custom_type_id, "emergency")
        traci.vehicletype.setShapeClass(custom_type_id, guiShape)
        # traci.vehicletype.setColor(custom_type_id, (0, 0, 255, 255))
        traci.vehicletype.setSpeedFactor(custom_type_id, 1.2) 
        traci.vehicletype.setParameter(custom_type_id, "has.bluelight.device", "true")
        traci.vehicletype.setParameter(custom_type_id, "lcStrategic", "100.0")
        traci.vehicletype.setParameter(custom_type_id, "lcCooperative", "0.0")
        traci.vehicletype.setParameter(custom_type_id, "lcSpeedGain", "100.0")
        traci.vehicletype.setParameter(custom_type_id, "lcKeepRight", "0.0")
    return custom_type_id


def create_pedestrian_type():
    """Create a custom vehicle type representing a pedestrian using a small static car.

    Returns:
        str: The ID of the custom vehicle type for pedestrian.
    """
    custom_type_id = "PEDESTRIAN"
    
    if custom_type_id not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", custom_type_id)
        traci.vehicletype.setVehicleClass(custom_type_id, "passenger")
        traci.vehicletype.setShapeClass(custom_type_id, "passenger")
        traci.vehicletype.setLength(custom_type_id, 0.5)  # Very small length
        traci.vehicletype.setWidth(custom_type_id, 0.5)   # Very small width
        traci.vehicletype.setHeight(custom_type_id, 1.7)  # Human height
        # traci.vehicletype.setMaxSpeed(custom_type_id, 0)  # Static, no movement
        # traci.vehicletype.setSpeedFactor(custom_type_id, 0)
        traci.vehicletype.setColor(custom_type_id, (255, 0, 0, 255))  # Red color for visibility
    return custom_type_id


class StalledObjectAdversity(AbstractStaticAdversity):

    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        
        if self._placement_mode == "lane_position":
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
        elif self._placement_mode == "xy_angle":
            if self._x is None or self._y is None:
                logger.warning("X and Y coordinates are not provided for xy_angle placement mode.")
                return False
            if self._angle is None:
                logger.warning("Angle is not provided for xy_angle placement mode.")
                return False
        else:
            logger.warning(f"Invalid placement mode: {self._placement_mode}. Must be 'lane_position' or 'xy_angle'.")
            return False
            
        if self._object_type == "":
            logger.warning("Object type is not provided. Using default value 'DEFAULT_VEHTYPE'.")
            self._object_type = "DEFAULT_VEHTYPE"
        elif self._object_type in ["EMERGENCY", "FIREBRIGADE", "POLICE"]:
            self._object_type = create_emergency_police_type(self._object_type)
        elif self._object_type == "PEDESTRIAN":
            self._object_type = create_pedestrian_type()
        elif self._object_type == "TRUCK":
            self._object_type = create_truck_type()
        else:
            vehicle_type_list = traci.vehicletype.getIDList()
            if self._object_type not in vehicle_type_list:
                logger.warning(f"Vehicle type {self._object_type} is not available. Using default value 'DEFAULT_VEHTYPE'.")
                self._object_type = "DEFAULT_VEHTYPE"
        return True
    
    def set_vehicle_feature(self, vehicle_id: str):
        traci.vehicle.setSpeedMode(vehicle_id, 0)
        traci.vehicle.setLaneChangeMode(vehicle_id, 0)

    def add_vehicle(self, vehicle_id: str):
        if self._placement_mode == "lane_position":
            stalled_object_route_id = self.set_vehicle_route(vehicle_id)
            traci.vehicle.add(
                vehicle_id,
                routeID=stalled_object_route_id,
                typeID=self._object_type,
                vclass=self._vclass
            )
            self.set_vehicle_feature(vehicle_id)
            traci.vehicle.moveTo(vehicle_id, self._lane_id, self._lane_position)
            traci.vehicle.setSpeed(vehicle_id, 0)
        elif self._placement_mode == "xy_angle":
            edge_id = self._get_edge_from_xy()
            stalled_object_route_id = self.set_vehicle_route_for_xy(vehicle_id, edge_id)
            traci.vehicle.add(
                vehicle_id,
                routeID=stalled_object_route_id,
                typeID=self._object_type,
            )
            self.set_vehicle_feature(vehicle_id)
            traci.vehicle.moveToXY(vehicle_id, "", -1, self._x, self._y, self._angle, keepRoute=2)
            traci.vehicle.setSpeed(vehicle_id, 0)

    def set_vehicle_route(self, vehicle_id: str):
        edge_id = traci.lane.getEdgeID(self._lane_id)
        stalled_object_route_id = f"r_stalled_object"
        if stalled_object_route_id not in traci.route.getIDList():
            traci.route.add(stalled_object_route_id, [edge_id])
        return stalled_object_route_id
    
    def set_vehicle_route_for_xy(self, vehicle_id: str, edge_id: str):
        stalled_object_route_id = f"r_stalled_object_xy"
        if stalled_object_route_id not in traci.route.getIDList():
            traci.route.add(stalled_object_route_id, [edge_id])
        return stalled_object_route_id
    
    def _get_edge_from_xy(self):
        try:
            edge_id = traci.simulation.convertRoad(self._x, self._y, isGeo=False)[0]
            return edge_id
        except:
            logger.warning(f"Failed to get edge from coordinates ({self._x}, {self._y}). Using default edge.")
            return "1"
    
    def initialize(self, time: float):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."
        if self._object_type == "PEDESTRIAN":
            stalled_object_id = f"VRU_{self._object_type}_stalled_object"
        else:
            stalled_object_id = f"BV_{self._object_type}_stalled_object"
        self._static_adversarial_object_id_list.append(stalled_object_id)
        
        if self._placement_mode == "lane_position":
            edge_id = traci.lane.getEdgeID(self._lane_id)
            self.edge_id = edge_id
            self.lane_index = self._lane_id.split("_")[-1]
            self.lane_position = self._lane_position
        elif self._placement_mode == "xy_angle":
            edge_id = self._get_edge_from_xy()
            self.edge_id = edge_id
            self.lane_index = 0
            self.lane_position = None
        
        self.add_vehicle(stalled_object_id)

        self._duration=0
        self._is_active = True
        self.stalled_object_id = stalled_object_id

    def update(self, time: float):
        if self._is_active and self.end_time != -1 and time >= self.end_time:
            try:
                traci.vehicle.remove(self.stalled_object_id)
            except:
                logger.warning(f"Failed to remove the vehicle {self.stalled_object_id}.")
            self._is_active = False
        if self._is_active:
            if self._placement_mode == "lane_position":
                traci.vehicle.moveTo(self.stalled_object_id, self._lane_id, self._lane_position)
            elif self._placement_mode == "xy_angle":
                edge_id = self._get_edge_from_xy()
                traci.vehicle.moveToXY(self.stalled_object_id, "", -1, self._x, self._y, self._angle, keepRoute=2)
            traci.vehicle.setSpeed(self.stalled_object_id, 0)

    