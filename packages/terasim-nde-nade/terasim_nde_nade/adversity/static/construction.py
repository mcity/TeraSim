from loguru import logger
import math

from terasim.overlay import traci

from ...utils import AbstractStaticAdversity


def create_construction_cone_type():
    """Create a custom vehicle type for construction cones.
    
    Returns:
        str: The ID of the custom vehicle type.
    """
    custom_type_id = "CONSTRUCTION_CONE"
    
    if custom_type_id not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", custom_type_id)
        traci.vehicletype.setVehicleClass(custom_type_id, "passenger")
        traci.vehicletype.setShapeClass(custom_type_id, "passenger")
        traci.vehicletype.setLength(custom_type_id, 0.3)  # Smaller cone
        traci.vehicletype.setWidth(custom_type_id, 0.3)   # Smaller cone
        traci.vehicletype.setHeight(custom_type_id, 0.7)  # Cone height
        traci.vehicletype.setMinGap(custom_type_id, 0.1)  # Minimal gap
        traci.vehicletype.setColor(custom_type_id, (255, 140, 0, 255))  # Orange color
    return custom_type_id


def create_construction_barrier_type():
    """Create a custom vehicle type for construction barriers.
    
    Returns:
        str: The ID of the custom vehicle type.
    """
    custom_type_id = "CONSTRUCTION_BARRIER"
    
    if custom_type_id not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", custom_type_id)
        traci.vehicletype.setVehicleClass(custom_type_id, "passenger")
        traci.vehicletype.setShapeClass(custom_type_id, "passenger")
        traci.vehicletype.setLength(custom_type_id, 1.5)  # Smaller barrier
        traci.vehicletype.setWidth(custom_type_id, 0.6)   # Narrower barrier
        traci.vehicletype.setHeight(custom_type_id, 1.0)  # Barrier height
        traci.vehicletype.setMinGap(custom_type_id, 0.1)  # Minimal gap
        traci.vehicletype.setColor(custom_type_id, (255, 255, 0, 255))  # Yellow color
    return custom_type_id


def create_construction_sign_type():
    """Create a custom vehicle type for construction warning signs.
    
    Returns:
        str: The ID of the custom vehicle type.
    """
    custom_type_id = "CONSTRUCTION_SIGN"
    
    if custom_type_id not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", custom_type_id)
        traci.vehicletype.setVehicleClass(custom_type_id, "passenger")
        traci.vehicletype.setShapeClass(custom_type_id, "passenger")
        traci.vehicletype.setLength(custom_type_id, 0.8)  # Sign length
        traci.vehicletype.setWidth(custom_type_id, 0.3)   # Sign width  
        traci.vehicletype.setHeight(custom_type_id, 1.5)  # Sign height
        traci.vehicletype.setMinGap(custom_type_id, 0.1)  # Minimal gap
        traci.vehicletype.setColor(custom_type_id, (255, 0, 0, 255))  # Red color for warning
    return custom_type_id


class ConstructionAdversity(AbstractStaticAdversity):
    def __init__(self, **kwargs):
        # Extract our custom parameters before passing to parent
        self._construction_mode = kwargs.pop("construction_mode", "full_lane")  # "full_lane" or "partial_lane"
        self._start_position = kwargs.pop("start_position", None)
        self._end_position = kwargs.pop("end_position", None)
        self._construction_type = kwargs.pop("construction_type", "cone")  # "cone", "barrier", or "mixed"
        self._spacing = kwargs.pop("spacing", 20.0)  # Spacing between construction objects (MUTCD default ~20m)
        self._lane_offset = kwargs.pop("lane_offset", 0.0)  # Lateral offset from lane center
        
        # Speed-based spacing parameters
        self._speed_limit = kwargs.pop("speed_limit", None)  # Speed limit in mph for dynamic spacing
        self._use_dynamic_spacing = kwargs.pop("use_dynamic_spacing", False)  # Enable MUTCD speed-based spacing
        
        # Taper zone parameters
        self._taper_length_in = kwargs.pop("taper_length_in", 60.0)  # Entry taper length
        self._taper_length_out = kwargs.pop("taper_length_out", 30.0)  # Exit taper length
        self._taper_type = kwargs.pop("taper_type", "linear")  # "linear" or "curved"
        self._work_zone_offset = kwargs.pop("work_zone_offset", None)  # Work zone lateral offset
        
        # Zone configuration
        self._warning_zone_length = kwargs.pop("warning_zone_length", 100.0)  # Warning zone length
        self._warning_zone_spacing = kwargs.pop("warning_zone_spacing", 30.0)  # Warning zone spacing (MUTCD ~30m)
        self._buffer_zone_length = kwargs.pop("buffer_zone_length", 10.0)  # Buffer zone length
        self._termination_zone_length = kwargs.pop("termination_zone_length", 30.0)  # Termination zone
        
        # Warning sign placement
        self._warning_sign_offset = kwargs.pop("warning_sign_offset", -2.5)  # Place signs on shoulder (negative = right)
        
        # Call parent constructor with remaining kwargs
        super().__init__(**kwargs)
        
        # Initialize other attributes
        self._construction_object_ids = []
        
        # If work_zone_offset not specified, use lane_offset
        if self._work_zone_offset is None:
            self._work_zone_offset = self._lane_offset
        
    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        if self._lane_id == "":
            logger.warning("Lane ID is not provided.")
            return False
        try:
            allowed_type_list = traci.lane.getAllowed(self._lane_id)
            lane_length = traci.lane.getLength(self._lane_id)
            self._lane_width = traci.lane.getWidth(self._lane_id)
        except:
            logger.warning(f"Failed to get lane information for {self._lane_id}.")
            return False
            
        # Additional validation for partial lane mode
        if self._construction_mode == "partial_lane":
            if self._start_position is None or self._end_position is None:
                logger.warning("Start and end positions must be provided for partial lane closure.")
                return False
            if self._start_position < 0 or self._end_position > lane_length:
                logger.warning(f"Invalid position range: {self._start_position}-{self._end_position} for lane length {lane_length}.")
                return False
            if self._start_position >= self._end_position:
                logger.warning("Start position must be less than end position.")
                return False
                
        return True
    
    def _calculate_zone_positions(self):
        """Calculate the position ranges for each construction zone."""
        zones = {}
        
        # Calculate actual start position considering warning zone
        actual_start = self._start_position
        
        # Warning zone (before the main construction)
        if self._warning_zone_length > 0:
            zones['warning'] = (
                max(0, actual_start - self._warning_zone_length),
                actual_start
            )
        
        # Entry taper zone
        if self._taper_length_in > 0:
            zones['taper_in'] = (
                actual_start,
                actual_start + self._taper_length_in
            )
        
        # Buffer zone
        buffer_start = actual_start + self._taper_length_in
        if self._buffer_zone_length > 0:
            zones['buffer'] = (
                buffer_start,
                buffer_start + self._buffer_zone_length
            )
        
        # Work zone
        work_start = buffer_start + self._buffer_zone_length
        work_end = self._end_position - self._taper_length_out
        if work_end > work_start:
            zones['work'] = (work_start, work_end)
        
        # Exit taper zone
        if self._taper_length_out > 0:
            zones['taper_out'] = (
                self._end_position - self._taper_length_out,
                self._end_position
            )
        
        # Termination zone
        if self._termination_zone_length > 0:
            zones['termination'] = (
                self._end_position,
                self._end_position + self._termination_zone_length
            )
        
        return zones
    
    def _calculate_lateral_offset(self, position, zone_type, zone_start, zone_end, object_type=None):
        """Calculate the lateral offset for an object based on its position and zone.
        
        Args:
            position: Longitudinal position on the lane
            zone_type: Type of construction zone
            zone_start: Start position of the zone
            zone_end: End position of the zone
            object_type: Type of object being placed (for special handling of signs)
            
        Returns:
            float: Lateral offset in meters
        """
        lane_index = int(self._lane_id.split('_')[-1])
        is_left_lane = lane_index > 1
        # Special handling for warning signs - place on shoulder
        if object_type == 'sign' and zone_type in ['warning', 'termination']:
            return self._warning_sign_offset  # Negative value places on right shoulder
        
        if zone_type in ['warning', 'termination']:
            # Cones in warning/termination zones stay in lane center
            return 0.0
        
        elif zone_type == 'taper_in':
            # Gradual offset increase from right edge to work zone
            zone_length = zone_end - zone_start
            if zone_length <= 0:
                # Start at appropriate edge based on lane type
                if is_left_lane:
                    return self._lane_width / 2 - 0.3  # Start at left edge (positive = left)
                else:
                    return -(self._lane_width / 2 - 0.3)  # Start at right edge (negative = right)
            progress = (position - zone_start) / zone_length
            

            if is_left_lane:
                edge_offset = self._lane_width / 2 - 0.3  # Positive for left side
            else:
                edge_offset = -(self._lane_width / 2 - 0.3)  # Negative for right side
            
            if self._taper_type == 'linear':
                offset = edge_offset + progress * (self._work_zone_offset - edge_offset)
            elif self._taper_type == 'curved':
                # S-curve transition for smoother flow
                s_curve = 3 * progress**2 - 2 * progress**3
                offset = edge_offset + s_curve * (self._work_zone_offset - edge_offset)
            else:
                offset = edge_offset + progress * (self._work_zone_offset - edge_offset)
            
            # Ensure offset stays within lane boundaries
            max_left_offset = self._lane_width / 2 - 0.3  # Leave 0.3m margin
            max_right_offset = -(self._lane_width / 2 - 0.3)
            return max(max_right_offset, min(offset, max_left_offset))
        
        elif zone_type in ['buffer', 'work']:
            # Full offset in work zone, but ensure within lane boundaries
            max_left_offset = self._lane_width / 2 - 0.3  # Leave 0.3m margin on left
            max_right_offset = -(self._lane_width / 2 - 0.3)  # Leave 0.3m margin on right
            return max(max_right_offset, min(self._work_zone_offset, max_left_offset))
        
        elif zone_type == 'taper_out':
            # Gradual offset decrease from work zone to right edge
            zone_length = zone_end - zone_start
            if zone_length <= 0:
                return self._work_zone_offset
            progress = (position - zone_start) / zone_length
            
            if is_left_lane:
                edge_offset = self._lane_width / 2 - 0.3  # Positive for left side
            else:
                edge_offset = -(self._lane_width / 2 - 0.3)  # Negative for right side
            
            if self._taper_type == 'linear':
                offset = self._work_zone_offset + progress * (edge_offset - self._work_zone_offset)
            elif self._taper_type == 'curved':
                # S-curve transition
                s_curve = 3 * progress**2 - 2 * progress**3
                offset = self._work_zone_offset + s_curve * (edge_offset - self._work_zone_offset)
            else:
                offset = self._work_zone_offset + progress * (edge_offset - self._work_zone_offset)
            
            # Ensure offset stays within lane boundaries
            max_left_offset = self._lane_width / 2 - 0.3  # Leave 0.3m margin
            max_right_offset = -(self._lane_width / 2 - 0.3)
            return max(max_right_offset, min(offset, max_left_offset))
        
        return 0.0
    
    def _calculate_shoulder_coordinates(self, lane_position):
        """Calculate the actual shoulder coordinates for placing warning signs.
        
        Args:
            lane_position: Position along the lane in meters
            
        Returns:
            tuple: (x, y, angle) coordinates for shoulder placement
        """
        # Get lane center coordinates at this position
        edge_id = traci.lane.getEdgeID(self._lane_id)
        lane_index = int(self._lane_id.split('_')[-1])  # Extract lane index from lane ID
        x_center, y_center = traci.simulation.convert2D(edge_id, lane_position, lane_index)
        is_left_lane = lane_index > 1
        
        # Get lane angle at this position
        lane_angle = traci.lane.getAngle(self._lane_id, lane_position)
        
        # Calculate perpendicular angle (90 degrees to the right)
        # In SUMO, angles are in degrees, 0 is North, clockwise positive
        perpendicular_angle = (-lane_angle) % 360
        perpendicular_rad = math.radians(perpendicular_angle)
        
        # Calculate offset distance (lane width/2 + shoulder offset)
        offset_distance = self._lane_width / 2 + abs(self._warning_sign_offset)
        
        # Calculate shoulder coordinates
        # Note: SUMO uses a different coordinate system where y increases northward
        if is_left_lane:
            # For left lane, place sign on left shoulder (subtract offset)
            x_shoulder = x_center - offset_distance * math.cos(perpendicular_rad)
            y_shoulder = y_center - offset_distance * math.sin(perpendicular_rad)
        else:
            # For right lane, place sign on right shoulder (add offset)
            x_shoulder = x_center + offset_distance * math.cos(perpendicular_rad)
            y_shoulder = y_center + offset_distance * math.sin(perpendicular_rad)
        
        return x_shoulder, y_shoulder, lane_angle
    
    def _place_object(self, position, lateral_offset, object_type, zone_type):
        """Place a single construction object at the specified position.
        
        Args:
            position: Longitudinal position on the lane
            lateral_offset: Lateral offset from lane center
            object_type: Type of object ('cone', 'barrier', 'sign')
            zone_type: Zone type for logging and ID generation
        """
        # Create unique object ID
        object_id = f"CONSTRUCTION_{zone_type}_{self._lane_id}_{len(self._construction_object_ids)}"
        self._construction_object_ids.append(object_id)
        
        # Add vehicle to simulation
        traci.vehicle.add(
            object_id,
            routeID=self._route_id,
            typeID=object_type,
        )
        
        # Set vehicle properties
        traci.vehicle.setSpeedMode(object_id, 0)
        traci.vehicle.setLaneChangeMode(object_id, 0)
        
        # Check if this is a warning sign that should be placed on shoulder
        type_name = None
        if object_type == self._sign_type:
            type_name = 'sign'
        
        if type_name == 'sign' and zone_type in ['warning', 'termination']:
            # Special handling for warning signs - place on shoulder using moveToXY
            x_shoulder, y_shoulder, angle = self._calculate_shoulder_coordinates(position)
            
            # Use moveToXY to place sign on shoulder
            traci.vehicle.moveToXY(
                object_id,
                "",  # Empty string allows placement anywhere
                -1,  # Lane index -1 means any lane
                x_shoulder,
                y_shoulder,
                angle,  # Keep parallel to road
                keepRoute=2  # 2 = ignore route, force placement
            )
            logger.debug(f"Placed warning sign {object_id} on shoulder at ({x_shoulder:.1f}, {y_shoulder:.1f})")
        else:
            # Normal placement for cones and barriers
            traci.vehicle.moveTo(object_id, self._lane_id, position)
            
            # Apply lateral offset for non-sign objects
            if lateral_offset != 0:
                try:
                    traci.vehicle.changeSublane(object_id, lateral_offset)
                except:
                    logger.debug(f"Could not apply lateral offset {lateral_offset} to {object_id}")
        
        # Set speed to 0 for all objects
        traci.vehicle.setSpeed(object_id, 0)
    
    def _calculate_dynamic_spacing(self, zone_type):
        """Calculate spacing based on MUTCD standards and speed limit."""
        if not self._use_dynamic_spacing or self._speed_limit is None:
            # Use default spacing if dynamic spacing is disabled
            if zone_type == 'warning':
                return self._warning_zone_spacing
            elif zone_type in ['taper_in', 'taper_out']:
                return self._spacing * 0.7
            elif zone_type == 'buffer':
                return self._spacing * 0.8
            else:
                return self._spacing
        
        # MUTCD speed-based spacing (in meters)
        # Convert mph to m/s first: 1 mph = 0.44704 m/s
        # Then apply MUTCD formula: spacing = speed limit in feet
        mph_to_meters = 0.3048  # 1 foot = 0.3048 meters
        
        if zone_type in ['taper_in', 'taper_out']:
            # Taper: spacing = speed limit in feet (converted to meters)
            return self._speed_limit * mph_to_meters
        elif zone_type in ['work', 'buffer']:
            # Tangent: spacing = 2 * speed limit in feet (converted to meters)
            return 2 * self._speed_limit * mph_to_meters
        elif zone_type in ['warning', 'termination']:
            # Warning zones: typically larger spacing
            return max(30.0, 3 * self._speed_limit * mph_to_meters)
        else:
            return self._spacing
    
    def _create_construction_objects(self):
        """Create construction objects with proper zone-based placement."""
        edge_id = traci.lane.getEdgeID(self._lane_id)
        
        # Create route for construction objects
        self._route_id = f"r_construction_{self._lane_id}"
        if self._route_id not in traci.route.getIDList():
            traci.route.add(self._route_id, [edge_id])
        
        # Create object types and store them as instance variables for comparison
        self._cone_type = create_construction_cone_type()
        self._barrier_type = create_construction_barrier_type()
        self._sign_type = create_construction_sign_type()
        
        # Calculate zones
        zones = self._calculate_zone_positions()
        
        # Process each zone
        for zone_type, (zone_start, zone_end) in zones.items():
            # Calculate dynamic spacing based on speed limit
            spacing = self._calculate_dynamic_spacing(zone_type)
            
            # Determine object type for this zone
            if zone_type == 'warning':
                object_types = ['sign']  # Only warning signs in warning zone
            elif zone_type in ['taper_in', 'taper_out']:
                object_types = ['cone']
            elif zone_type == 'buffer':
                object_types = ['cone', 'cone', 'barrier']  # Mostly cones, some barriers
            elif zone_type == 'work':
                if self._construction_type == 'mixed':
                    object_types = ['cone', 'cone', 'barrier']
                else:
                    object_types = [self._construction_type]
            elif zone_type == 'termination':
                object_types = ['sign']
            else:
                continue
            
            # Place objects in this zone
            current_pos = zone_start
            object_index = 0
            
            while current_pos < zone_end:
                # Select object type
                obj_type_name = object_types[object_index % len(object_types)]
                if obj_type_name == 'cone':
                    type_id = self._cone_type
                elif obj_type_name == 'barrier':
                    type_id = self._barrier_type
                elif obj_type_name == 'sign':
                    type_id = self._sign_type
                
                # Calculate lateral offset for this position
                lateral_offset = self._calculate_lateral_offset(
                    current_pos, zone_type, zone_start, zone_end, obj_type_name
                )
                
                # Place the object
                self._place_object(current_pos, lateral_offset, type_id, zone_type)
                
                current_pos += spacing
                object_index += 1
        
        logger.info(f"Created {len(self._construction_object_ids)} construction objects in {len(zones)} zones on lane {self._lane_id}")
    
    def initialize(self, time: float):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."

        # Check for and remove vehicles in the construction zone (except stalled vehicle)
        if self._start_position is not None and self._end_position is not None:
            # Get all vehicles on the lane
            vehicles_on_lane = traci.lane.getLastStepVehicleIDs(self._lane_id)

            for vehicle_id in vehicles_on_lane:
                # Skip if this is a stalled vehicle (check if it's marked as stalled)
                # Stalled vehicles typically have "stalled" or "STALLED" in their ID
                if "stalled" in vehicle_id.lower() or "STALLED" in vehicle_id:
                    logger.debug(f"Skipping stalled vehicle {vehicle_id} in construction zone")
                    continue

                # Get vehicle position on the lane
                try:
                    vehicle_pos = traci.vehicle.getLanePosition(vehicle_id)

                    # Check if vehicle is inside the construction zone
                    if self._start_position <= vehicle_pos <= self._end_position:
                        logger.info(f"Removing vehicle {vehicle_id} from construction zone at position {vehicle_pos}")
                        traci.vehicle.remove(vehicle_id)
                except Exception as e:
                    logger.debug(f"Could not check/remove vehicle {vehicle_id}: {e}")

        if self._construction_mode == "full_lane":
            # Original behavior: block entire lane
            traci.lane.setDisallowed(self._lane_id, ["all"])
        else:
            # Partial lane closure: use construction objects
            self._create_construction_objects()
            self._is_active = True

    def update(self, time: float):
        """Update the adversarial event.
        """
        if self._construction_mode == "partial_lane" and self._is_active:
            # For zone-based construction, we need to maintain positions more carefully
            zones = self._calculate_zone_positions()
            
            # Keep track of object positions
            for object_id in self._construction_object_ids:
                if object_id in traci.vehicle.getIDList():
                    # Extract zone type from object ID
                    parts = object_id.split('_')
                    if len(parts) >= 4:
                        zone_type = parts[2]
                        
                        # Maintain position and speed
                        try:
                            traci.vehicle.setSpeed(object_id, 0)
                        except:
                            logger.debug(f"Failed to maintain {object_id}")
                    
            # Check if we need to remove objects (based on end_time)
            if self.end_time != -1 and time >= self.end_time:
                for object_id in self._construction_object_ids:
                    try:
                        traci.vehicle.remove(object_id)
                    except:
                        logger.debug(f"Failed to remove {object_id}")
                self._is_active = False
        elif self._construction_mode == "full_lane" and self.end_time != -1 and time >= self.end_time:
            # Re-allow traffic on the lane
            try:
                traci.lane.setAllowed(self._lane_id, [])  # Empty list means all allowed
            except:
                logger.debug(f"Failed to re-open lane {self._lane_id}")
            self._is_active = False