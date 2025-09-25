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
        # Extract lane_ids parameter (backward compatible with lane_id)
        self._lane_ids = kwargs.pop("lane_ids", None)

        # Extract closure direction for multiple lanes
        self._closure_direction = kwargs.pop("closure_direction", "right")  # "left" or "right"

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

        # Handle backward compatibility for lane_id/lane_ids
        if self._lane_ids is None:
            # Check if parent class set _lane_id from kwargs
            if hasattr(self, '_lane_id') and self._lane_id:
                self._lane_ids = [self._lane_id]
            else:
                self._lane_ids = []
        elif not isinstance(self._lane_ids, list):
            # Convert single lane_id string to list
            self._lane_ids = [self._lane_ids]

        # For backward compatibility, set _lane_id to first lane if available
        if self._lane_ids and len(self._lane_ids) > 0:
            self._lane_id = self._lane_ids[0]
        elif not hasattr(self, '_lane_id'):
            self._lane_id = ""

        # Initialize other attributes
        self._construction_object_ids = []

        # Dictionary to store lane information for multiple lanes
        self._lane_info = {}  # {lane_id: {'length': float, 'width': float}}

        # If work_zone_offset not specified, use lane_offset
        if self._work_zone_offset is None:
            self._work_zone_offset = self._lane_offset

    def get_boundary_lane(self):
        """Get the boundary lane ID based on closure direction.

        For multiple lane closures, returns the lane at the boundary of the closure:
        - If closure_direction is "right", returns the leftmost closed lane (highest index)
        - If closure_direction is "left", returns the rightmost closed lane (lowest index)

        In SUMO: Lane 0 is rightmost, higher numbers are further left.

        Returns:
            str: The lane ID at the closure boundary, or empty string if no lanes.
        """
        if not self._lane_ids:
            return ""

        if len(self._lane_ids) == 1:
            return self._lane_ids[0]

        # Sort lanes by their index (number at the end of the ID)
        sorted_lanes = sorted(self._lane_ids, key=lambda x: int(x.split('_')[-1]))

        if self._closure_direction == "right":
            # Right closure (closing right lanes): return leftmost closed lane (highest index)
            # Example: closing lanes 0,1,2 -> return lane 2 (leftmost of the closed lanes)
            return sorted_lanes[-1]
        else:  # "left"
            # Left closure (closing left lanes): return rightmost closed lane (lowest index)
            # Example: closing lanes 2,3 -> return lane 2 (rightmost of the closed lanes)
            return sorted_lanes[0]
    
    def _calculate_other_lanes_width(self, exclude_lane_id):
        """Calculate the total width of all lanes in the construction zone except the specified lane.

        Args:
            exclude_lane_id (str): Lane ID to exclude from the calculation

        Returns:
            float: Total width of other construction zone lanes in meters.
        """
        # Sort lanes by their index to ensure consistent results regardless of input order
        sorted_lanes = sorted(self._lane_ids, key=lambda x: int(x.split('_')[-1]))

        total_width = 0.0
        for lane_id in sorted_lanes:
            if lane_id == exclude_lane_id:
                continue  # Skip the excluded lane

            if lane_id in self._lane_info:
                total_width += self._lane_info[lane_id]['width']
            else:
                # Fallback to querying SUMO if not in cache
                try:
                    width = traci.lane.getWidth(lane_id)
                    total_width += width
                except:
                    logger.warning(f"Could not get width for lane {lane_id}, using default 3.2m")
                    total_width += 3.2  # Standard lane width fallback
        return total_width

    def is_effective(self):
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        if self._lane_id == "":
            logger.warning("Lane ID is not provided.")
            return False

        # Populate lane information dictionary for all configured lanes
        for lane_id in self._lane_ids:
            try:
                allowed_type_list = traci.lane.getAllowed(lane_id)
                lane_length = traci.lane.getLength(lane_id)
                lane_width = traci.lane.getWidth(lane_id)

                # Store in dictionary
                self._lane_info[lane_id] = {
                    'length': lane_length,
                    'width': lane_width
                }

                # Set backward compatibility attribute for primary lane
                if lane_id == self._lane_id:
                    self._lane_width = lane_width

            except:
                logger.warning(f"Failed to get lane information for {lane_id}.")
                return False

        # Get primary lane info for validation
        if self._lane_id in self._lane_info:
            lane_length = self._lane_info[self._lane_id]['length']
        else:
            logger.warning(f"Primary lane {self._lane_id} not found in lane info.")
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
        # Use closure direction to determine placement side
        is_left_closure = self._closure_direction == "left"
        # Special handling for warning signs - place on shoulder
        if object_type == 'sign' and zone_type in ['warning', 'termination']:
            return self._warning_sign_offset  # Negative value places on right shoulder
        
        if zone_type in ['warning', 'termination']:
            # Cones in warning/termination zones stay in lane center
            return 0.0
        
        elif zone_type == 'taper_in':
            # Gradual offset increase from edge of current lane plus other lanes to work zone
            zone_length = zone_end - zone_start
            # Use closure direction to determine placement side
            is_left_closure = self._closure_direction == "left"

            # Get boundary lane for reference
            boundary_lane = self.get_boundary_lane()
            other_lanes_width = self._calculate_other_lanes_width(boundary_lane)

            if zone_length <= 0:
                # Start at appropriate edge based on closure direction and other lanes
                if is_left_closure:
                    return self._lane_width / 2 - 0.3 + other_lanes_width  # Current lane edge + other lanes
                else:
                    return -(self._lane_width / 2 - 0.3 + other_lanes_width)  # Current lane edge + other lanes
            progress = (position - zone_start) / zone_length

            # Calculate edge offset: current lane's half-width + width of other lanes in construction zone
            if is_left_closure:
                edge_offset = self._lane_width / 2 - 0.3 + other_lanes_width  # Positive for left side
            else:
                edge_offset = -(self._lane_width / 2 - 0.3 + other_lanes_width)  # Negative for right side
            
            if self._taper_type == 'linear':
                offset = edge_offset + progress * (self._work_zone_offset - edge_offset)
            elif self._taper_type == 'curved':
                # S-curve transition for smoother flow
                s_curve = 3 * progress**2 - 2 * progress**3
                offset = edge_offset + s_curve * (self._work_zone_offset - edge_offset)
            else:
                offset = edge_offset + progress * (self._work_zone_offset - edge_offset)


            return offset
        
        elif zone_type in ['buffer', 'work']:
            # Full offset in work zone (already validated during initialization)
            return self._work_zone_offset
        
        elif zone_type == 'taper_out':
            # Gradual offset decrease from work zone to edge of current lane plus other lanes
            zone_length = zone_end - zone_start
            # Use closure direction to determine placement side
            is_left_closure = self._closure_direction == "left"

            # Get boundary lane for reference
            boundary_lane = self.get_boundary_lane()
            other_lanes_width = self._calculate_other_lanes_width(boundary_lane)

            if zone_length <= 0:
                return self._work_zone_offset
            progress = (position - zone_start) / zone_length

            # Calculate edge offset: current lane's half-width + width of other lanes in construction zone
            if is_left_closure:
                edge_offset = self._lane_width / 2 - 0.3 + other_lanes_width  # Positive for left side
            else:
                edge_offset = -(self._lane_width / 2 - 0.3 + other_lanes_width)  # Negative for right side
            
            if self._taper_type == 'linear':
                offset = self._work_zone_offset + progress * (edge_offset - self._work_zone_offset)
            elif self._taper_type == 'curved':
                # S-curve transition
                s_curve = 3 * progress**2 - 2 * progress**3
                offset = self._work_zone_offset + s_curve * (edge_offset - self._work_zone_offset)
            else:
                offset = self._work_zone_offset + progress * (edge_offset - self._work_zone_offset)

            return offset
        
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
        # Use closure direction to determine placement side
        is_left_closure = self._closure_direction == "left"
        
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
        if is_left_closure:
            # For left closure, place sign on left shoulder (subtract offset)
            x_shoulder = x_center - offset_distance * math.cos(perpendicular_rad)
            y_shoulder = y_center - offset_distance * math.sin(perpendicular_rad)
        else:
            # For right closure, place sign on right shoulder (add offset)
            x_shoulder = x_center + offset_distance * math.cos(perpendicular_rad)
            y_shoulder = y_center + offset_distance * math.sin(perpendicular_rad)
        
        return x_shoulder, y_shoulder, lane_angle
    
    def _place_object(self, position, lateral_offset, object_type, zone_type, lane_id=None):
        """Place a single construction object at the specified position.

        Args:
            position: Longitudinal position on the lane
            lateral_offset: Lateral offset from lane center
            object_type: Type of object ('cone', 'barrier', 'sign')
            zone_type: Zone type for logging and ID generation
            lane_id: Optional lane ID to place object on (defaults to self._lane_id)
        """
        current_lane = lane_id or self._lane_id
        # Create unique object ID
        object_id = f"CONSTRUCTION_{zone_type}_{current_lane}_{len(self._construction_object_ids)}"
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
            traci.vehicle.moveTo(object_id, current_lane, position)
            
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
        # Get boundary lane for cone placement in work zone
        boundary_lane = self.get_boundary_lane()

        # Create object types and store them as instance variables for comparison
        self._cone_type = create_construction_cone_type()
        self._barrier_type = create_construction_barrier_type()
        self._sign_type = create_construction_sign_type()

        # Calculate zones based on first lane (they should be same for all lanes)
        zones = self._calculate_zone_positions()

        # Process each zone
        for zone_type, (zone_start, zone_end) in zones.items():
            # Calculate dynamic spacing based on speed limit
            spacing = self._calculate_dynamic_spacing(zone_type)

            # For work zone, only place cones on boundary lane
            if zone_type == 'work':
                if not boundary_lane:
                    continue

                # Create route for this lane if not exists
                edge_id = traci.lane.getEdgeID(boundary_lane)
                route_id = f"r_construction_{boundary_lane}"
                if route_id not in traci.route.getIDList():
                    traci.route.add(route_id, [edge_id])
                self._route_id = route_id

                # Place cones only on boundary lane
                current_pos = zone_start
                while current_pos < zone_end:
                    # Calculate lateral offset for this position
                    lateral_offset = self._calculate_lateral_offset(
                        current_pos, zone_type, zone_start, zone_end, 'cone'
                    )

                    # Place cone on boundary lane
                    object_id = f"CONSTRUCTION_{zone_type}_{boundary_lane}_{len(self._construction_object_ids)}"
                    self._construction_object_ids.append(object_id)

                    traci.vehicle.add(
                        object_id,
                        routeID=route_id,
                        typeID=self._cone_type,
                    )

                    traci.vehicle.setSpeedMode(object_id, 0)
                    traci.vehicle.setLaneChangeMode(object_id, 0)
                    traci.vehicle.moveTo(object_id, boundary_lane, current_pos)

                    if lateral_offset != 0:
                        try:
                            traci.vehicle.changeSublane(object_id, lateral_offset)
                        except:
                            logger.debug(f"Could not apply lateral offset {lateral_offset} to {object_id}")

                    traci.vehicle.setSpeed(object_id, 0)
                    current_pos += spacing

            else:
                # For non-work zones, also place on boundary lane for consistency
                if not boundary_lane:
                    continue

                edge_id = traci.lane.getEdgeID(boundary_lane)
                route_id = f"r_construction_{boundary_lane}"
                if route_id not in traci.route.getIDList():
                    traci.route.add(route_id, [edge_id])
                self._route_id = route_id

                # Determine object type for this zone
                if zone_type == 'warning':
                    object_types = ['sign']  # Only warning signs in warning zone
                elif zone_type in ['taper_in', 'taper_out']:
                    object_types = ['cone']
                elif zone_type == 'buffer':
                    object_types = ['cone', 'cone', 'barrier']  # Mostly cones, some barriers
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

                    # Place the object on boundary lane
                    self._place_object(current_pos, lateral_offset, type_id, zone_type, boundary_lane)

                    current_pos += spacing
                    object_index += 1

        logger.info(f"Created {len(self._construction_object_ids)} construction objects in zones, with work zone cones on boundary lane {boundary_lane}")
    
    def initialize(self, time: float):
        """Initialize the adversarial event.
        """
        assert self.is_effective(), "Adversarial event is not effective."

        # Validate and correct work zone offset at initialization
        max_left_offset = self._lane_width / 2 - 0.3  # Leave 0.3m margin on left
        max_right_offset = -(self._lane_width / 2 - 0.3)  # Leave 0.3m margin on right

        # Clamp work zone offset to valid range
        if self._work_zone_offset > max_left_offset:
            logger.warning(f"Work zone offset {self._work_zone_offset} exceeds max left offset {max_left_offset}, clamping to max")
            self._work_zone_offset = max_left_offset
        elif self._work_zone_offset < max_right_offset:
            logger.warning(f"Work zone offset {self._work_zone_offset} exceeds max right offset {max_right_offset}, clamping to max")
            self._work_zone_offset = max_right_offset

        # Check for and remove vehicles in the construction zone (except stalled vehicle)
        if self._start_position is not None and self._end_position is not None:
            # Check all lanes in the construction zone
            for lane_id in self._lane_ids:
                # Get all vehicles on each construction lane
                vehicles_on_lane = traci.lane.getLastStepVehicleIDs(lane_id)

                for vehicle_id in vehicles_on_lane:
                    # Skip if this is a stalled vehicle (check if it's marked as stalled)
                    # Stalled vehicles typically have "stalled" or "STALLED" in their ID
                    if "stalled" in vehicle_id.lower() or "STALLED" in vehicle_id:
                        logger.debug(f"Skipping stalled vehicle {vehicle_id} in construction zone on lane {lane_id}")
                        continue

                    # Get vehicle position on the lane
                    try:
                        vehicle_pos = traci.vehicle.getLanePosition(vehicle_id)

                        # Check if vehicle is inside the construction zone
                        if self._start_position <= vehicle_pos <= self._end_position:
                            logger.info(f"Removing vehicle {vehicle_id} from construction zone at position {vehicle_pos} on lane {lane_id}")
                            traci.vehicle.remove(vehicle_id)
                    except Exception as e:
                        logger.debug(f"Could not check/remove vehicle {vehicle_id} on lane {lane_id}: {e}")

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