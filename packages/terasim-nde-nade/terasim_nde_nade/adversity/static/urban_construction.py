from loguru import logger
import math
from typing import List, Dict, Tuple, Optional

from terasim.overlay import traci

from .construction import ConstructionAdversity
from ...utils import AbstractStaticAdversity


class UrbanConstructionAdversity(ConstructionAdversity):
    """
    Urban construction zone adversity that can span multiple intersections.
    Extends the base ConstructionAdversity to handle complex urban scenarios.
    """
    
    def __init__(self, **kwargs):
        # Extract urban-specific parameters
        self._corridor_config = kwargs.pop("corridor_definition", {})
        self._lane_config = kwargs.pop("lane_configuration", {})
        self._intersection_config = kwargs.pop("intersection_handling", {})
        self._vehicle_config = kwargs.pop("construction_vehicles", {})
        self._worker_config = kwargs.pop("workers", {})
        self._zone_config = kwargs.pop("zones", {})
        
        # Initialize corridor edges list
        self._corridor_edges = []
        self._intersections = []
        self._merge_points = []
        
        # Construction assets
        self._construction_vehicle_ids = []
        self._worker_ids = []
        
        # Call parent constructor
        super().__init__(**kwargs)
        
    def is_effective(self) -> bool:
        """
        Check if the urban construction zone can be created.
        
        Returns:
            bool: True if the construction zone can be created
        """
        # Find corridor edges based on configuration
        try:
            self._corridor_edges = self._find_corridor_edges()
            if not self._corridor_edges:
                logger.warning("No corridor edges found")
                return False
                
            # Validate each edge exists
            for edge in self._corridor_edges:
                if edge not in traci.edge.getIDList():
                    logger.warning(f"Edge {edge} does not exist")
                    return False
                    
            # Find intersections in the corridor
            self._intersections = self._find_corridor_intersections()
            logger.info(f"Found {len(self._intersections)} intersections in corridor")
            
            return True
            
        except Exception as e:
            logger.error(f"Error validating urban construction zone: {e}")
            return False
            
    def _find_corridor_edges(self) -> List[str]:
        """
        Find all edges that make up the construction corridor.
        
        Returns:
            List[str]: List of edge IDs in the corridor
        """
        mode = self._corridor_config.get("mode", "intersection_count")
        
        if mode == "intersection_count":
            return self._find_by_intersection_count(
                start_edge=self._corridor_config.get("start_edge"),
                num_intersections=self._corridor_config.get("num_intersections", 1),
                direction=self._corridor_config.get("direction", "forward")
            )
        elif mode == "explicit":
            return self._corridor_config.get("edges", [])
        elif mode == "path":
            return self._find_path_edges(
                start=self._corridor_config.get("start_edge"),
                end=self._corridor_config.get("end_edge"),
                path_type=self._corridor_config.get("path_type", "shortest")
            )
        else:
            logger.warning(f"Unknown corridor mode: {mode}")
            return []
            
    def _find_by_intersection_count(self, start_edge: str, num_intersections: int, 
                                   direction: str = "forward") -> List[str]:
        """
        Find corridor edges by traversing a specified number of intersections.
        
        Args:
            start_edge: Starting edge ID
            num_intersections: Number of intersections to traverse
            direction: Direction to traverse ("forward", "backward", or "both")
            
        Returns:
            List[str]: Ordered list of edge IDs
        """
        if not start_edge or start_edge not in traci.edge.getIDList():
            logger.error(f"Invalid start edge: {start_edge}")
            return []
            
        corridor_edges = [start_edge]
        intersections_crossed = 0
        current_edge = start_edge
        visited_edges = {start_edge}
        
        while intersections_crossed < num_intersections:
            # Get connected edges based on direction
            if direction == "forward":
                # Get the junction at the end of current edge
                to_junction = traci.edge.getToJunction(current_edge)
                if not to_junction:
                    break
                next_edges = traci.junction.getOutgoingEdges(to_junction)
            elif direction == "backward":
                # Get the junction at the start of current edge  
                from_junction = traci.edge.getFromJunction(current_edge)
                if not from_junction:
                    break
                next_edges = traci.junction.getIncomingEdges(from_junction)
            else:  # both
                to_junction = traci.edge.getToJunction(current_edge)
                from_junction = traci.edge.getFromJunction(current_edge)
                next_edges = []
                if to_junction:
                    next_edges.extend(traci.junction.getOutgoingEdges(to_junction))
                if from_junction:
                    next_edges.extend(traci.junction.getIncomingEdges(from_junction))
                    
            if not next_edges:
                logger.warning(f"No more edges found after {current_edge}")
                break
                
            # Filter out already visited edges
            next_edges = [e for e in next_edges if e not in visited_edges]
            if not next_edges:
                logger.warning(f"All connected edges already visited")
                break
                
            # Select the main road (highest priority)
            next_edge = self._select_main_road(next_edges)
            
            # Check if we're crossing a valid intersection
            junction = traci.edge.getToJunction(current_edge) if direction == "forward" else traci.edge.getFromJunction(current_edge)
            if self._is_valid_intersection(junction):
                intersections_crossed += 1
                logger.debug(f"Crossed intersection {junction}, count: {intersections_crossed}")
                
            corridor_edges.append(next_edge)
            visited_edges.add(next_edge)
            current_edge = next_edge
            
        logger.info(f"Found corridor with {len(corridor_edges)} edges crossing {intersections_crossed} intersections")
        return corridor_edges
        
    def _select_main_road(self, edges: List[str]) -> str:
        """
        Select the main road from a list of edges based on priority.
        
        Args:
            edges: List of edge IDs to choose from
            
        Returns:
            str: The selected edge ID
        """
        if len(edges) == 1:
            return edges[0]
            
        best_edge = edges[0]
        best_priority = -1
        best_lanes = 0
        
        for edge in edges:
            # Get edge attributes
            try:
                edge_type = traci.edge.getType(edge)
                num_lanes = traci.edge.getLaneNumber(edge)
                
                # Extract priority from type (e.g., "highway.primary" has higher priority)
                priority = self._get_priority_from_type(edge_type)
                
                # Select based on priority first, then number of lanes
                if priority > best_priority or (priority == best_priority and num_lanes > best_lanes):
                    best_priority = priority
                    best_lanes = num_lanes
                    best_edge = edge
            except:
                continue
                
        return best_edge
        
    def _get_priority_from_type(self, edge_type: str) -> int:
        """
        Get priority value from edge type string.
        
        Args:
            edge_type: Edge type string (e.g., "highway.primary")
            
        Returns:
            int: Priority value (higher is more important)
        """
        priority_map = {
            "highway.motorway": 15,
            "highway.trunk": 14,
            "highway.primary": 12,
            "highway.secondary": 11,
            "highway.tertiary": 10,
            "highway.unclassified": 4,
            "highway.residential": 3,
            "highway.service": 1
        }
        
        for key, priority in priority_map.items():
            if key in edge_type:
                return priority
                
        return 0
        
    def _is_valid_intersection(self, junction_id: str) -> bool:
        """
        Check if a junction is a valid intersection (not a dead end).
        
        Args:
            junction_id: Junction ID to check
            
        Returns:
            bool: True if it's a valid intersection
        """
        if not junction_id:
            return False
            
        try:
            # Check if it's an internal junction (starts with ':')
            if junction_id.startswith(':'):
                return False
                
            # Check number of connected edges - valid intersection should have more than 2
            incoming = traci.junction.getIncomingEdges(junction_id)
            outgoing = traci.junction.getOutgoingEdges(junction_id)
            
            # Filter out internal edges (those starting with ':')
            incoming = [e for e in incoming if not e.startswith(':')]
            outgoing = [e for e in outgoing if not e.startswith(':')]
            
            # A valid intersection should have at least 3 total connections
            total_connections = len(set(incoming + outgoing))
            return total_connections >= 3
        except:
            return False
            
    def _find_corridor_intersections(self) -> List[Dict]:
        """
        Find all intersections along the corridor.
        
        Returns:
            List[Dict]: List of intersection information
        """
        intersections = []
        
        for i, edge in enumerate(self._corridor_edges):
            # Check junction at the end of this edge
            to_junction = traci.edge.getToJunction(edge)
            
            if self._is_valid_intersection(to_junction):
                # Don't duplicate if it's the last edge
                if i < len(self._corridor_edges) - 1:
                    intersections.append({
                        'junction_id': to_junction,
                        'edge_before': edge,
                        'edge_after': self._corridor_edges[i + 1] if i + 1 < len(self._corridor_edges) else None,
                        'position': traci.junction.getPosition(to_junction)
                    })
                    
        return intersections
        
    def _calculate_merge_points(self):
        """
        Calculate merge points before each intersection.
        """
        self._merge_points = []
        merge_distance = self._intersection_config.get("merge_distance", 150)
        
        for intersection in self._intersections:
            edge_before = intersection['edge_before']
            if not edge_before:
                continue
                
            edge_length = traci.lane.getLength(f"{edge_before}_0")
            merge_start = max(0, edge_length - merge_distance)
            
            self._merge_points.append({
                'edge': edge_before,
                'start_pos': merge_start,
                'end_pos': edge_length,
                'junction': intersection['junction_id'],
                'taper_length': min(merge_distance, edge_length)
            })
            
        logger.info(f"Calculated {len(self._merge_points)} merge points")
        
    def _create_construction_zones(self):
        """
        Create construction zones along the corridor with proper MUTCD structure.
        """
        # Calculate merge points first
        self._calculate_merge_points()
        
        # Create zones for each edge
        for edge in self._corridor_edges:
            # Check if this edge has a merge point
            merge_point = None
            for mp in self._merge_points:
                if mp['edge'] == edge:
                    merge_point = mp
                    break
                    
            # Create zones for this edge
            self._create_edge_zones(edge, merge_point)
            
    def _create_edge_zones(self, edge: str, merge_point: Optional[Dict] = None):
        """
        Create construction zones for a single edge.
        
        Args:
            edge: Edge ID
            merge_point: Merge point information if this edge has one
        """
        edge_length = traci.lane.getLength(f"{edge}_0")
        
        # Determine which lanes to close
        lanes_to_close = self._lane_config.get("lanes_to_close", [0])
        num_lanes = traci.edge.getLaneNumber(edge)
        
        # Warning zone (start of edge)
        warning_length = self._zone_config.get("warning", {}).get("length", 200)
        warning_start = 0
        warning_end = min(warning_length, edge_length * 0.3)
        
        # Work zone (main construction area)
        if merge_point:
            # If there's a merge point, work zone ends before it
            work_start = warning_end
            work_end = merge_point['start_pos']
        else:
            # Otherwise, work zone covers most of the edge
            work_start = warning_end
            work_end = edge_length * 0.9
            
        # Place construction objects
        self._place_zone_objects(edge, warning_start, warning_end, "warning")
        self._place_zone_objects(edge, work_start, work_end, "work")
        
        # Handle merge taper if present
        if merge_point:
            self._create_merge_taper(
                edge=merge_point['edge'],
                start_pos=merge_point['start_pos'],
                end_pos=merge_point['end_pos'],
                lanes_to_close=lanes_to_close
            )
            
    def _create_merge_taper(self, edge: str, start_pos: float, end_pos: float, 
                           lanes_to_close: List[int]):
        """
        Create a merge taper before an intersection.
        
        Args:
            edge: Edge ID
            start_pos: Start position of taper
            end_pos: End position of taper (at intersection)
            lanes_to_close: List of lane indices to close
        """
        taper_length = end_pos - start_pos
        cone_spacing = 5.0  # Dense spacing in taper zone
        num_cones = int(taper_length / cone_spacing)
        
        for i in range(num_cones + 1):
            position = start_pos + (i * cone_spacing)
            if position > end_pos:
                position = end_pos
                
            # Calculate progress through taper (0 to 1)
            progress = i / max(num_cones, 1)
            
            # For merge-before strategy, gradually move cones from right lane edge to block the lane
            for lane_idx in lanes_to_close:
                lane_id = f"{edge}_{lane_idx}"
                lane_width = traci.lane.getWidth(lane_id)
                
                # Calculate lateral offset
                # Start at right edge of lane, move to left edge to block it
                start_offset = -lane_width / 2 + 0.3  # Start near right edge
                end_offset = lane_width / 2 - 0.3  # End near left edge (blocking lane)
                lateral_offset = start_offset + (end_offset - start_offset) * progress
                
                # Create cone
                cone_id = f"TAPER_CONE_{edge}_{lane_idx}_{i}"
                self._place_cone_at_position(cone_id, edge, lane_idx, position, lateral_offset)
                
        # Place merge warning signs
        self._place_merge_signs(edge, start_pos)
        
    def _place_zone_objects(self, edge: str, start_pos: float, end_pos: float, zone_type: str):
        """
        Place construction objects in a zone.
        
        Args:
            edge: Edge ID
            start_pos: Start position of zone
            end_pos: End position of zone
            zone_type: Type of zone ("warning", "work", "buffer")
        """
        # Determine spacing based on zone type
        if zone_type == "warning":
            spacing = 30.0  # Sparse in warning zone
            object_type = "sign"
        elif zone_type == "work":
            spacing = 10.0  # Regular in work zone
            object_type = "cone"
        else:
            spacing = 20.0
            object_type = "cone"
            
        # Place objects
        current_pos = start_pos
        obj_count = 0
        
        while current_pos <= end_pos:
            if zone_type == "warning" and obj_count < 3:
                # Place warning signs
                sign_id = f"WARNING_SIGN_{edge}_{obj_count}"
                self._place_warning_sign(sign_id, edge, current_pos)
            elif zone_type == "work":
                # Place cones and barriers
                for lane_idx in self._lane_config.get("lanes_to_close", [0]):
                    # Place cone at lane edge
                    cone_id = f"WORK_CONE_{edge}_{lane_idx}_{obj_count}"
                    lane_id = f"{edge}_{lane_idx}"
                    lane_width = traci.lane.getWidth(lane_id)
                    lateral_offset = -lane_width / 2 + 0.5  # Right edge of lane
                    
                    self._place_cone_at_position(cone_id, edge, lane_idx, current_pos, lateral_offset)
                    
                    # Occasionally place barriers
                    if obj_count % 5 == 0:
                        barrier_id = f"BARRIER_{edge}_{lane_idx}_{obj_count}"
                        self._place_barrier_at_position(barrier_id, edge, lane_idx, current_pos, lateral_offset)
                        
            current_pos += spacing
            obj_count += 1
            
    def _place_cone_at_position(self, cone_id: str, edge: str, lane_idx: int, 
                               position: float, lateral_offset: float):
        """
        Place a traffic cone at a specific position.
        
        Args:
            cone_id: Unique ID for the cone
            edge: Edge ID
            lane_idx: Lane index
            position: Longitudinal position on lane
            lateral_offset: Lateral offset from lane center
        """
        if cone_id not in self._construction_object_ids:
            self._construction_object_ids.append(cone_id)
            
        # Create cone type if not exists
        cone_type = self._create_cone_type()
        
        # Create route for the cone
        route_id = f"route_{cone_id}"
        if route_id not in traci.route.getIDList():
            traci.route.add(route_id, [edge])
            
        # Add cone as vehicle
        traci.vehicle.add(
            cone_id,
            routeID=route_id,
            typeID=cone_type
        )
        
        # Position the cone
        lane_id = f"{edge}_{lane_idx}"
        traci.vehicle.moveTo(cone_id, lane_id, position)
        
        # Apply lateral offset
        if lateral_offset != 0:
            try:
                traci.vehicle.changeSublane(cone_id, lateral_offset)
            except:
                pass
                
        # Make it static
        traci.vehicle.setSpeed(cone_id, 0)
        traci.vehicle.setSpeedMode(cone_id, 0)
        traci.vehicle.setLaneChangeMode(cone_id, 0)
        
    def _place_barrier_at_position(self, barrier_id: str, edge: str, lane_idx: int,
                                  position: float, lateral_offset: float):
        """
        Place a concrete barrier at a specific position.
        
        Args:
            barrier_id: Unique ID for the barrier
            edge: Edge ID
            lane_idx: Lane index
            position: Longitudinal position on lane
            lateral_offset: Lateral offset from lane center
        """
        if barrier_id not in self._construction_object_ids:
            self._construction_object_ids.append(barrier_id)
            
        # Create barrier type if not exists
        barrier_type = self._create_barrier_type()
        
        # Create route
        route_id = f"route_{barrier_id}"
        if route_id not in traci.route.getIDList():
            traci.route.add(route_id, [edge])
            
        # Add barrier
        traci.vehicle.add(
            barrier_id,
            routeID=route_id,
            typeID=barrier_type
        )
        
        # Position the barrier
        lane_id = f"{edge}_{lane_idx}"
        traci.vehicle.moveTo(barrier_id, lane_id, position)
        
        # Apply lateral offset
        if lateral_offset != 0:
            try:
                traci.vehicle.changeSublane(barrier_id, lateral_offset)
            except:
                pass
                
        # Make it static
        traci.vehicle.setSpeed(barrier_id, 0)
        traci.vehicle.setSpeedMode(barrier_id, 0)
        traci.vehicle.setLaneChangeMode(barrier_id, 0)
        
    def _place_warning_sign(self, sign_id: str, edge: str, position: float):
        """
        Place a warning sign on the shoulder.
        
        Args:
            sign_id: Unique ID for the sign
            edge: Edge ID
            position: Longitudinal position
        """
        if sign_id not in self._construction_object_ids:
            self._construction_object_ids.append(sign_id)
            
        # Create sign type
        sign_type = self._create_sign_type()
        
        # Create route
        route_id = f"route_{sign_id}"
        if route_id not in traci.route.getIDList():
            traci.route.add(route_id, [edge])
            
        # Add sign
        traci.vehicle.add(
            sign_id,
            routeID=route_id,
            typeID=sign_type
        )
        
        # Position on rightmost lane
        num_lanes = traci.edge.getLaneNumber(edge)
        rightmost_lane = f"{edge}_0"
        traci.vehicle.moveTo(sign_id, rightmost_lane, position)
        
        # Move to shoulder (right side)
        lane_width = traci.lane.getWidth(rightmost_lane)
        shoulder_offset = -(lane_width / 2 + 2.0)  # 2m onto shoulder
        
        try:
            traci.vehicle.changeSublane(sign_id, shoulder_offset)
        except:
            pass
            
        # Make static
        traci.vehicle.setSpeed(sign_id, 0)
        traci.vehicle.setSpeedMode(sign_id, 0)
        traci.vehicle.setLaneChangeMode(sign_id, 0)
        
    def _place_merge_signs(self, edge: str, merge_start: float):
        """
        Place merge warning signs before a merge taper.
        
        Args:
            edge: Edge ID
            merge_start: Start position of merge taper
        """
        signs = self._intersection_config.get("merge_signs", ["Lane Closed Ahead", "Merge Left"])
        
        for i, sign_text in enumerate(signs):
            sign_pos = max(0, merge_start - (i + 1) * 50)  # 50m spacing before merge
            sign_id = f"MERGE_SIGN_{edge}_{i}"
            self._place_warning_sign(sign_id, edge, sign_pos)
            
    def _create_cone_type(self) -> str:
        """Create or get the construction cone vehicle type."""
        cone_type = "URBAN_CONE"
        
        if cone_type not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", cone_type)
            traci.vehicletype.setVehicleClass(cone_type, "passenger")
            traci.vehicletype.setLength(cone_type, 0.5)
            traci.vehicletype.setWidth(cone_type, 0.5)
            traci.vehicletype.setHeight(cone_type, 0.7)
            traci.vehicletype.setMinGap(cone_type, 0.1)
            traci.vehicletype.setColor(cone_type, (255, 140, 0, 255))  # Orange
            
        return cone_type
        
    def _create_barrier_type(self) -> str:
        """Create or get the construction barrier vehicle type."""
        barrier_type = "URBAN_BARRIER"
        
        if barrier_type not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", barrier_type)
            traci.vehicletype.setVehicleClass(barrier_type, "passenger")
            traci.vehicletype.setLength(barrier_type, 3.0)
            traci.vehicletype.setWidth(barrier_type, 0.8)
            traci.vehicletype.setHeight(barrier_type, 1.0)
            traci.vehicletype.setMinGap(barrier_type, 0.1)
            traci.vehicletype.setColor(barrier_type, (128, 128, 128, 255))  # Gray concrete
            
        return barrier_type
        
    def _create_sign_type(self) -> str:
        """Create or get the warning sign vehicle type."""
        sign_type = "URBAN_SIGN"
        
        if sign_type not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", sign_type)
            traci.vehicletype.setVehicleClass(sign_type, "passenger")
            traci.vehicletype.setLength(sign_type, 1.0)
            traci.vehicletype.setWidth(sign_type, 0.3)
            traci.vehicletype.setHeight(sign_type, 2.0)
            traci.vehicletype.setMinGap(sign_type, 0.1)
            traci.vehicletype.setColor(sign_type, (255, 255, 0, 255))  # Yellow warning
            
        return sign_type
        
    def _place_construction_vehicles(self):
        """
        Place static construction vehicles in the work zone.
        """
        if not self._vehicle_config:
            return
            
        # Place excavator
        if "excavator" in self._vehicle_config:
            self._place_excavator()
            
        # Place dump trucks
        if "dump_truck" in self._vehicle_config:
            self._place_dump_trucks()
            
        # Place warning vehicles
        if "warning_vehicle" in self._vehicle_config:
            self._place_warning_vehicles()
            
    def _place_excavator(self):
        """Place an excavator in the center of the work zone."""
        excavator_config = self._vehicle_config.get("excavator", {})
        if excavator_config.get("count", 0) <= 0:
            return
            
        # Find center of corridor
        middle_edge_idx = len(self._corridor_edges) // 2
        middle_edge = self._corridor_edges[middle_edge_idx]
        edge_length = traci.lane.getLength(f"{middle_edge}_0")
        center_pos = edge_length / 2
        
        # Create excavator type
        excavator_type = self._create_excavator_type()
        
        # Place excavator
        excavator_id = f"EXCAVATOR_{self._adversity_id}"
        route_id = f"route_{excavator_id}"
        
        if route_id not in traci.route.getIDList():
            traci.route.add(route_id, [middle_edge])
            
        traci.vehicle.add(
            excavator_id,
            routeID=route_id,
            typeID=excavator_type
        )
        
        # Position in closed lane
        closed_lane = self._lane_config.get("lanes_to_close", [0])[0]
        lane_id = f"{middle_edge}_{closed_lane}"
        traci.vehicle.moveTo(excavator_id, lane_id, center_pos)
        
        # Make static
        traci.vehicle.setSpeed(excavator_id, 0)
        traci.vehicle.setSpeedMode(excavator_id, 0)
        traci.vehicle.setLaneChangeMode(excavator_id, 0)
        
        self._construction_vehicle_ids.append(excavator_id)
        logger.info(f"Placed excavator at {middle_edge} position {center_pos}")
        
    def _place_dump_trucks(self):
        """Place dump trucks distributed along the corridor."""
        dump_config = self._vehicle_config.get("dump_truck", {})
        count = dump_config.get("count", 0)
        if count <= 0:
            return
            
        spacing = dump_config.get("spacing", 200)
        dump_type = self._create_dump_truck_type()
        
        # Distribute trucks along corridor
        total_length = sum(traci.lane.getLength(f"{e}_0") for e in self._corridor_edges)
        truck_spacing = total_length / (count + 1)
        
        current_distance = truck_spacing
        truck_count = 0
        
        for edge in self._corridor_edges:
            if truck_count >= count:
                break
                
            edge_length = traci.lane.getLength(f"{edge}_0")
            
            while current_distance <= edge_length and truck_count < count:
                # Place truck
                truck_id = f"DUMP_TRUCK_{self._adversity_id}_{truck_count}"
                route_id = f"route_{truck_id}"
                
                if route_id not in traci.route.getIDList():
                    traci.route.add(route_id, [edge])
                    
                traci.vehicle.add(
                    truck_id,
                    routeID=route_id,
                    typeID=dump_type
                )
                
                # Position in closed lane
                closed_lane = self._lane_config.get("lanes_to_close", [0])[0]
                lane_id = f"{edge}_{closed_lane}"
                traci.vehicle.moveTo(truck_id, lane_id, current_distance)
                
                # Make static
                traci.vehicle.setSpeed(truck_id, 0)
                traci.vehicle.setSpeedMode(truck_id, 0)
                traci.vehicle.setLaneChangeMode(truck_id, 0)
                
                self._construction_vehicle_ids.append(truck_id)
                truck_count += 1
                current_distance += truck_spacing
                
            current_distance -= edge_length
            
    def _place_warning_vehicles(self):
        """Place warning vehicles at zone boundaries."""
        warning_config = self._vehicle_config.get("warning_vehicle", {})
        positions = warning_config.get("placement", ["zone_start", "zone_end"])
        
        warning_type = self._create_warning_vehicle_type()
        
        for i, position in enumerate(positions):
            if position == "zone_start" and self._corridor_edges:
                edge = self._corridor_edges[0]
                pos = 50  # 50m into first edge
            elif position == "zone_end" and self._corridor_edges:
                edge = self._corridor_edges[-1]
                edge_length = traci.lane.getLength(f"{edge}_0")
                pos = edge_length - 50  # 50m before end
            else:
                continue
                
            vehicle_id = f"WARNING_VEHICLE_{self._adversity_id}_{i}"
            route_id = f"route_{vehicle_id}"
            
            if route_id not in traci.route.getIDList():
                traci.route.add(route_id, [edge])
                
            traci.vehicle.add(
                vehicle_id,
                routeID=route_id,
                typeID=warning_type
            )
            
            # Position on shoulder
            lane_id = f"{edge}_0"
            traci.vehicle.moveTo(vehicle_id, lane_id, pos)
            
            # Move to shoulder
            lane_width = traci.lane.getWidth(lane_id)
            shoulder_offset = -(lane_width / 2 + 1.5)
            
            try:
                traci.vehicle.changeSublane(vehicle_id, shoulder_offset)
            except:
                pass
                
            # Make static
            traci.vehicle.setSpeed(vehicle_id, 0)
            traci.vehicle.setSpeedMode(vehicle_id, 0)
            traci.vehicle.setLaneChangeMode(vehicle_id, 0)
            
            self._construction_vehicle_ids.append(vehicle_id)
            
    def _create_excavator_type(self) -> str:
        """Create excavator vehicle type."""
        excavator_type = "CONSTRUCTION_EXCAVATOR"
        
        if excavator_type not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", excavator_type)
            traci.vehicletype.setVehicleClass(excavator_type, "truck")
            traci.vehicletype.setLength(excavator_type, 8.0)
            traci.vehicletype.setWidth(excavator_type, 3.0)
            traci.vehicletype.setHeight(excavator_type, 3.5)
            traci.vehicletype.setMinGap(excavator_type, 0.5)
            traci.vehicletype.setColor(excavator_type, (255, 200, 0, 255))  # Yellow
            
        return excavator_type
        
    def _create_dump_truck_type(self) -> str:
        """Create dump truck vehicle type."""
        dump_type = "CONSTRUCTION_DUMP_TRUCK"
        
        if dump_type not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", dump_type)
            traci.vehicletype.setVehicleClass(dump_type, "truck")
            traci.vehicletype.setLength(dump_type, 10.0)
            traci.vehicletype.setWidth(dump_type, 2.5)
            traci.vehicletype.setHeight(dump_type, 3.0)
            traci.vehicletype.setMinGap(dump_type, 0.5)
            traci.vehicletype.setColor(dump_type, (255, 140, 0, 255))  # Orange
            
        return dump_type
        
    def _create_warning_vehicle_type(self) -> str:
        """Create warning vehicle type."""
        warning_type = "CONSTRUCTION_WARNING"
        
        if warning_type not in traci.vehicletype.getIDList():
            traci.vehicletype.copy("DEFAULT_VEHTYPE", warning_type)
            traci.vehicletype.setVehicleClass(warning_type, "passenger")
            traci.vehicletype.setLength(warning_type, 6.0)
            traci.vehicletype.setWidth(warning_type, 2.0)
            traci.vehicletype.setHeight(warning_type, 2.5)
            traci.vehicletype.setMinGap(warning_type, 0.5)
            traci.vehicletype.setColor(warning_type, (255, 255, 0, 255))  # Bright yellow
            
        return warning_type
        
    def _place_workers(self):
        """
        Place construction workers inside the barriers.
        """
        if not self._worker_config or not self._worker_config.get("enabled", False):
            return
            
        count = self._worker_config.get("count", 0)
        if count <= 0:
            return
            
        spacing = self._worker_config.get("spacing", 50)
        
        # Distribute workers along the corridor
        worker_count = 0
        current_distance = spacing
        
        for edge in self._corridor_edges:
            if worker_count >= count:
                break
                
            edge_length = traci.lane.getLength(f"{edge}_0")
            
            while current_distance <= edge_length and worker_count < count:
                worker_id = f"WORKER_{self._adversity_id}_{worker_count}"
                
                # Add worker as person
                traci.person.add(
                    worker_id,
                    edgeID=edge,
                    pos=current_distance
                )
                
                # Set worker appearance
                traci.person.setColor(worker_id, (255, 140, 0, 255))  # Orange safety vest
                traci.person.setWidth(worker_id, 0.5)
                traci.person.setLength(worker_id, 0.5)
                traci.person.setSpeed(worker_id, 0)  # Static worker
                
                self._worker_ids.append(worker_id)
                worker_count += 1
                current_distance += spacing
                
            current_distance -= edge_length
            
        logger.info(f"Placed {worker_count} workers in construction zone")
        
    def initialize(self, time: float):
        """
        Initialize the urban construction zone.
        
        Args:
            time: Current simulation time
        """
        if not self.is_effective():
            logger.error("Urban construction zone is not effective")
            return
            
        logger.info(f"Initializing urban construction zone across {len(self._corridor_edges)} edges")
        
        # Create construction zones along the corridor
        self._create_construction_zones()
        
        # Place construction vehicles
        self._place_construction_vehicles()
        
        # Place workers
        self._place_workers()
        
        self._is_active = True
        logger.info(f"Urban construction zone initialized with {len(self._construction_object_ids)} objects")
        
    def update(self, time: float):
        """
        Update the urban construction zone.
        
        Args:
            time: Current simulation time
        """
        if not self._is_active:
            return
            
        # Maintain static objects
        for obj_id in self._construction_object_ids:
            if obj_id in traci.vehicle.getIDList():
                try:
                    traci.vehicle.setSpeed(obj_id, 0)
                except:
                    pass
                    
        # Maintain static vehicles
        for vehicle_id in self._construction_vehicle_ids:
            if vehicle_id in traci.vehicle.getIDList():
                try:
                    traci.vehicle.setSpeed(vehicle_id, 0)
                except:
                    pass
                    
        # Check if construction should end
        if self.end_time != -1 and time >= self.end_time:
            self._cleanup()
            self._is_active = False
            
    def _cleanup(self):
        """
        Remove all construction objects and vehicles.
        """
        # Remove construction objects
        for obj_id in self._construction_object_ids:
            try:
                if obj_id in traci.vehicle.getIDList():
                    traci.vehicle.remove(obj_id)
            except:
                pass
                
        # Remove construction vehicles
        for vehicle_id in self._construction_vehicle_ids:
            try:
                if vehicle_id in traci.vehicle.getIDList():
                    traci.vehicle.remove(vehicle_id)
            except:
                pass
                
        # Remove workers
        for worker_id in self._worker_ids:
            try:
                if worker_id in traci.person.getIDList():
                    traci.person.remove(worker_id)
            except:
                pass
                
        logger.info("Urban construction zone cleaned up")