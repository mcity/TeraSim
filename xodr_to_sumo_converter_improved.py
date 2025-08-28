#!/usr/bin/env python3
"""
Improved OpenDRIVE to SUMO Converter
Focuses on geometry accuracy and proper coordinate transformation
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set
import math
import numpy as np
import os
import subprocess
import sys
import argparse
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

@dataclass
class PlainNode:
    """SUMO Plain XML node"""
    id: str
    x: float
    y: float
    type: str = "priority"
    
@dataclass  
class PlainEdge:
    """SUMO Plain XML edge"""
    id: str
    from_node: str
    to_node: str
    num_lanes: int = 1
    speed: float = 13.89  # m/s (50 km/h)
    priority: int = 1
    type: str = ""
    name: str = ""
    shape: Optional[List[Tuple[float, float]]] = None
    lane_data: Optional[List[Dict]] = None
    
@dataclass
class PlainConnection:
    """SUMO Plain XML connection"""
    from_edge: str
    to_edge: str
    from_lane: int
    to_lane: int
    dir: str = "s"
    state: str = "M"
    via: Optional[str] = None  # Via points as string "x1,y1 x2,y2 ..."

@dataclass
class OpenDriveRoad:
    """OpenDRIVE road data"""
    id: str
    name: str
    junction: str
    length: float
    lanes_left: List[Dict] = field(default_factory=list)
    lanes_right: List[Dict] = field(default_factory=list)
    geometry: List[Dict] = field(default_factory=list)
    predecessor: Optional[Dict] = None
    successor: Optional[Dict] = None
    road_type: str = "town"
    speed_limit: float = 13.89
    is_junction_internal: bool = False  # Added for junction detection
    actual_junction_id: Optional[str] = None  # Added for junction mapping

class GeometryCalculator:
    """Handles precise geometry calculations for OpenDRIVE elements"""
    
    @staticmethod
    def calculate_line_points(x0, y0, hdg, length, num_points=10):
        """Calculate points along a straight line"""
        points = []
        for i in range(num_points):
            s = (i / (num_points - 1)) * length if num_points > 1 else 0
            x = x0 + s * math.cos(hdg)
            y = y0 + s * math.sin(hdg)
            points.append((x, y))
        return points
    
    @staticmethod
    def calculate_arc_points(x0, y0, hdg, length, curvature, num_points=20):
        """Calculate points along an arc with proper curvature"""
        if abs(curvature) < 1e-6:
            return GeometryCalculator.calculate_line_points(x0, y0, hdg, length, num_points)
        
        points = []
        radius = 1.0 / curvature
        
        # Calculate center of arc
        cx = x0 - radius * math.sin(hdg)
        cy = y0 + radius * math.cos(hdg)
        
        # Initial angle
        theta0 = hdg - math.pi/2 if curvature > 0 else hdg + math.pi/2
        
        for i in range(num_points):
            s = (i / (num_points - 1)) * length if num_points > 1 else 0
            dtheta = s * curvature
            theta = theta0 + dtheta
            
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)
            points.append((x, y))
            
        return points
    
    @staticmethod
    def calculate_spiral_points(x0, y0, hdg, length, curv_start, curv_end, num_points=30):
        """Calculate points along a spiral (clothoid) with linear curvature change"""
        points = []
        
        # For Euler spiral, curvature changes linearly
        a = (curv_end - curv_start) / length if length > 0 else 0
        
        for i in range(num_points):
            s = (i / (num_points - 1)) * length if num_points > 1 else 0
            
            # Integrate to get position using Fresnel integrals approximation
            x_local = 0
            y_local = 0
            theta = hdg
            
            # Numerical integration with smaller steps for accuracy
            steps = max(10, int(s * 10))
            ds = s / steps if steps > 0 else 0
            
            for j in range(steps):
                s_j = j * ds
                curv = curv_start + a * s_j
                theta += curv * ds
                x_local += math.cos(theta) * ds
                y_local += math.sin(theta) * ds
            
            # Transform to global coordinates
            x = x0 + x_local * math.cos(hdg) - y_local * math.sin(hdg)
            y = y0 + x_local * math.sin(hdg) + y_local * math.cos(hdg)
            points.append((x, y))
            
        return points
    
    @staticmethod
    def calculate_parampoly3_points(x0, y0, hdg, length, coeffs_u, coeffs_v, p_range="normalized", num_points=20):
        """Calculate points along a parametric cubic polynomial"""
        points = []
        
        for i in range(num_points):
            if p_range == "normalized":
                p = i / (num_points - 1) if num_points > 1 else 0
            else:  # arcLength
                p = (i / (num_points - 1)) * length if num_points > 1 else 0
            
            # Calculate local coordinates using polynomial
            u = coeffs_u[0] + coeffs_u[1]*p + coeffs_u[2]*p**2 + coeffs_u[3]*p**3
            v = coeffs_v[0] + coeffs_v[1]*p + coeffs_v[2]*p**2 + coeffs_v[3]*p**3
            
            # Transform to global coordinates
            x = x0 + u * math.cos(hdg) - v * math.sin(hdg)
            y = y0 + u * math.sin(hdg) + v * math.cos(hdg)
            points.append((x, y))
            
        return points

class ImprovedOpenDriveToSumoConverter:
    """
    Improved OpenDRIVE to SUMO Converter with better geometry handling
    """
    
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.nodes: List[PlainNode] = []
        self.edges: List[PlainEdge] = []
        self.connections: List[PlainConnection] = []
        
        # Mapping tables
        self.node_map: Dict[str, str] = {}
        self.road_map: Dict[str, OpenDriveRoad] = {}
        self.junction_roads: Dict[str, List[str]] = {}
        self.junction_connections: Dict[str, List[Dict]] = {}
        self.junctions: Dict[str, Dict] = {}  # Changed to Dict for easier access
        
        # Junction internal roads mapping
        self.junction_internal_roads: Dict[str, str] = {}  # road_id -> junction_id
        self.junction_node_mapping: Dict[str, str] = {}  # junction_id -> node_id
        self.road_node_mapping: Dict[str, Dict[str, str]] = {}  # road_id -> {from: node_id, to: node_id}
        
        # Geometry calculator
        self.geom_calc = GeometryCalculator()
        
        # Node counter
        self.node_counter = 0
        
    def convert(self, xodr_file: str, output_prefix: str, use_netconvert: bool = True) -> bool:
        """
        Convert OpenDRIVE file to SUMO format
        """
        try:
            # 1. Parse OpenDRIVE file
            logger.info(f"Parsing OpenDRIVE file: {xodr_file}")
            if not self._parse_opendrive(xodr_file):
                return False
            
            # 2. Auto-detect and fix junction internal roads
            logger.info("Identifying junction internal roads...")
            self._identify_junction_internal_roads()
            
            # 3. Convert to Plain XML elements
            logger.info("Converting to Plain XML format...")
            self._create_nodes()
            self._create_edges()
            self._create_junction_connections()  # New method for junction-based connections
            
            # 4. Write Plain XML files
            logger.info(f"Writing Plain XML files with prefix: {output_prefix}")
            self._write_plain_xml(output_prefix)
            
            # 5. Use netconvert to generate final network
            if use_netconvert:
                logger.info("Running netconvert to generate final network...")
                result = self._run_netconvert(output_prefix)
                
                # 6. Validate conversion
                self._validate_conversion()
                
                return result
            
            # 6. Validate conversion
            self._validate_conversion()
            
            return True
            
        except Exception as e:
            logger.error(f"Conversion failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _parse_opendrive(self, xodr_file: str) -> bool:
        """Parse OpenDRIVE file with improved geometry handling"""
        try:
            tree = ET.parse(xodr_file)
            root = tree.getroot()
            
            # Parse all roads
            roads = root.findall('.//road')
            logger.info(f"Found {len(roads)} roads")
            
            for road_elem in roads:
                road = self._parse_road(road_elem)
                self.road_map[road.id] = road
                
                # Record junction internal roads
                if road.junction != '-1':
                    if road.junction not in self.junction_roads:
                        self.junction_roads[road.junction] = []
                    self.junction_roads[road.junction].append(road.id)
            
            # Parse all junctions
            junctions = root.findall('.//junction')
            logger.info(f"Found {len(junctions)} junctions")
            
            for junction_elem in junctions:
                junction_id = junction_elem.get('id')
                junction_data = {
                    'id': junction_id,
                    'connections': []
                }
                self.junction_connections[junction_id] = []
                
                # Parse connections within junction
                for conn_elem in junction_elem.findall('.//connection'):
                    connection = {
                        'id': conn_elem.get('id'),
                        'incomingRoad': conn_elem.get('incomingRoad'),
                        'connectingRoad': conn_elem.get('connectingRoad'),
                        'contactPoint': conn_elem.get('contactPoint'),
                        'laneLinks': []
                    }
                    
                    # Parse lane links
                    for lane_link in conn_elem.findall('.//laneLink'):
                        connection['laneLinks'].append({
                            'from': int(lane_link.get('from')),
                            'to': int(lane_link.get('to'))
                        })
                    
                    self.junction_connections[junction_id].append(connection)
                    junction_data['connections'].append(connection)
                
                self.junctions[junction_id] = junction_data
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to parse OpenDRIVE: {e}")
            return False
    
    def _identify_junction_internal_roads(self):
        """Auto-detect and mark all junction internal roads, ignoring incorrect junction attributes"""
        logger.info("Auto-detecting junction internal roads...")
        
        # Clear any existing mapping
        self.junction_internal_roads = {}
        
        # Iterate through all junctions
        for junction_id, junction_data in self.junctions.items():
            connecting_roads = set()
            
            # Extract all connecting roads from connections
            for connection in junction_data.get('connections', []):
                connecting_road_id = connection.get('connectingRoad')
                if connecting_road_id:
                    connecting_roads.add(connecting_road_id)
                    self.junction_internal_roads[connecting_road_id] = junction_id
                    
                    # Mark the road as junction internal
                    if connecting_road_id in self.road_map:
                        road = self.road_map[connecting_road_id]
                        # Check if junction attribute is incorrect
                        if road.junction == '-1':
                            logger.warning(
                                f"Road {connecting_road_id} is used as connecting road in junction {junction_id}, "
                                f"but has junction='-1'. Auto-correcting."
                            )
                        # Mark as internal regardless of original junction attribute
                        road.is_junction_internal = True
                        road.actual_junction_id = junction_id
            
            logger.info(f"Junction {junction_id} contains connecting roads: {connecting_roads}")
        
        logger.info(f"Identified {len(self.junction_internal_roads)} junction internal roads")
        return self.junction_internal_roads
    
    def _create_junction_nodes(self):
        """Create single unified node for each junction"""
        logger.info("Creating unified junction nodes...")
        
        for junction_id, junction_data in self.junctions.items():
            # Collect all points related to this junction
            points = []
            
            # Get endpoints from incoming roads
            for connection in junction_data.get('connections', []):
                incoming_road_id = connection.get('incomingRoad')
                contact_point = connection.get('contactPoint', 'end')
                
                if incoming_road_id in self.road_map:
                    road = self.road_map[incoming_road_id]
                    
                    # Get the appropriate endpoint
                    if contact_point == 'start':
                        if road.geometry and len(road.geometry) > 0:
                            geom = road.geometry[0]
                            point = (geom['x'], geom['y'])
                            points.append(point)
                    else:  # end
                        if road.geometry and len(road.geometry) > 0:
                            # Get end point of last geometry segment
                            last_geom = road.geometry[-1]
                            end_points = self._get_geometry_end_point(last_geom)
                            if end_points:
                                points.append(end_points)
            
            # Also collect points from connecting roads for better center calculation
            for road_id in self.junction_internal_roads:
                if self.junction_internal_roads[road_id] == junction_id:
                    road = self.road_map.get(road_id)
                    if road and road.geometry:
                        # Add midpoint of connecting road
                        first_geom = road.geometry[0]
                        last_geom = road.geometry[-1]
                        
                        start_pt = (first_geom['x'], first_geom['y'])
                        end_pt = self._get_geometry_end_point(last_geom)
                        
                        if end_pt:
                            mid_x = (start_pt[0] + end_pt[0]) / 2
                            mid_y = (start_pt[1] + end_pt[1]) / 2
                            points.append((mid_x, mid_y))
            
            # Calculate junction center
            if points:
                center_x = sum(p[0] for p in points) / len(points)
                center_y = sum(p[1] for p in points) / len(points)
            else:
                logger.warning(f"Junction {junction_id} has no valid geometry points")
                center_x, center_y = 0, 0
            
            # Create junction node
            node_id = f"junction_{junction_id}"
            node = PlainNode(
                id=node_id,
                x=center_x,
                y=center_y,
                type='priority'  # Could be enhanced based on junction type
            )
            
            self.nodes.append(node)
            self.junction_node_mapping[junction_id] = node_id
            self.node_map[node_id] = node_id
            
            logger.info(f"Created junction node: {node_id} at ({center_x:.2f}, {center_y:.2f})")
        
        return self.junction_node_mapping
    
    def _get_geometry_end_point(self, geom: Dict) -> Optional[Tuple[float, float]]:
        """Get the end point of a geometry segment"""
        geom_type = geom.get('type', 'line')
        
        if geom_type == 'line':
            x0, y0 = geom['x'], geom['y']
            hdg = geom['hdg']
            length = geom['length']
            x = x0 + length * math.cos(hdg)
            y = y0 + length * math.sin(hdg)
            return (x, y)
        elif geom_type == 'arc':
            points = self.geom_calc.calculate_arc_points(
                geom['x'], geom['y'], geom['hdg'], 
                geom['length'], geom['curvature'], 
                num_points=2
            )
            return points[-1] if points else None
        # Add other geometry types as needed
        
        return None
    
    def _parse_road(self, road_elem: ET.Element) -> OpenDriveRoad:
        """Parse single road with improved geometry parsing"""
        road = OpenDriveRoad(
            id=road_elem.get('id'),
            name=road_elem.get('name', ''),
            junction=road_elem.get('junction', '-1'),
            length=float(road_elem.get('length', 0))
        )
        
        # Parse geometry information with support for all types
        plan_view = road_elem.find('.//planView')
        if plan_view is not None:
            for geom_elem in plan_view.findall('.//geometry'):
                geom = {
                    's': float(geom_elem.get('s', 0)),
                    'x': float(geom_elem.get('x', 0)),
                    'y': float(geom_elem.get('y', 0)),
                    'hdg': float(geom_elem.get('hdg', 0)),
                    'length': float(geom_elem.get('length', 0))
                }
                
                # Determine geometry type and parse specific parameters
                if geom_elem.find('.//line') is not None:
                    geom['type'] = 'line'
                elif geom_elem.find('.//arc') is not None:
                    geom['type'] = 'arc'
                    arc = geom_elem.find('.//arc')
                    geom['curvature'] = float(arc.get('curvature', 0))
                elif geom_elem.find('.//spiral') is not None:
                    geom['type'] = 'spiral'
                    spiral = geom_elem.find('.//spiral')
                    geom['curvStart'] = float(spiral.get('curvStart', 0))
                    geom['curvEnd'] = float(spiral.get('curvEnd', 0))
                elif geom_elem.find('.//paramPoly3') is not None:
                    geom['type'] = 'paramPoly3'
                    poly = geom_elem.find('.//paramPoly3')
                    geom['aU'] = float(poly.get('aU', 0))
                    geom['bU'] = float(poly.get('bU', 0))
                    geom['cU'] = float(poly.get('cU', 0))
                    geom['dU'] = float(poly.get('dU', 0))
                    geom['aV'] = float(poly.get('aV', 0))
                    geom['bV'] = float(poly.get('bV', 0))
                    geom['cV'] = float(poly.get('cV', 0))
                    geom['dV'] = float(poly.get('dV', 0))
                    geom['pRange'] = poly.get('pRange', 'normalized')
                else:
                    geom['type'] = 'line'  # Default to line
                
                road.geometry.append(geom)
        
        # Parse road type and speed limit
        type_elem = road_elem.find('.//type')
        if type_elem is not None:
            road.road_type = type_elem.get('type', 'town')
            
            # Parse speed limit if available
            speed_elem = type_elem.find('.//speed')
            if speed_elem is not None:
                max_speed = speed_elem.get('max')
                if max_speed:
                    speed_unit = speed_elem.get('unit', 'ms')
                    if speed_unit == 'ms' or speed_unit == 'm/s':
                        road.speed_limit = float(max_speed)
                    elif speed_unit == 'kmh' or speed_unit == 'km/h':
                        road.speed_limit = float(max_speed) / 3.6
                    elif speed_unit == 'mph':
                        road.speed_limit = float(max_speed) * 0.44704
        
        # Parse lanes
        lanes_elem = road_elem.find('.//lanes')
        if lanes_elem is not None:
            lane_section = lanes_elem.find('.//laneSection')
            if lane_section is not None:
                # Left lanes
                left = lane_section.find('.//left')
                if left is not None:
                    for lane in left.findall('.//lane'):
                        lane_type = lane.get('type')
                        if lane_type in ['driving', 'entry', 'exit', 'onRamp', 'offRamp', 'shoulder']:
                            road.lanes_left.append({
                                'id': int(lane.get('id')),
                                'type': lane_type,
                                'width': self._get_lane_width(lane)
                            })
                
                # Right lanes
                right = lane_section.find('.//right')
                if right is not None:
                    for lane in right.findall('.//lane'):
                        lane_type = lane.get('type')
                        if lane_type in ['driving', 'entry', 'exit', 'onRamp', 'offRamp', 'shoulder']:
                            road.lanes_right.append({
                                'id': int(lane.get('id')),
                                'type': lane_type,
                                'width': self._get_lane_width(lane)
                            })
        
        # Parse connection relationships
        link = road_elem.find('.//link')
        if link is not None:
            pred = link.find('.//predecessor')
            if pred is not None:
                road.predecessor = {
                    'elementId': pred.get('elementId'),
                    'elementType': pred.get('elementType'),
                    'contactPoint': pred.get('contactPoint')
                }
            
            succ = link.find('.//successor')
            if succ is not None:
                road.successor = {
                    'elementId': succ.get('elementId'),
                    'elementType': succ.get('elementType'),
                    'contactPoint': succ.get('contactPoint')
                }
        
        return road
    
    def _get_lane_width(self, lane_elem: ET.Element) -> float:
        """Get lane width from lane element, preferring non-zero width definitions"""
        width_elems = lane_elem.findall('.//width')
        if width_elems:
            # First try to find a width element with non-zero 'a' coefficient
            for width_elem in width_elems:
                a = float(width_elem.get('a', 0))
                if abs(a) > 0.01:  # Consider widths greater than 1cm as valid
                    return abs(a)
            
            # If all widths are near zero, take the last one (fallback)
            last_width = width_elems[-1]
            a = float(last_width.get('a', 3.5))
            return abs(a)  # Use absolute value
        return 3.5  # Default width
    
    def _calculate_road_shape(self, road: OpenDriveRoad) -> List[Tuple[float, float]]:
        """Calculate precise road shape points from geometry"""
        all_points = []
        
        for geom in road.geometry:
            if geom['type'] == 'line':
                points = self.geom_calc.calculate_line_points(
                    geom['x'], geom['y'], geom['hdg'], geom['length']
                )
            elif geom['type'] == 'arc':
                points = self.geom_calc.calculate_arc_points(
                    geom['x'], geom['y'], geom['hdg'], geom['length'], 
                    geom.get('curvature', 0)
                )
            elif geom['type'] == 'spiral':
                points = self.geom_calc.calculate_spiral_points(
                    geom['x'], geom['y'], geom['hdg'], geom['length'],
                    geom.get('curvStart', 0), geom.get('curvEnd', 0)
                )
            elif geom['type'] == 'paramPoly3':
                coeffs_u = [geom.get('aU', 0), geom.get('bU', 0), 
                           geom.get('cU', 0), geom.get('dU', 0)]
                coeffs_v = [geom.get('aV', 0), geom.get('bV', 0),
                           geom.get('cV', 0), geom.get('dV', 0)]
                points = self.geom_calc.calculate_parampoly3_points(
                    geom['x'], geom['y'], geom['hdg'], geom['length'],
                    coeffs_u, coeffs_v, geom.get('pRange', 'normalized')
                )
            else:
                # Default to line
                points = self.geom_calc.calculate_line_points(
                    geom['x'], geom['y'], geom['hdg'], geom['length']
                )
            
            # Avoid duplicate points at segment boundaries
            if all_points and len(points) > 0:
                if math.dist(all_points[-1], points[0]) < 0.01:
                    points = points[1:]
            
            all_points.extend(points)
        
        return all_points
    
    def _get_road_start_end(self, road: OpenDriveRoad) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Get precise start and end positions of a road"""
        if not road.geometry:
            return ((0, 0), (0, 0))
        
        # Start position
        first_geom = road.geometry[0]
        start_x = first_geom['x']
        start_y = first_geom['y']
        
        # Calculate end position by traversing all geometry
        shape = self._calculate_road_shape(road)
        if shape:
            end_x, end_y = shape[-1]
        else:
            # Fallback calculation
            end_x = start_x
            end_y = start_y
            for geom in road.geometry:
                if geom['type'] == 'line':
                    end_x = geom['x'] + geom['length'] * math.cos(geom['hdg'])
                    end_y = geom['y'] + geom['length'] * math.sin(geom['hdg'])
        
        return ((start_x, start_y), (end_x, end_y))
    
    def _create_node(self, x: float, y: float, node_type: str = "priority") -> str:
        """Create or find a node at given position"""
        # Check if node already exists at this position (with tolerance)
        tolerance = 0.1
        for node in self.nodes:
            if abs(node.x - x) < tolerance and abs(node.y - y) < tolerance:
                return node.id
        
        # Create new node
        node_id = f"n{self.node_counter}"
        self.node_counter += 1
        self.nodes.append(PlainNode(node_id, x, y, node_type))
        return node_id
    
    def _create_nodes(self):
        """Create nodes for road endpoints and junctions"""
        logger.info("Creating nodes...")
        
        # First, create unified junction nodes
        self._create_junction_nodes()
        
        # Then create nodes for regular roads and long connecting roads
        for road_id, road in self.road_map.items():
            # For junction internal roads, check length
            if road_id in self.junction_internal_roads:
                # Only skip short internal roads (< 10m)
                if road.length < 10.0:
                    logger.debug(f"Skipping node creation for short junction internal road {road_id} (length: {road.length:.2f}m)")
                    continue
                else:
                    logger.info(f"Creating nodes for long connecting road {road_id} (length: {road.length:.2f}m)")
            
            (start_x, start_y), (end_x, end_y) = self._get_road_start_end(road)
            
            # For connecting roads that belong to a junction, use junction nodes
            if road_id in self.junction_internal_roads:
                # This is a long connecting road that needs junction connections
                junction_id = self.junction_internal_roads[road_id]
                
                # Determine which end connects to the junction based on connections
                # Usually, connecting roads have one end at junction, other end to regular road
                from_node_id = None
                to_node_id = None
                
                # Check predecessor and successor to determine connection pattern
                if road.predecessor and road.predecessor['elementType'] == 'junction':
                    from_node_id = self.junction_node_mapping.get(road.predecessor['elementId'])
                elif road.predecessor and road.predecessor['elementType'] == 'road':
                    # Connect to the road's endpoint
                    pred_road_id = road.predecessor['elementId']
                    pred_contact = road.predecessor.get('contactPoint', 'end')
                    if pred_contact == 'start':
                        from_node_id = self.node_map.get(f"{pred_road_id}_start")
                    else:
                        from_node_id = self.node_map.get(f"{pred_road_id}_end")
                
                if not from_node_id:
                    from_node_id = self._create_node(start_x, start_y)
                
                if road.successor and road.successor['elementType'] == 'junction':
                    to_node_id = self.junction_node_mapping.get(road.successor['elementId'])
                elif road.successor and road.successor['elementType'] == 'road':
                    # Connect to the road's endpoint
                    succ_road_id = road.successor['elementId']
                    succ_contact = road.successor.get('contactPoint', 'start')
                    if succ_contact == 'start':
                        to_node_id = self.node_map.get(f"{succ_road_id}_start")
                    else:
                        to_node_id = self.node_map.get(f"{succ_road_id}_end")
                
                if not to_node_id:
                    to_node_id = self._create_node(end_x, end_y)
            else:
                # Regular road logic
                # Check if road connects to junction at start
                from_node_id = None
                if road.predecessor and road.predecessor['elementType'] == 'junction':
                    junction_id = road.predecessor['elementId']
                    from_node_id = self.junction_node_mapping.get(junction_id)
                    logger.debug(f"Road {road_id} connects to junction {junction_id} at start")
                
                # Create start node if not connected to junction
                if not from_node_id:
                    from_node_id = self._create_node(start_x, start_y)
                
                # Check if road connects to junction at end
                to_node_id = None
                if road.successor and road.successor['elementType'] == 'junction':
                    junction_id = road.successor['elementId']
                    to_node_id = self.junction_node_mapping.get(junction_id)
                    logger.debug(f"Road {road_id} connects to junction {junction_id} at end")
                
                # Create end node if not connected to junction
                if not to_node_id:
                    to_node_id = self._create_node(end_x, end_y)
            
            # Store mapping for this road
            self.road_node_mapping[road_id] = {
                'from': from_node_id,
                'to': to_node_id
            }
            self.node_map[f"{road_id}_start"] = from_node_id
            self.node_map[f"{road_id}_end"] = to_node_id
        
        logger.info(f"Created {len(self.nodes)} nodes total ({len(self.junction_node_mapping)} junction nodes)")
    
    def _create_edges(self):
        """Create edges from roads with precise geometry"""
        logger.info("Creating edges...")
        
        for road_id, road in self.road_map.items():
            # For junction internal roads, check if they're long enough to be real roads
            if road_id in self.junction_internal_roads:
                # Only skip very short internal roads (< 10m)
                # Long connecting roads should be preserved as edges
                if road.length < 10.0:
                    logger.debug(f"Skipping edge creation for short junction internal road {road_id} (length: {road.length:.2f}m)")
                    continue
                else:
                    logger.info(f"Creating edge for long connecting road {road_id} (length: {road.length:.2f}m)")
            
            # Get nodes from road_node_mapping
            node_mapping = self.road_node_mapping.get(road_id)
            if not node_mapping:
                # Fallback to old method
                from_node = self.node_map.get(f"{road_id}_start")
                to_node = self.node_map.get(f"{road_id}_end")
            else:
                from_node = node_mapping['from']
                to_node = node_mapping['to']
                
                if not from_node or not to_node:
                    logger.warning(f"Missing nodes for road {road_id}")
                    continue
                
                # Calculate number of lanes
                num_lanes = max(len(road.lanes_right), len(road.lanes_left), 1)
                
                # Calculate road shape
                shape = self._calculate_road_shape(road)
                
                # Collect lane data 
                # OpenDRIVE right lanes: -1 (innermost/closest to centerline) to -N (outermost)
                # SUMO lanes: 0 (rightmost/outermost) to N-1 (leftmost/innermost/closest to centerline)
                # Example mapping:
                #   OpenDRIVE -4 (outermost) -> SUMO lane 0 (rightmost)
                #   OpenDRIVE -3 -> SUMO lane 1
                #   OpenDRIVE -2 -> SUMO lane 2
                #   OpenDRIVE -1 (innermost) -> SUMO lane 3 (leftmost)
                lane_data = []
                # Sort lanes by ID in ascending order (most negative first: -4, -3, -2, -1)
                sorted_lanes = sorted(road.lanes_right, key=lambda x: x['id'])
                for lane in sorted_lanes:
                    lane_data.append({
                        'id': lane['id'],
                        'type': lane['type'],
                        'width': lane['width']
                    })
                
                # Create edge
                edge = PlainEdge(
                    id=road_id,
                    from_node=from_node,
                    to_node=to_node,
                    num_lanes=num_lanes,
                    speed=road.speed_limit,
                    priority=1,
                    name=road.name,
                    shape=shape if len(shape) > 2 else None,
                    lane_data=lane_data if lane_data else None
                )
                
                self.edges.append(edge)
        
        logger.info(f"Created {len(self.edges)} edges")
    
    def _get_lane_type(self, road_data: Optional[OpenDriveRoad], lane_id: int) -> str:
        """Get the type of a lane by its ID"""
        if not road_data:
            return "unknown"
        
        # Check right lanes (negative IDs)
        if lane_id < 0:
            for lane in road_data.lanes_right:
                if lane['id'] == lane_id:
                    return lane['type']
        # Check left lanes (positive IDs)
        elif lane_id > 0:
            for lane in road_data.lanes_left:
                if lane['id'] == lane_id:
                    return lane['type']
        
        return "unknown"
    
    def _convert_to_sumo_lane_index(self, road_data: Optional[OpenDriveRoad], lane_id: int) -> int:
        """Convert OpenDRIVE lane ID to SUMO lane index considering lane order"""
        if not road_data or lane_id == 0:
            return 0
        
        # For right lanes (negative IDs)
        if lane_id < 0:
            # Sort lanes by ID (most negative first)
            sorted_lanes = sorted(road_data.lanes_right, key=lambda x: x['id'])
            # Find the position of this lane
            for idx, lane in enumerate(sorted_lanes):
                if lane['id'] == lane_id:
                    return idx
        
        return 0
    
    def _extract_via_points_from_road(self, road: OpenDriveRoad, num_points: int = 5) -> List[Tuple[float, float]]:
        """Extract via points from a connecting road's geometry"""
        via_points = []
        
        if not road.geometry:
            return via_points
        
        # Sample points along the road geometry
        for geom in road.geometry:
            geom_type = geom.get('type', 'line')
            
            if geom_type == 'line':
                points = self.geom_calc.calculate_line_points(
                    geom['x'], geom['y'], geom['hdg'], 
                    geom['length'], num_points=num_points
                )
                via_points.extend(points)
            elif geom_type == 'arc':
                points = self.geom_calc.calculate_arc_points(
                    geom['x'], geom['y'], geom['hdg'],
                    geom['length'], geom['curvature'],
                    num_points=num_points
                )
                via_points.extend(points)
            # Add other geometry types as needed
        
        return via_points
    
    def _find_outgoing_road(self, connecting_road: OpenDriveRoad) -> Optional[str]:
        """Find the outgoing road from a connecting road"""
        # Check successor
        if connecting_road.successor and connecting_road.successor['elementType'] == 'road':
            return connecting_road.successor['elementId']
        
        # Check predecessor if needed (for reversed connections)
        if connecting_road.predecessor and connecting_road.predecessor['elementType'] == 'road':
            # Need to check if this is actually the outgoing road
            # This depends on the connection direction
            return connecting_road.predecessor['elementId']
        
        return None
    
    def _create_junction_connections(self):
        """Create connections based on junction definitions using connecting road geometry as via paths"""
        logger.info("Creating junction-based connections...")
        
        connections_created = 0
        shoulder_connections_skipped = 0
        
        for junction_id, junction_data in self.junctions.items():
            junction_node = self.junction_node_mapping.get(junction_id)
            
            for connection in junction_data.get('connections', []):
                incoming_road_id = connection.get('incomingRoad')
                connecting_road_id = connection.get('connectingRoad')
                contact_point = connection.get('contactPoint', 'end')
                
                # Get the roads
                incoming_road = self.road_map.get(incoming_road_id)
                connecting_road = self.road_map.get(connecting_road_id)
                
                if not incoming_road or not connecting_road:
                    logger.warning(f"Missing road data for connection in junction {junction_id}")
                    continue
                
                # Check if connecting road is long enough to be an actual edge
                if connecting_road.length >= 10.0:
                    # For long connecting roads, create simple connection to the connecting road
                    # The connecting road itself will handle further connections
                    outgoing_road_id = connecting_road_id
                    logger.debug(f"Using long connecting road {connecting_road_id} as target")
                else:
                    # Find the outgoing road for short internal roads
                    outgoing_road_id = self._find_outgoing_road(connecting_road)
                    if not outgoing_road_id:
                        logger.warning(f"Cannot determine outgoing road for connecting road {connecting_road_id}")
                        continue
                
                # Skip if outgoing road is the same as incoming (U-turn)
                if outgoing_road_id == incoming_road_id:
                    continue
                
                outgoing_road = self.road_map.get(outgoing_road_id)
                if not outgoing_road:
                    continue
                
                # Extract via points only for short connecting roads
                via_string = None
                if connecting_road.length < 10.0:
                    via_points = self._extract_via_points_from_road(connecting_road, num_points=3)
                    via_string = " ".join(f"{p[0]:.2f},{p[1]:.2f}" for p in via_points) if via_points else None
                
                # Create connection for each lane link
                for lane_link in connection.get('laneLinks', []):
                    from_lane_id = lane_link.get('from')
                    to_lane_id = lane_link.get('to')
                    
                    # Check if either lane is a shoulder
                    from_lane_type = self._get_lane_type(incoming_road, from_lane_id)
                    to_lane_type = self._get_lane_type(outgoing_road, to_lane_id)
                    
                    if from_lane_type == 'shoulder' or to_lane_type == 'shoulder':
                        shoulder_connections_skipped += 1
                        logger.debug(f"Skipping shoulder connection in junction {junction_id}")
                        continue
                    
                    # Convert to SUMO lane indices
                    from_lane_idx = self._convert_to_sumo_lane_index(incoming_road, from_lane_id)
                    to_lane_idx = self._convert_to_sumo_lane_index(outgoing_road, to_lane_id)
                    
                    # Create connection
                    conn = PlainConnection(
                        from_edge=incoming_road_id,
                        to_edge=outgoing_road_id,
                        from_lane=from_lane_idx,
                        to_lane=to_lane_idx,
                        via=via_string if via_string else None
                    )
                    
                    self.connections.append(conn)
                    connections_created += 1
                    
                    logger.debug(
                        f"Created connection: {incoming_road_id}[{from_lane_idx}] -> "
                        f"{outgoing_road_id}[{to_lane_idx}] via junction {junction_id}"
                    )
        
        logger.info(f"Created {connections_created} connections via junction definitions")
        if shoulder_connections_skipped > 0:
            logger.info(f"Skipped {shoulder_connections_skipped} shoulder lane connections")
    
    def _create_connections(self):
        """Create connections for junctions, excluding shoulder lanes"""
        logger.info("Creating connections...")
        
        shoulder_connections_skipped = 0
        
        for junction_id, connections in self.junction_connections.items():
            for conn in connections:
                incoming_road = conn['incomingRoad']
                connecting_road = conn['connectingRoad']
                
                # Get road data for lane type checking
                incoming_road_data = self.road_map.get(incoming_road)
                connecting_road_data = self.road_map.get(connecting_road)
                if not connecting_road_data:
                    continue
                
                # Find the target road based on successor/predecessor
                target_road = None
                target_road_data = None
                if connecting_road_data.successor and \
                   connecting_road_data.successor['elementType'] == 'road':
                    target_road = connecting_road_data.successor['elementId']
                    target_road_data = self.road_map.get(target_road)
                elif connecting_road_data.predecessor and \
                     connecting_road_data.predecessor['elementType'] == 'road':
                    target_road = connecting_road_data.predecessor['elementId']
                    target_road_data = self.road_map.get(target_road)
                
                if not target_road or target_road == incoming_road:
                    continue
                
                # Create connections for each lane link, excluding shoulders
                for lane_link in conn['laneLinks']:
                    from_lane_id = lane_link['from']
                    to_lane_id = lane_link['to']
                    
                    # Check if either lane is a shoulder
                    from_lane_type = self._get_lane_type(incoming_road_data, from_lane_id)
                    to_lane_type = self._get_lane_type(target_road_data, to_lane_id)
                    
                    # Skip connection if either lane is a shoulder
                    if from_lane_type == 'shoulder' or to_lane_type == 'shoulder':
                        shoulder_connections_skipped += 1
                        logger.debug(f"Skipping shoulder connection: {incoming_road}:{from_lane_id} -> {target_road}:{to_lane_id}")
                        continue
                    
                    # Convert to SUMO lane indices using proper mapping
                    from_lane_idx = self._convert_to_sumo_lane_index(incoming_road_data, from_lane_id)
                    to_lane_idx = self._convert_to_sumo_lane_index(target_road_data, to_lane_id)
                    
                    connection = PlainConnection(
                        from_edge=incoming_road,
                        to_edge=target_road,
                        from_lane=from_lane_idx,
                        to_lane=to_lane_idx
                    )
                    
                    self.connections.append(connection)
        
        logger.info(f"Created {len(self.connections)} connections")
        if shoulder_connections_skipped > 0:
            logger.info(f"Skipped {shoulder_connections_skipped} shoulder lane connections")
    
    def _write_plain_xml(self, prefix: str):
        """Write Plain XML files"""
        # Write nodes file
        self._write_nodes_file(f"{prefix}.nod.xml")
        
        # Write edges file
        self._write_edges_file(f"{prefix}.edg.xml")
        
        # Write connections file
        self._write_connections_file(f"{prefix}.con.xml")
    
    def _write_nodes_file(self, filename: str):
        """Write nodes to Plain XML file"""
        root = ET.Element("nodes")
        
        for node in self.nodes:
            node_elem = ET.SubElement(root, "node")
            node_elem.set("id", node.id)
            node_elem.set("x", f"{node.x:.2f}")
            node_elem.set("y", f"{node.y:.2f}")
            node_elem.set("type", node.type)
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="    ")
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Wrote {filename}")
    
    def _write_edges_file(self, filename: str):
        """Write edges to Plain XML file with lane-specific attributes"""
        root = ET.Element("edges")
        
        for edge in self.edges:
            edge_elem = ET.SubElement(root, "edge")
            edge_elem.set("id", edge.id)
            edge_elem.set("from", edge.from_node)
            edge_elem.set("to", edge.to_node)
            edge_elem.set("numLanes", str(edge.num_lanes))
            edge_elem.set("speed", f"{edge.speed:.2f}")
            
            if edge.name:
                edge_elem.set("name", edge.name)
            
            # Add shape if available
            if edge.shape and len(edge.shape) > 2:
                shape_str = " ".join([f"{x:.2f},{y:.2f}" for x, y in edge.shape])
                edge_elem.set("shape", shape_str)
            
            # Add lane-specific data if available
            if hasattr(edge, 'lane_data') and edge.lane_data:
                for i, lane_info in enumerate(edge.lane_data):
                    lane_elem = ET.SubElement(edge_elem, "lane")
                    lane_elem.set("index", str(i))
                    
                    # Set lane type and permissions based on OpenDRIVE lane type
                    lane_type = lane_info.get('type')
                    
                    # Set width for all lanes
                    lane_elem.set("width", f"{lane_info.get('width', 3.2):.2f}")
                    
                    if lane_type == 'shoulder':
                        # Shoulder lanes in SUMO
                        lane_elem.set("type", "shoulder")
                        # Shoulders typically allow emergency vehicles only
                        lane_elem.set("allow", "emergency authority army vip")
                    elif lane_type in ['driving', 'entry', 'exit', 'onRamp', 'offRamp']:
                        # Regular driving lanes - no special type needed
                        # Default allow is "all" so we don't need to set it explicitly
                        pass
                    elif lane_type == 'sidewalk':
                        # Sidewalk lanes
                        lane_elem.set("type", "sidewalk")
                        lane_elem.set("allow", "pedestrian")
                    elif lane_type == 'biking':
                        # Bike lanes
                        lane_elem.set("allow", "bicycle")
                    
                    # Set speed if different from edge default
                    if 'speed' in lane_info:
                        lane_elem.set("speed", f"{lane_info['speed']:.2f}")
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="    ")
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Wrote {filename}")
    
    def _write_connections_file(self, filename: str):
        """Write connections to Plain XML file"""
        root = ET.Element("connections")
        
        for conn in self.connections:
            conn_elem = ET.SubElement(root, "connection")
            conn_elem.set("from", conn.from_edge)
            conn_elem.set("to", conn.to_edge)
            conn_elem.set("fromLane", str(conn.from_lane))
            conn_elem.set("toLane", str(conn.to_lane))
            
            # Add via attribute if present
            if conn.via:
                conn_elem.set("via", conn.via)
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space="    ")
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Wrote {filename}")
    
    def _run_netconvert(self, prefix: str) -> bool:
        """Run netconvert to generate final network"""
        try:
            # Check if SUMO is installed - try multiple paths
            import shutil
            netconvert_path = shutil.which('netconvert')
            if not netconvert_path:
                sumo_home = os.environ.get('SUMO_HOME')
                if sumo_home:
                    netconvert_path = os.path.join(sumo_home, 'bin', 'netconvert')
                else:
                    netconvert_path = 'netconvert'
            
            # Run netconvert
            cmd = [
                netconvert_path,
                '-n', f'{prefix}.nod.xml',
                '-e', f'{prefix}.edg.xml',
                '-x', f'{prefix}.con.xml',
                '-o', f'{prefix}.net.xml',
                '--geometry.remove', 'false',
                '--junctions.join', 'true',
                '--junctions.join-dist', '10',
                '--junctions.corner-detail', '5',
                '--rectangular-lane-cut', 'false',
                '--walkingareas', 'false',
                '--no-turnarounds', 'false',
                '--offset.disable-normalization', 'true',
                '--lefthand', 'false',
                '--geometry.min-radius.fix', 'true',
                '--check-lane-foes.all', 'true'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info(f"Successfully generated {prefix}.net.xml")
                return True
            else:
                logger.error(f"netconvert failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to run netconvert: {e}")
            return False
    
    def _validate_conversion(self):
        """Validate the conversion results"""
        logger.info("Validating conversion...")
        
        validation_results = {
            "Junction node count matches": len(self.junction_node_mapping) == len(self.junctions),
            "No junction internal edges": all(
                edge.id not in self.junction_internal_roads 
                for edge in self.edges
            ),
            "Connections created": len(self.connections) > 0,
            "All junctions have nodes": all(
                junction_id in self.junction_node_mapping 
                for junction_id in self.junctions.keys()
            ),
            "Junction internal roads identified": len(self.junction_internal_roads) > 0
        }
        
        # Report results
        for check, result in validation_results.items():
            status = "✓" if result else "✗"
            logger.info(f"  {status} {check}")
        
        # Summary statistics
        logger.info(f"\nConversion Statistics:")
        logger.info(f"  - Total roads: {len(self.road_map)}")
        logger.info(f"  - Junction internal roads: {len(self.junction_internal_roads)}")
        logger.info(f"  - Regular edges created: {len(self.edges)}")
        logger.info(f"  - Total nodes: {len(self.nodes)}")
        logger.info(f"  - Junction nodes: {len(self.junction_node_mapping)}")
        logger.info(f"  - Connections: {len(self.connections)}")
        
        return all(validation_results.values())

def main():
    parser = argparse.ArgumentParser(description='Convert OpenDRIVE to SUMO network')
    parser.add_argument('xodr_file', help='Input OpenDRIVE file')
    parser.add_argument('-o', '--output', default='converted', 
                       help='Output file prefix (default: converted)')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Enable verbose output')
    parser.add_argument('--no-netconvert', action='store_true',
                       help='Skip netconvert step')
    
    args = parser.parse_args()
    
    # Create converter
    converter = ImprovedOpenDriveToSumoConverter(verbose=args.verbose)
    
    # Convert
    success = converter.convert(
        args.xodr_file, 
        args.output, 
        use_netconvert=not args.no_netconvert
    )
    
    if success:
        logger.info("Conversion completed successfully")
        return 0
    else:
        logger.error("Conversion failed")
        return 1

if __name__ == "__main__":
    sys.exit(main())