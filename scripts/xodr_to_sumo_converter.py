#!/usr/bin/env python3
"""
OpenDRIVE to SUMO Converter
Using SUMO's officially recommended Plain XML intermediate format

Author: TeraSim Team
License: MIT
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math
import os
import subprocess
import sys
import argparse
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Try to import pyOpenDRIVE
try:
    from pyOpenDRIVE.OpenDriveMap import PyOpenDriveMap
    from pyOpenDRIVE.Road import PyRoad
    from pyOpenDRIVE.Junction import PyJunction
    from pyOpenDRIVE.Lane import PyLane
    from pyOpenDRIVE.LaneSection import PyLaneSection
    PYOPENDRIVE_AVAILABLE = True
    logger.info("pyOpenDRIVE is available - enhanced geometry processing enabled")
except ImportError as e:
    PYOPENDRIVE_AVAILABLE = False
    logger.warning(f"pyOpenDRIVE not available: {e}. Falling back to XML parsing.")
    PyOpenDriveMap = None
    PyRoad = None
    PyJunction = None
    PyLane = None
    PyLaneSection = None

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
    shape: Optional[List[Tuple[float, float]]] = None  # Edge shape points
    lane_data: Optional[List[Dict]] = None  # List of lane data with restrictions
    
@dataclass
class PlainConnection:
    """SUMO Plain XML connection"""
    from_edge: str
    to_edge: str
    from_lane: int
    to_lane: int
    dir: str = "s"  # s=straight, r=right, l=left, t=turn(u-turn)
    state: str = "M"  # M=major, m=minor, =equal, s=stop, w=allway_stop, y=yield, o=dead_end
    via: Optional[List[Tuple[float, float]]] = None  # Via points for preserving geometry

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
    road_type: str = "town"  # town, rural, motorway, etc.
    speed_limit: float = 13.89  # m/s (default 50 km/h)

class OpenDriveToSumoConverter:
    """
    OpenDRIVE to SUMO Converter
    Using Plain XML intermediate format, conforming to SUMO's official recommendations
    """
    
    # OpenDRIVE RoadLink Type enumeration values (from libOpenDRIVE)
    ROADLINK_TYPE_NONE = 0
    ROADLINK_TYPE_ROAD = 1
    ROADLINK_TYPE_JUNCTION = 2
    
    # OpenDRIVE RoadLink ContactPoint enumeration values
    CONTACT_POINT_NONE = 0
    CONTACT_POINT_START = 1
    CONTACT_POINT_END = 2
    
    def __init__(self, verbose: bool = False, use_pyopendrive: bool = True):
        self.verbose = verbose
        self.use_pyopendrive = use_pyopendrive and PYOPENDRIVE_AVAILABLE
        self.nodes: List[PlainNode] = []
        self.edges: List[PlainEdge] = []
        self.connections: List[PlainConnection] = []
        
        # Mapping tables
        self.node_map: Dict[str, str] = {}  # OpenDRIVE junction ID -> Plain node ID
        self.road_map: Dict[str, OpenDriveRoad] = {}  # OpenDRIVE road ID -> Road data
        self.junction_roads: Dict[str, List[str]] = {}  # Junction ID -> List of connecting roads
        self.junction_connections: Dict[str, List[Dict]] = {}  # Junction ID -> List of connections
        self.junctions: List[Dict] = []  # List of junction data for processing connections
        
        # Lane mapping: (road_id, lane_id, direction) -> (edge_id, lane_index)
        self.lane_mapping: Dict[Tuple[str, int, str], Tuple[str, int]] = {}
        
        # Node counter
        self.node_counter = 0
        
        # pyOpenDRIVE specific
        self.odr_map = None  # type: Optional[PyOpenDriveMap]
        self.py_roads = {}  # type: Dict[str, PyRoad]
        self.py_junctions = {}  # type: Dict[str, PyJunction]
    
    def _decode_if_bytes(self, value):
        """将 bytes 转换为 str，如果已经是 str 则直接返回"""
        if isinstance(value, bytes):
            return value.decode('utf-8')
        return value
        
    def convert(self, xodr_file: str, output_prefix: str, use_netconvert: bool = True) -> bool:
        """
        Convert OpenDRIVE file to SUMO format
        
        Args:
            xodr_file: Input OpenDRIVE file path
            output_prefix: Output file prefix
            use_netconvert: Whether to use netconvert to generate final network
            
        Returns:
            Whether conversion was successful
        """
        try:
            # 1. Parse OpenDRIVE file
            logger.info(f"Parsing OpenDRIVE file: {xodr_file}")
            if self.use_pyopendrive:
                logger.info("Using pyOpenDRIVE for enhanced geometry processing")
                if not self._parse_with_pyopendrive(xodr_file):
                    return False
            else:
                logger.info("Using XML parsing (fallback mode)")
                if not self._parse_opendrive(xodr_file):
                    return False
            
            # 2. Convert to Plain XML elements
            logger.info("Converting to Plain XML format...")
            self._create_nodes()
            self._create_edges()
            self._create_connections()
            
            # 3. Write Plain XML files
            logger.info(f"Writing Plain XML files with prefix: {output_prefix}")
            self._write_plain_xml(output_prefix)
            
            # 4. Use netconvert to generate final network
            if use_netconvert:
                logger.info("Running netconvert to generate final network...")
                return self._run_netconvert(output_prefix)
            
            return True
            
        except Exception as e:
            import traceback
            logger.error(f"Conversion failed: {e}")
            traceback.print_exc()
            return False
    
    def _parse_opendrive(self, xodr_file: str) -> bool:
        """Parse OpenDRIVE file"""
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
                
                self.junctions.append(junction_data)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to parse OpenDRIVE: {e}")
            return False
    
    def _parse_road(self, road_elem: ET.Element) -> OpenDriveRoad:
        """Parse single road"""
        road = OpenDriveRoad(
            id=road_elem.get('id'),
            name=road_elem.get('name', ''),
            junction=road_elem.get('junction', '-1'),
            length=float(road_elem.get('length', 0))
        )
        
        # Parse geometry information
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
                
                # Determine geometry type
                if geom_elem.find('.//line') is not None:
                    geom['type'] = 'line'
                elif geom_elem.find('.//arc') is not None:
                    geom['type'] = 'arc'
                    arc = geom_elem.find('.//arc')
                    geom['curvature'] = float(arc.get('curvature', 0))
                else:
                    geom['type'] = 'line'
                
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
                        road.speed_limit = float(max_speed) / 3.6  # Convert km/h to m/s
                    elif speed_unit == 'mph':
                        road.speed_limit = float(max_speed) * 0.44704  # Convert mph to m/s
        
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
                        # Include shoulder lanes as well as driving lanes
                        if lane_type in ['driving', 'entry', 'exit', 'onRamp', 'offRamp', 'shoulder']:
                            lane_data = {
                                'id': int(lane.get('id')),
                                'type': lane_type,
                                'width': self._get_lane_width(lane)
                            }
                            
                            # 🆕 解析车道link信息
                            lane_link = lane.find('.//link')
                            if lane_link is not None:
                                predecessor = lane_link.find('.//predecessor')
                                successor = lane_link.find('.//successor')
                                
                                if predecessor is not None:
                                    lane_data['predecessor'] = {'id': int(predecessor.get('id'))}
                                if successor is not None:
                                    lane_data['successor'] = {'id': int(successor.get('id'))}
                            
                            road.lanes_left.append(lane_data)
                
                # Right lanes
                right = lane_section.find('.//right')
                if right is not None:
                    for lane in right.findall('.//lane'):
                        lane_type = lane.get('type')
                        # Include shoulder lanes as well as driving lanes
                        if lane_type in ['driving', 'entry', 'exit', 'onRamp', 'offRamp', 'shoulder']:
                            lane_data = {
                                'id': int(lane.get('id')),
                                'type': lane_type,
                                'width': self._get_lane_width(lane)
                            }
                            
                            # 🆕 解析车道link信息
                            lane_link = lane.find('.//link')
                            if lane_link is not None:
                                predecessor = lane_link.find('.//predecessor')
                                successor = lane_link.find('.//successor')
                                
                                if predecessor is not None:
                                    lane_data['predecessor'] = {'id': int(predecessor.get('id'))}
                                if successor is not None:
                                    lane_data['successor'] = {'id': int(successor.get('id'))}
                            
                            road.lanes_right.append(lane_data)
        
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
    
    def _parse_with_pyopendrive(self, xodr_file: str) -> bool:
        """Parse OpenDRIVE file using pyOpenDRIVE library for enhanced geometry processing"""
        try:
            # Load the OpenDRIVE map with pyOpenDRIVE
            self.odr_map = PyOpenDriveMap(xodr_file.encode())
            
            # Get all roads from pyOpenDRIVE
            py_roads = self.odr_map.get_roads()
            logger.info(f"Found {len(py_roads)} roads using pyOpenDRIVE")
            
            # Convert PyRoad objects to our internal format and store PyRoad references
            for py_road in py_roads:
                road_id = self._decode_if_bytes(py_road.id)
                self.py_roads[road_id] = py_road
                
                # Create OpenDriveRoad object compatible with existing code
                road = OpenDriveRoad(
                    id=road_id,
                    name=self._decode_if_bytes(py_road.name) if hasattr(py_road, 'name') else '',
                    junction=self._decode_if_bytes(py_road.junction),
                    length=py_road.length
                )
                
                # Get road links (predecessor/successor)
                if hasattr(py_road, 'predecessor') and py_road.predecessor and py_road.predecessor.type != self.ROADLINK_TYPE_NONE:
                    # Determine element type based on correct enumeration values
                    if py_road.predecessor.type == self.ROADLINK_TYPE_ROAD:
                        element_type = 'road'
                    elif py_road.predecessor.type == self.ROADLINK_TYPE_JUNCTION:
                        element_type = 'junction'
                    else:
                        logger.warning(f"Unknown predecessor type {py_road.predecessor.type} for road {road_id}")
                        element_type = None
                    
                    if element_type:
                        road.predecessor = {
                            'elementId': self._decode_if_bytes(py_road.predecessor.id),
                            'elementType': element_type,
                            'contactPoint': 'start' if py_road.predecessor.contact_point == self.CONTACT_POINT_START else 'end'
                        }
                else:
                    road.predecessor = None
                
                if hasattr(py_road, 'successor') and py_road.successor and py_road.successor.type != self.ROADLINK_TYPE_NONE:
                    # Determine element type based on correct enumeration values
                    if py_road.successor.type == self.ROADLINK_TYPE_ROAD:
                        element_type = 'road'
                    elif py_road.successor.type == self.ROADLINK_TYPE_JUNCTION:
                        element_type = 'junction'
                    else:
                        logger.warning(f"Unknown successor type {py_road.successor.type} for road {road_id}")
                        element_type = None
                    
                    if element_type:
                        road.successor = {
                            'elementId': self._decode_if_bytes(py_road.successor.id),
                            'elementType': element_type,
                            'contactPoint': 'start' if py_road.successor.contact_point == self.CONTACT_POINT_START else 'end'
                        }
                else:
                    road.successor = None
                
                # Parse lanes using pyOpenDRIVE
                lane_sections = py_road.get_lanesections()
                for lane_section in lane_sections:
                    lanes = lane_section.get_lanes()
                    for lane in lanes:
                        lane_data = {
                            'id': lane.id,
                            'type': self._decode_if_bytes(lane.type) if hasattr(lane, 'type') else 'driving',
                            's_start': lane_section.s0,
                            's_end': py_road.get_lanesection_end(lane_section)
                        }
                        
                        if lane.id < 0:  # Right lanes
                            road.lanes_right.append(lane_data)
                        elif lane.id > 0:  # Left lanes
                            road.lanes_left.append(lane_data)
                        # Skip center lane (id == 0)
                
                # Add geometry data from pyOpenDRIVE
                road.geometry = self._extract_geometry_from_pyopendrive(py_road)
                
                self.road_map[road_id] = road
                
                # Record junction internal roads
                if road.junction != '-1':
                    if road.junction not in self.junction_roads:
                        self.junction_roads[road.junction] = []
                    self.junction_roads[road.junction].append(road_id)
            
            # Get all junctions from pyOpenDRIVE
            py_junctions = self.odr_map.get_junctions()
            logger.info(f"Found {len(py_junctions)} junctions using pyOpenDRIVE")
            
            for py_junction in py_junctions:
                junction_id = self._decode_if_bytes(py_junction.id)
                self.py_junctions[junction_id] = py_junction
                
                junction_data = {
                    'id': junction_id,
                    'connections': []
                }
                self.junction_connections[junction_id] = []
                
                # Parse connections using pyOpenDRIVE's junction connection information
                connections_dict = py_junction.id_to_connection
                for conn_id, py_connection in connections_dict.items():
                    connection = {
                        'id': self._decode_if_bytes(conn_id),
                        'incomingRoad': self._decode_if_bytes(py_connection.incoming_road),
                        'connectingRoad': self._decode_if_bytes(py_connection.connecting_road),
                        'contactPoint': 'start' if py_connection.contact_point == 0 else 'end',
                        'laneLinks': []
                    }
                    
                    # Parse lane links
                    for lane_link in py_connection.lane_links:
                        connection['laneLinks'].append({
                            'from': lane_link.frm,  # Note: pyOpenDRIVE uses 'frm' not 'from'
                            'to': lane_link.to
                        })
                    
                    self.junction_connections[junction_id].append(connection)
                    junction_data['connections'].append(connection)
                
                self.junctions.append(junction_data)
            
            logger.info("Successfully parsed OpenDRIVE file using pyOpenDRIVE")
            return True
            
        except Exception as e:
            logger.error(f"Failed to parse OpenDRIVE with pyOpenDRIVE: {e}")
            logger.info("Falling back to XML parsing...")
            return self._parse_opendrive(xodr_file)
    
    def _extract_geometry_from_pyopendrive(self, py_road) -> List[Dict]:
        """Extract geometry data from pyOpenDRIVE road object"""
        geometry = []
        
        try:
            # Sample points along the road centerline
            # For most use cases, we need start point, some intermediate points, and end point
            num_samples = max(5, int(py_road.length / 10))  # At least 5 points, or one every 10 meters
            
            for i in range(num_samples + 1):
                s = (i / num_samples) * py_road.length
                
                # Get coordinates at (s, t=0, h=0) for centerline
                xyz = py_road.get_xyz(s, 0, 0)
                coords = xyz.array
                
                # Store in format compatible with existing code
                geom_data = {
                    'x': coords[0],
                    'y': coords[1], 
                    'hdg': 0,  # We'll calculate heading if needed
                    's': s,
                    'length': py_road.length / num_samples if i < num_samples else 0,
                    'type': 'line'  # Assume line segments for now
                }
                geometry.append(geom_data)
            
        except Exception as e:
            logger.warning(f"Failed to extract geometry from pyOpenDRIVE road {py_road.id}: {e}")
            # Return empty geometry, will fall back to (0, 0) coordinates
            return []
        
        return geometry
    
    def _get_lane_width(self, lane_elem: ET.Element) -> float:
        """Get lane width - take the last width element which is the actual width"""
        width_elems = lane_elem.findall('.//width')
        if width_elems:
            # Use the last width element (OpenDRIVE often has multiple with sOffset)
            for width_elem in reversed(width_elems):
                a_val = width_elem.get('a')
                if a_val and float(a_val) > 0:
                    return float(a_val)
            # If all width elements have a=0, use default
            return 3.5
        return 3.5
    
    def _determine_junction_type(self, junction_id: str, internal_road_ids: List[str]) -> str:
        """
        Determine junction type based on junction complexity
        
        Returns:
            Junction type: 'traffic_light', 'priority', 'right_before_left', etc.
        """
        # Count incoming/outgoing roads (not internal junction roads)
        connected_roads = set()
        total_lanes = 0
        
        for road_id in internal_road_ids:
            road = self.road_map[road_id]
            
            # Check predecessor
            if road.predecessor and road.predecessor['elementType'] == 'road':
                connected_roads.add(road.predecessor['elementId'])
            
            # Check successor  
            if road.successor and road.successor['elementType'] == 'road':
                connected_roads.add(road.successor['elementId'])
            
            # Count lanes
            total_lanes += len(road.lanes_left) + len(road.lanes_right)
        
        num_connected_roads = len(connected_roads)
        
        # Determine type based on complexity
        if num_connected_roads >= 4 and total_lanes > 8:
            # Complex intersection - use traffic lights
            return "traffic_light"
        elif num_connected_roads == 4:
            # 4-way intersection - could be priority or traffic light
            if total_lanes > 6:
                return "traffic_light"
            else:
                return "priority"
        elif num_connected_roads == 3:
            # T-junction - usually priority
            return "priority"
        elif num_connected_roads == 2:
            # Simple connection - unregulated
            return "priority"
        else:
            # Default to priority for simple junctions
            return "priority"
    
    def _create_nodes(self):
        """Create Plain XML nodes - one node per junction, regular nodes for road endpoints"""
        # Collect all junction IDs referenced by roads
        referenced_junctions = set()
        junction_road_endpoints = {}  # junction_id -> list of (x, y, road_id, position)
        
        # First pass: Identify junction connections and collect connection points
        for road_id, road in self.road_map.items():
            if road.junction != '-1':
                continue  # Skip junction internal roads
            
            # Check predecessor
            if road.predecessor and road.predecessor['elementType'] == 'junction':
                junction_id = road.predecessor['elementId']
                referenced_junctions.add(junction_id)
                start_pos = self._calculate_road_start(road)
                if start_pos:
                    if junction_id not in junction_road_endpoints:
                        junction_road_endpoints[junction_id] = []
                    junction_road_endpoints[junction_id].append((start_pos[0], start_pos[1], road_id, 'start'))
            else:
                # Create regular start node
                start_node = self._create_road_endpoint_node(road, 'start')
                self.node_map[f"{road_id}_start"] = start_node
            
            # Check successor
            if road.successor and road.successor['elementType'] == 'junction':
                junction_id = road.successor['elementId']
                referenced_junctions.add(junction_id)
                end_pos = self._calculate_road_end(road)
                if end_pos:
                    if junction_id not in junction_road_endpoints:
                        junction_road_endpoints[junction_id] = []
                    junction_road_endpoints[junction_id].append((end_pos[0], end_pos[1], road_id, 'end'))
            else:
                # Create regular end node
                end_node = self._create_road_endpoint_node(road, 'end')
                self.node_map[f"{road_id}_end"] = end_node
        
        # Second pass: Create junction nodes from internal roads (if any)
        for junction_id, internal_road_ids in self.junction_roads.items():
            if junction_id in referenced_junctions:
                continue  # Already handled by road connections
                
            # Calculate junction center from all internal roads
            center_x, center_y = self._calculate_junction_center(internal_road_ids)
            
            # Determine junction type based on complexity
            junction_type = self._determine_junction_type(junction_id, internal_road_ids)
            
            # Create single junction node
            node_id = f"junction_{junction_id}"
            self.nodes.append(PlainNode(
                id=node_id,
                x=center_x,
                y=center_y,
                type=junction_type
            ))
            self.node_map[junction_id] = node_id
            
            logger.debug(f"Created junction node {node_id} at ({center_x:.2f}, {center_y:.2f})")
        
        # Third pass: Create junction nodes from road connections
        for junction_id in referenced_junctions:
            if junction_id in self.node_map:
                continue  # Already created
                
            # Calculate junction center from connecting road endpoints
            if junction_id in junction_road_endpoints:
                points = junction_road_endpoints[junction_id]
                center_x = sum(p[0] for p in points) / len(points)
                center_y = sum(p[1] for p in points) / len(points)
                
                # Create junction node
                node_id = f"junction_{junction_id}"
                self.nodes.append(PlainNode(
                    id=node_id,
                    x=center_x,
                    y=center_y,
                    type="priority"
                ))
                self.node_map[junction_id] = node_id
                
                logger.debug(f"Created junction node {node_id} at ({center_x:.2f}, {center_y:.2f}) from {len(points)} road connections")
        
        # Fourth pass: Handle junctions from pyOpenDRIVE if using pyOpenDRIVE
        if self.use_pyopendrive and self.py_junctions:
            pyopendrive_junctions_created = 0
            for junction_id, py_junction in self.py_junctions.items():
                if junction_id not in self.node_map:  # Avoid duplicate creation
                    # Get junction center from pyOpenDRIVE
                    center_x, center_y = self._get_junction_center_from_pyopendrive(py_junction, junction_id)
                    
                    # Create junction node
                    node_id = f"junction_{junction_id}"
                    self.nodes.append(PlainNode(
                        id=node_id,
                        x=center_x,
                        y=center_y,
                        type="priority"
                    ))
                    self.node_map[junction_id] = node_id
                    pyopendrive_junctions_created += 1
                    
                    logger.debug(f"Created pyOpenDRIVE junction node {node_id} at ({center_x:.2f}, {center_y:.2f})")
            
            if pyopendrive_junctions_created > 0:
                logger.info(f"Created {pyopendrive_junctions_created} additional junction nodes from pyOpenDRIVE")
        
        total_junctions = len(self.junction_roads) + len(referenced_junctions - set(self.junction_roads.keys()))
        if self.use_pyopendrive:
            total_junctions += len([j for j in self.py_junctions.keys() if j not in self.node_map or j not in self.junction_roads])
        logger.info(f"Created {len(self.nodes)} nodes ({total_junctions} junctions)")
    
    def _create_road_endpoint_node(self, road: OpenDriveRoad, position: str) -> str:
        """Create a node for road endpoint (not connected to junction)"""
        if position == 'start':
            if road.geometry:
                x, y = road.geometry[0]['x'], road.geometry[0]['y']
            else:
                x, y = 0, 0
            node_id = f"node_{road.id}_start"
        else:  # end
            end_pos = self._calculate_road_end(road)
            if end_pos:
                x, y = end_pos
            else:
                x, y = 0, 0
            node_id = f"node_{road.id}_end"
        
        # Check if node already exists at this position
        tolerance = 0.01
        for node in self.nodes:
            if abs(node.x - x) < tolerance and abs(node.y - y) < tolerance:
                return node.id
        
        # Create new node
        self.nodes.append(PlainNode(id=node_id, x=x, y=y, type="priority"))
        return node_id
    
    def _calculate_junction_center(self, internal_road_ids: List[str]) -> Tuple[float, float]:
        """Calculate junction center from connected normal roads' endpoints"""
        # First, find the junction ID from internal roads
        junction_id = None
        for road_id in internal_road_ids:
            road = self.road_map.get(road_id)
            if road and road.junction != '-1':
                junction_id = road.junction
                break
        
        if not junction_id:
            # Fallback to original method
            return self._calculate_junction_center_from_internal_roads(internal_road_ids)
        
        # Collect connection points from normal roads
        connection_points = []
        
        # Find all normal roads that connect to this junction
        for road_id, road in self.road_map.items():
            if road.junction != '-1':  # Skip internal roads
                continue
            
            # Check if road ends at this junction (via successor)
            if road.successor and road.successor.get('elementType') == 'junction' and road.successor.get('elementId') == junction_id:
                end_point = self._calculate_road_end(road)
                if end_point:
                    connection_points.append(end_point)
                    logger.debug(f"Road {road_id} ends at junction {junction_id}: {end_point}")
            
            # Check if road starts from this junction (via predecessor)
            if road.predecessor and road.predecessor.get('elementType') == 'junction' and road.predecessor.get('elementId') == junction_id:
                if road.geometry:
                    start_point = (road.geometry[0]['x'], road.geometry[0]['y'])
                    connection_points.append(start_point)
                    logger.debug(f"Road {road_id} starts from junction {junction_id}: {start_point}")
        
        # Use connection points if found
        if connection_points:
            center_x = sum(p[0] for p in connection_points) / len(connection_points)
            center_y = sum(p[1] for p in connection_points) / len(connection_points)
            logger.debug(f"Junction {junction_id} center from {len(connection_points)} connection points: ({center_x:.2f}, {center_y:.2f})")
            return center_x, center_y
        
        # Fallback to internal roads method
        logger.debug(f"No connection points found for junction {junction_id}, using internal roads")
        return self._calculate_junction_center_from_internal_roads(internal_road_ids)
    
    def _calculate_junction_center_from_internal_roads(self, internal_road_ids: List[str]) -> Tuple[float, float]:
        """Fallback method: Calculate junction center from internal roads"""
        points = []
        
        for road_id in internal_road_ids:
            road = self.road_map.get(road_id)
            if not road or not road.geometry:
                continue
            
            # Add start point
            points.append((road.geometry[0]['x'], road.geometry[0]['y']))
            
            # Add end point
            end_pos = self._calculate_road_end(road)
            if end_pos:
                points.append(end_pos)
        
        if points:
            center_x = sum(p[0] for p in points) / len(points)
            center_y = sum(p[1] for p in points) / len(points)
            return center_x, center_y
        
        logger.warning(f"No geometry found for junction internal roads, using origin")
        return 0.0, 0.0
    
    def _get_junction_center_from_pyopendrive(self, py_junction, junction_id: str) -> Tuple[float, float]:
        """Get junction center coordinates from pyOpenDRIVE PyJunction object"""
        try:
            # Try to get junction bounding box or center from pyOpenDRIVE
            if hasattr(py_junction, 'bounding_box') and py_junction.bounding_box:
                # Use bounding box center
                bbox = py_junction.bounding_box
                center_x = (bbox.min_x + bbox.max_x) / 2.0
                center_y = (bbox.min_y + bbox.max_y) / 2.0
                logger.debug(f"Junction {junction_id} center from bounding box: ({center_x:.2f}, {center_y:.2f})")
                return center_x, center_y
            
        except Exception as e:
            logger.debug(f"Could not get bounding box for junction {junction_id}: {e}")
        
        # Fallback: calculate center from connected roads' endpoints
        connection_points = []
        
        # Find all roads that connect to this junction
        for road_id, road in self.road_map.items():
            if road.junction != '-1':  # Skip internal roads
                continue
            
            # Check if road connects to this junction
            if (road.predecessor and road.predecessor.get('elementType') == 'junction' 
                and road.predecessor.get('elementId') == junction_id):
                if road.geometry:
                    start_point = (road.geometry[0]['x'], road.geometry[0]['y'])
                    connection_points.append(start_point)
            
            if (road.successor and road.successor.get('elementType') == 'junction' 
                and road.successor.get('elementId') == junction_id):
                end_point = self._calculate_road_end(road)
                if end_point:
                    connection_points.append(end_point)
        
        # Calculate average position from connection points
        if connection_points:
            center_x = sum(p[0] for p in connection_points) / len(connection_points)
            center_y = sum(p[1] for p in connection_points) / len(connection_points)
            logger.debug(f"Junction {junction_id} center from {len(connection_points)} connection points: ({center_x:.2f}, {center_y:.2f})")
            return center_x, center_y
        
        # Final fallback: use origin if no connection points found
        logger.warning(f"Could not determine center for junction {junction_id}, using origin")
        return 0.0, 0.0
    
    def _find_junction_for_connecting_road(self, connecting_road_id: str) -> Optional[str]:
        """Find which junction contains the given connecting road"""
        logger.debug(f"Looking for junction for connecting road: {connecting_road_id}")
        
        # First check if this is a junction internal road
        connecting_road = self.road_map.get(connecting_road_id)
        if connecting_road and connecting_road.junction != '-1':
            logger.debug(f"Found connecting road {connecting_road_id} is junction internal road of junction {connecting_road.junction}")
            return connecting_road.junction
        
        # If using pyOpenDRIVE, check junction connections
        if self.use_pyopendrive and self.py_junctions:
            for junction_id, py_junction in self.py_junctions.items():
                # Check if this road is used as a connecting road in any junction connection
                connections_dict = py_junction.id_to_connection
                for conn_id, py_connection in connections_dict.items():
                    connecting_road_in_conn = self._decode_if_bytes(py_connection.connecting_road)
                    if connecting_road_in_conn == connecting_road_id:
                        logger.debug(f"Found connecting road {connecting_road_id} in junction {junction_id}")
                        return junction_id
        
        # Fallback: check if connecting_road_id is actually a junction ID
        if self.use_pyopendrive and connecting_road_id in self.py_junctions:
            logger.debug(f"Connecting road ID {connecting_road_id} is actually a junction ID")
            return connecting_road_id
        
        # No junction found
        logger.debug(f"No junction found for connecting road {connecting_road_id}")
        return None
    
    def _get_or_create_node(self, road: OpenDriveRoad, position: str) -> str:
        """Get or create node - maintaining precise geometry"""
        if position == 'start':
            link = road.predecessor
            if road.geometry:
                x, y = road.geometry[0]['x'], road.geometry[0]['y']
            else:
                x, y = 0, 0
        else:  # end
            link = road.successor
            end_pos = self._calculate_road_end(road)
            if end_pos:
                x, y = end_pos
            else:
                x, y = 0, 0
        
        # Check if connected to junction
        if link and link['elementType'] == 'junction':
            junction_id = link['elementId']
            if junction_id in self.node_map:
                # Junction node already exists
                return self.node_map[junction_id]
            else:
                # Create junction node at the exact connection point
                node_id = f"junction_{junction_id}"
                # Determine junction type (will be updated later if needed)
                junction_type = "priority"
                if junction_id in self.junction_roads:
                    junction_type = self._determine_junction_type(junction_id, self.junction_roads[junction_id])
                self.nodes.append(PlainNode(id=node_id, x=x, y=y, type=junction_type))
                self.node_map[junction_id] = node_id
                return node_id
        else:
            # Create regular node
            return self._create_new_node(x, y)
    
    def _create_new_node(self, x: float, y: float) -> str:
        """Create new node"""
        node_id = f"node_{self.node_counter}"
        self.node_counter += 1
        
        # Check if a similar node already exists (avoid duplicates)
        # Use very small tolerance to maintain geometric precision
        tolerance = 0.01  # 1cm tolerance - only merge truly identical points
        for node in self.nodes:
            if abs(node.x - x) < tolerance and abs(node.y - y) < tolerance:
                return node.id
        
        self.nodes.append(PlainNode(id=node_id, x=x, y=y))
        return node_id
    
    def _calculate_road_start(self, road: OpenDriveRoad) -> Optional[Tuple[float, float]]:
        """Calculate road start position"""
        if not road.geometry:
            return None
        
        # The start position is simply the first geometry's position
        first_geom = road.geometry[0]
        return (first_geom['x'], first_geom['y'])
    
    def _calculate_road_end(self, road: OpenDriveRoad) -> Optional[Tuple[float, float]]:
        """Calculate road endpoint position"""
        if not road.geometry:
            return None
        
        # Simplification: only consider the last geometry segment
        last_geom = road.geometry[-1]
        
        if last_geom['type'] == 'line':
            x = last_geom['x'] + last_geom['length'] * math.cos(last_geom['hdg'])
            y = last_geom['y'] + last_geom['length'] * math.sin(last_geom['hdg'])
            return (x, y)
        elif last_geom['type'] == 'arc' and 'curvature' in last_geom:
            # Arc endpoint calculation
            curvature = last_geom['curvature']
            if abs(curvature) > 0.001:
                angle_change = last_geom['length'] * curvature
                end_hdg = last_geom['hdg'] + angle_change
                
                radius = 1.0 / abs(curvature)
                if curvature > 0:
                    cx = last_geom['x'] - radius * math.sin(last_geom['hdg'])
                    cy = last_geom['y'] + radius * math.cos(last_geom['hdg'])
                    x = cx + radius * math.sin(end_hdg)
                    y = cy - radius * math.cos(end_hdg)
                else:
                    cx = last_geom['x'] + radius * math.sin(last_geom['hdg'])
                    cy = last_geom['y'] - radius * math.cos(last_geom['hdg'])
                    x = cx - radius * math.sin(end_hdg)
                    y = cy + radius * math.cos(end_hdg)
                return (x, y)
        
        # Default to straight line
        x = last_geom['x'] + last_geom['length'] * math.cos(last_geom['hdg'])
        y = last_geom['y'] + last_geom['length'] * math.sin(last_geom['hdg'])
        return (x, y)
    
    def _generate_road_shape(self, road: OpenDriveRoad) -> List[Tuple[float, float]]:
        """Generate shape points for road geometry"""
        if not road.geometry:
            return []
        
        shape_points = []
        
        for geom in road.geometry:
            if geom['type'] == 'line':
                # For straight lines, add start and end points
                x_start = geom['x']
                y_start = geom['y']
                x_end = x_start + geom['length'] * math.cos(geom['hdg'])
                y_end = y_start + geom['length'] * math.sin(geom['hdg'])
                
                if not shape_points:
                    shape_points.append((x_start, y_start))
                shape_points.append((x_end, y_end))
                
            elif geom['type'] == 'arc' and 'curvature' in geom:
                # Sample arc with intermediate points
                curvature = geom['curvature']
                arc_length = geom['length']
                
                if abs(curvature) < 0.0001:
                    # Nearly straight, treat as line
                    x_start = geom['x']
                    y_start = geom['y']
                    x_end = x_start + arc_length * math.cos(geom['hdg'])
                    y_end = y_start + arc_length * math.sin(geom['hdg'])
                    if not shape_points:
                        shape_points.append((x_start, y_start))
                    shape_points.append((x_end, y_end))
                else:
                    # Sample arc with improved density
                    radius = 1.0 / abs(curvature)
                    angle_change = arc_length * curvature
                    
                    # Calculate center of arc
                    if curvature > 0:
                        cx = geom['x'] - radius * math.sin(geom['hdg'])
                        cy = geom['y'] + radius * math.cos(geom['hdg'])
                    else:
                        cx = geom['x'] + radius * math.sin(geom['hdg'])
                        cy = geom['y'] - radius * math.cos(geom['hdg'])
                    
                    # Improved sampling strategy
                    # 1. Based on arc length: one point every 2 meters
                    # 2. Based on angle: one point every 5 degrees
                    # 3. Minimum 3 points, maximum 50 points
                    samples_by_length = arc_length / 2.0  # One point every 2 meters
                    samples_by_angle = abs(angle_change) * 180.0 / math.pi / 5.0  # One point every 5 degrees
                    num_samples = int(max(3, min(50, max(samples_by_length, samples_by_angle))))
                    
                    logger.debug(f"Arc sampling: length={arc_length:.2f}m, curvature={curvature:.4f}, samples={num_samples}")
                    
                    for i in range(num_samples + 1):
                        t = i / num_samples
                        current_angle = geom['hdg'] + t * angle_change
                        
                        if curvature > 0:
                            x = cx + radius * math.sin(current_angle)
                            y = cy - radius * math.cos(current_angle)
                        else:
                            x = cx - radius * math.sin(current_angle)
                            y = cy + radius * math.cos(current_angle)
                        
                        # Avoid duplicates (with smaller tolerance for better precision)
                        if not shape_points or (abs(x - shape_points[-1][0]) > 0.01 or 
                                                abs(y - shape_points[-1][1]) > 0.01):
                            shape_points.append((x, y))
            else:
                # Unsupported geometry type, use straight line approximation
                x_start = geom['x']
                y_start = geom['y']
                x_end = x_start + geom['length'] * math.cos(geom['hdg'])
                y_end = y_start + geom['length'] * math.sin(geom['hdg'])
                
                if not shape_points:
                    shape_points.append((x_start, y_start))
                shape_points.append((x_end, y_end))
        
        return shape_points
    
    def _get_road_from_node(self, road: OpenDriveRoad) -> Optional[str]:
        """Get the from node for a road"""
        if road.predecessor:
            if road.predecessor['elementType'] == 'junction':
                # Road starts from a junction - need to find which actual junction
                junction_id = self._find_junction_for_connecting_road(road.predecessor['elementId'])
                if junction_id:
                    return f"junction_{junction_id}"
                # Fallback: use the elementId directly (might be junction ID)
                return f"junction_{road.predecessor['elementId']}"
            elif road.predecessor['elementType'] == 'road':
                # Road connects to another road - find the shared connection point
                pred_road_id = road.predecessor['elementId']
                contact_point = road.predecessor.get('contactPoint', 'start')
                if contact_point == 'end':
                    shared_node_key = f"{pred_road_id}_end"
                else:
                    shared_node_key = f"{pred_road_id}_start"
                
                # Use the shared node from the predecessor road
                shared_node = self.node_map.get(shared_node_key)
                if not shared_node:
                    # If the shared node doesn't exist, create it based on the predecessor road
                    pred_road = self.road_map.get(pred_road_id)
                    if pred_road:
                        position = 'end' if contact_point == 'end' else 'start'
                        shared_node = self._create_road_endpoint_node(pred_road, position)
                        self.node_map[shared_node_key] = shared_node
                return shared_node
        # Road has no predecessor - use stored endpoint node (ensure it exists and is unique)
        node_key = f"{road.id}_start"
        if node_key not in self.node_map:
            # Create endpoint node if it doesn't exist
            start_node = self._create_road_endpoint_node(road, 'start')
            self.node_map[node_key] = start_node
        return self.node_map.get(node_key)
    
    def _get_road_to_node(self, road: OpenDriveRoad) -> Optional[str]:
        """Get the to node for a road"""
        if road.successor:
            if road.successor['elementType'] == 'junction':
                # Road ends at a junction - need to find which actual junction
                junction_id = self._find_junction_for_connecting_road(road.successor['elementId'])
                if junction_id:
                    return f"junction_{junction_id}"
                # Fallback: use the elementId directly (might be junction ID)
                return f"junction_{road.successor['elementId']}"
            elif road.successor['elementType'] == 'road':
                # Road connects to another road - find the shared connection point
                succ_road_id = road.successor['elementId']
                contact_point = road.successor.get('contactPoint', 'start')
                if contact_point == 'end':
                    shared_node_key = f"{succ_road_id}_end"
                else:
                    shared_node_key = f"{succ_road_id}_start"
                
                # Use the shared node from the successor road
                shared_node = self.node_map.get(shared_node_key)
                if not shared_node:
                    # If the shared node doesn't exist, create it based on the successor road
                    succ_road = self.road_map.get(succ_road_id)
                    if succ_road:
                        position = 'end' if contact_point == 'end' else 'start'
                        shared_node = self._create_road_endpoint_node(succ_road, position)
                        self.node_map[shared_node_key] = shared_node
                return shared_node
        # Road has no successor - use stored endpoint node (ensure it exists and is unique)
        node_key = f"{road.id}_end"
        if node_key not in self.node_map:
            # Create endpoint node if it doesn't exist
            end_node = self._create_road_endpoint_node(road, 'end')
            self.node_map[node_key] = end_node
        return self.node_map.get(node_key)
    
    # Note: _create_junction_edges is no longer needed since we use via points instead
    # Junction internal roads are not created as explicit edges in the new approach
    
    def _find_junction_road_endpoint(self, junction_road: OpenDriveRoad, junction_id: str, find_start: bool = False) -> Optional[str]:
        """Find the endpoint node for a junction internal road
        
        Args:
            junction_road: The junction internal road
            junction_id: The junction ID
            find_start: If True, find the start node (predecessor), else find the end node (successor)
        """
        if find_start:
            # Find the start node (predecessor)
            if junction_road.predecessor:
                if junction_road.predecessor['elementType'] == 'road':
                    predecessor_road_id = junction_road.predecessor['elementId']
                    # Check if the predecessor road arrives at or departs from this junction
                    if predecessor_road_id in self.road_map:
                        predecessor_road = self.road_map[predecessor_road_id]
                        # Check if this road's successor points to our junction
                        if predecessor_road.successor and predecessor_road.successor['elementId'] == junction_id:
                            # The predecessor road arrives at this junction
                            return f"road_{predecessor_road_id}_to_junction_{junction_id}"
                        # Check if this road's predecessor points to our junction
                        elif predecessor_road.predecessor and predecessor_road.predecessor['elementId'] == junction_id:
                            # The predecessor road departs from this junction
                            return f"road_{predecessor_road_id}_from_junction_{junction_id}"
        else:
            # Find the end node (successor)
            if junction_road.successor:
                if junction_road.successor['elementType'] == 'road':
                    successor_road_id = junction_road.successor['elementId']
                    # Check if the successor road arrives at or departs from this junction
                    if successor_road_id in self.road_map:
                        successor_road = self.road_map[successor_road_id]
                        # Check if this road's successor points to our junction
                        if successor_road.successor and successor_road.successor['elementId'] == junction_id:
                            # The successor road arrives at this junction
                            return f"road_{successor_road_id}_to_junction_{junction_id}"
                        # Check if this road's predecessor points to our junction
                        elif successor_road.predecessor and successor_road.predecessor['elementId'] == junction_id:
                            # The successor road departs from this junction
                            return f"road_{successor_road_id}_from_junction_{junction_id}"
        
        return None
    
    def _is_connecting_road(self, road_id: str) -> bool:
        """Check if a road is used as a connecting road in any junction
        
        Args:
            road_id: The road ID to check
            
        Returns:
            True if the road is used as a connecting road in any junction connection
        """
        for junction_connections in self.junction_connections.values():
            for conn in junction_connections:
                if conn.get('connectingRoad') == road_id:
                    return True
        return False
    
    def _create_edges(self):
        """Create Plain XML edges - only for normal roads, not junction internal roads"""
        for road_id, road in self.road_map.items():
            # Skip junction internal roads completely
            if road.junction != '-1':
                logger.debug(f"Skipping junction internal road {road_id}")
                continue
            
            # Check if this road is used as a connecting road
            if self._is_connecting_road(road_id):
                logger.info(f"Road {road_id} is used as a connecting road (junction=-1), will use its geometry for junction connections")
                continue
            
            # Determine from and to nodes
            from_node = self._get_road_from_node(road)
            to_node = self._get_road_to_node(road)
            
            if not from_node or not to_node:
                logger.warning(f"Cannot determine nodes for road {road_id}")
                continue
            
            # Generate shape points from road geometry
            if self.use_pyopendrive:
                # Use pyOpenDRIVE for enhanced geometry
                shape_points = self._get_road_centerline_pyopendrive(road_id, eps=0.5)
                if not shape_points:
                    logger.warning(f"pyOpenDRIVE geometry failed for road {road_id}, falling back to XML geometry")
                    shape_points = self._generate_road_shape(road)
            else:
                shape_points = self._generate_road_shape(road)
            
            # Create forward edge for right lanes (OpenDRIVE right lanes have negative IDs)
            if road.lanes_right:
                edge_id = f"{road_id}.0"
                # Prepare lane data with restrictions
                lane_data = []
                # Sort by ID ascending to map outer lanes to lower indices
                # OpenDRIVE: -4 (outermost) to -1 (innermost)
                # SUMO: index 0 (rightmost) to index n-1 (leftmost)
                sorted_right_lanes = sorted(road.lanes_right, key=lambda x: x['id'])
                for sumo_index, lane_info in enumerate(sorted_right_lanes):
                    lane_dict = {'width': lane_info.get('width', 3.2)}
                    # Set type and restrictions for shoulder lanes
                    lane_type = self._decode_if_bytes(lane_info['type'])
                    if lane_type == 'shoulder':
                        lane_dict['type'] = 'shoulder'  # Set lane type
                        lane_dict['disallow'] = 'all'   # Disallow all vehicles
                    lane_data.append(lane_dict)
                    
                    # 🆕 构建车道映射：(road_id, lane_id, direction) -> (edge_id, sumo_index)
                    opendrive_lane_id = lane_info['id']
                    mapping_key = (road_id, opendrive_lane_id, 'forward')
                    self.lane_mapping[mapping_key] = (edge_id, sumo_index)
                    logger.debug(f"Lane mapping: Road {road_id} lane {opendrive_lane_id} -> {edge_id}:{sumo_index}")
                
                self.edges.append(PlainEdge(
                    id=edge_id,
                    from_node=from_node,
                    to_node=to_node,
                    num_lanes=len(road.lanes_right),
                    speed=road.speed_limit,
                    name=road.name,
                    type=road.road_type,
                    shape=shape_points if len(shape_points) >= 2 else None,
                    lane_data=lane_data
                ))
                logger.debug(f"Created forward edge {edge_id}: {from_node} -> {to_node}")
            
            # Create backward edge for left lanes (OpenDRIVE left lanes have positive IDs)
            if road.lanes_left:
                edge_id = f"{road_id}.1"
                # Reverse shape points for backward direction
                reversed_shape = list(reversed(shape_points)) if shape_points else None
                # Prepare lane data with restrictions
                lane_data = []
                sorted_left_lanes = sorted(road.lanes_left, key=lambda x: x['id'])
                for sumo_index, lane_info in enumerate(sorted_left_lanes):
                    lane_dict = {'width': lane_info.get('width', 3.2)}
                    # Set type and restrictions for shoulder lanes
                    lane_type = self._decode_if_bytes(lane_info['type'])
                    if lane_type == 'shoulder':
                        lane_dict['type'] = 'shoulder'  # Set lane type
                        lane_dict['disallow'] = 'all'   # Disallow all vehicles
                    lane_data.append(lane_dict)
                    
                    # 🆕 构建车道映射：(road_id, lane_id, direction) -> (edge_id, sumo_index)
                    opendrive_lane_id = lane_info['id']
                    mapping_key = (road_id, opendrive_lane_id, 'backward')
                    self.lane_mapping[mapping_key] = (edge_id, sumo_index)
                    logger.debug(f"Lane mapping: Road {road_id} lane {opendrive_lane_id} -> {edge_id}:{sumo_index}")
                
                self.edges.append(PlainEdge(
                    id=edge_id,
                    from_node=to_node,  # Note: direction is reversed
                    to_node=from_node,
                    num_lanes=len(road.lanes_left),
                    speed=road.speed_limit,
                    name=road.name,
                    type=road.road_type,
                    shape=reversed_shape if reversed_shape and len(reversed_shape) >= 2 else None,
                    lane_data=lane_data
                ))
                logger.debug(f"Created backward edge {edge_id}: {to_node} -> {from_node}")
        
        # Validation: Count roads and edges to ensure correctness
        normal_roads = [r for r_id, r in self.road_map.items() if r.junction == '-1']
        connecting_roads = [r for r in normal_roads if self._is_connecting_road(r.id)]
        expected_max_edges = len(normal_roads) - len(connecting_roads)
        
        logger.info(f"Created {len(self.edges)} edges (junction internal roads excluded)")
        logger.info(f"  Total roads: {len(self.road_map)}")
        logger.info(f"  Normal roads (junction=-1): {len(normal_roads)}")
        logger.info(f"  Connecting roads: {len(connecting_roads)}")
        logger.info(f"  Expected max edges: {expected_max_edges * 2}")  # *2 for forward/backward
        
        if len(self.edges) > expected_max_edges * 2:
            logger.warning(f"Warning: More edges than expected! Possible junction explosion?")
            logger.warning(f"  Created edges: {len(self.edges)}")
            logger.warning(f"  Expected max: {expected_max_edges * 2}")
    
    def _create_connections(self):
        """Create Plain XML connections using connecting road geometry as via points"""
        total_connections = 0
        successful_connections = 0
        failed_connections = 0
        
        # Process each junction in OpenDRIVE
        for junction_id, junction_connections in self.junction_connections.items():
            logger.info(f"Processing junction {junction_id}: {len(junction_connections)} connections")
            
            for conn in junction_connections:
                total_connections += 1
                
                # Get connection details
                incoming_road_id = conn.get('incomingRoad')
                connecting_road_id = conn.get('connectingRoad')
                contact_point = conn.get('contactPoint', 'start')
                
                # Get road objects
                incoming_road = self.road_map.get(incoming_road_id)
                connecting_road = self.road_map.get(connecting_road_id)
                
                if not incoming_road or not connecting_road:
                    logger.warning(f"Missing roads for connection: {incoming_road_id} -> {connecting_road_id}")
                    failed_connections += 1
                    continue
                
                # Special logging for road 107 issue
                if connecting_road_id == "107":
                    logger.info(f"Processing Road 107: incoming={incoming_road_id}, contact={contact_point}")
                
                # Determine the actual outgoing road
                outgoing_road_id = self._get_outgoing_road_from_connecting(connecting_road, contact_point)
                if not outgoing_road_id:
                    logger.warning(f"Cannot determine outgoing road for connecting road {connecting_road_id}")
                    failed_connections += 1
                    continue
                
                # Debug for connecting road 5
                if connecting_road_id == "5":
                    logger.info(f"Connecting road 5: incoming={incoming_road_id}, outgoing={outgoing_road_id}, contact={contact_point}")
                
                if connecting_road_id == "107":
                    logger.info(f"Road 107 outgoing road: {outgoing_road_id}")
                
                # Check if connecting road is a normal road or junction internal road
                if connecting_road.junction == '-1':
                    # This is a normal road being used as a connecting road
                    # Use its FULL geometry as the junction internal path
                    logger.debug(f"Connecting road {connecting_road_id} is a normal road (junction=-1), using full geometry")
                    if self.use_pyopendrive:
                        via_points = self._get_road_centerline_pyopendrive(connecting_road_id, eps=0.3)
                        if not via_points:
                            logger.warning(f"pyOpenDRIVE geometry failed for connecting road {connecting_road_id}, using XML fallback")
                            via_points = self._extract_full_road_geometry(connecting_road)
                    else:
                        via_points = self._extract_full_road_geometry(connecting_road)
                else:
                    # This is a junction internal road
                    # Extract via points from connecting road geometry
                    logger.debug(f"Connecting road {connecting_road_id} is junction internal (junction={connecting_road.junction})")
                    if self.use_pyopendrive:
                        # For junction internal roads, also try pyOpenDRIVE first
                        via_points = self._get_road_centerline_pyopendrive(connecting_road_id, eps=0.3)
                        if not via_points:
                            logger.warning(f"pyOpenDRIVE geometry failed for junction internal road {connecting_road_id}, using XML fallback")
                            via_points = self._extract_connecting_road_geometry(connecting_road, contact_point)
                    else:
                        via_points = self._extract_connecting_road_geometry(connecting_road, contact_point)
                
                # Process lane links
                connection_created = False
                for lane_link in conn.get('laneLinks', []):
                    from_lane_id = lane_link.get('from')        # incoming road车道ID
                    connecting_lane_id = lane_link.get('to')    # connecting road车道ID
                    
                    # 🆕 使用全局映射表查找SUMO lane indices
                    # 1. 查找incoming road的SUMO mapping
                    incoming_direction = 'forward' if from_lane_id < 0 else 'backward'
                    from_mapping = self._get_sumo_lane_index(incoming_road_id, from_lane_id, incoming_direction)
                    
                    # 2. 获取真正的outgoing road车道ID
                    outgoing_lane_id = self._get_outgoing_lane_from_connecting_road(
                        connecting_road, connecting_lane_id, contact_point)
                    
                    if outgoing_lane_id is None:
                        # 使用fallback映射
                        outgoing_lane_id = self._fallback_lane_mapping(
                            connecting_road, connecting_lane_id, outgoing_road_id, contact_point)
                    
                    if outgoing_lane_id is None:
                        logger.warning(f"Cannot map connecting road {connecting_road_id} lane {connecting_lane_id} to outgoing road {outgoing_road_id}")
                        continue
                    
                    # 3. 查找outgoing road的SUMO mapping
                    outgoing_direction = 'forward' if outgoing_lane_id < 0 else 'backward'
                    to_mapping = self._get_sumo_lane_index(outgoing_road_id, outgoing_lane_id, outgoing_direction)
                    
                    if not from_mapping or not to_mapping:
                        logger.warning(f"Missing SUMO mapping: incoming={from_mapping}, outgoing={to_mapping}")
                        failed_connections += 1
                        continue
                    
                    from_edge, from_lane = from_mapping
                    to_edge, to_lane = to_mapping
                    
                    logger.debug(f"Direct mapping: Road {incoming_road_id} lane {from_lane_id} -> {from_edge}:{from_lane}")
                    logger.debug(f"Direct mapping: Road {outgoing_road_id} lane {outgoing_lane_id} -> {to_edge}:{to_lane}")
                    
                    if from_edge and to_edge:
                        # Verify edges connect at the junction
                        from_edge_obj = next((e for e in self.edges if e.id == from_edge), None)
                        to_edge_obj = next((e for e in self.edges if e.id == to_edge), None)
                        
                        if from_edge_obj and to_edge_obj:
                            # Both edges should connect to the same junction node
                            junction_node_id = f"junction_{junction_id}"
                            if (from_edge_obj.to_node == junction_node_id and 
                                to_edge_obj.from_node == junction_node_id):
                                
                                # Validate lane indices exist
                                if (from_lane < from_edge_obj.num_lanes and 
                                    to_lane < to_edge_obj.num_lanes):
                                    
                                    self.connections.append(PlainConnection(
                                        from_edge=from_edge,
                                        to_edge=to_edge,
                                        from_lane=from_lane,
                                        to_lane=to_lane,
                                        via=via_points  # Precise turning path
                                    ))
                                    logger.debug(f"Created connection: {from_edge}:{from_lane} -> {to_edge}:{to_lane}")
                                    connection_created = True
                                    successful_connections += 1
                                else:
                                    logger.debug(f"Invalid lane indices: {from_edge}:{from_lane} (max {from_edge_obj.num_lanes-1}) -> {to_edge}:{to_lane} (max {to_edge_obj.num_lanes-1})")
                                    failed_connections += 1
                            else:
                                logger.warning(f"Junction {junction_id} connection failed - nodes don't match:")
                                logger.warning(f"  from_edge ({from_edge}): to_node={from_edge_obj.to_node}")
                                logger.warning(f"  to_edge ({to_edge}): from_node={to_edge_obj.from_node}")
                                logger.warning(f"  expected junction node: {junction_node_id}")
                                failed_connections += 1
                    else:
                        if not from_edge:
                            logger.debug(f"Could not find from_edge for road {incoming_road_id}")
                        if not to_edge:
                            logger.debug(f"Could not find to_edge for road {outgoing_road_id}")
                        failed_connections += 1
                
                if not connection_created and connecting_road_id == "107":
                    logger.warning(f"Road 107 connection was not created!")
        
        logger.info(f"Connection statistics:")
        logger.info(f"  Total connections in OpenDRIVE: {total_connections}")
        logger.info(f"  Successfully created: {successful_connections}")
        logger.info(f"  Failed: {failed_connections}")
        logger.info(f"Created {len(self.connections)} connections with via points")
    
    def _get_outgoing_road_from_connecting(self, connecting_road: OpenDriveRoad, contact_point: str) -> Optional[str]:
        """Get the outgoing road ID from a connecting road"""
        if contact_point == 'end':
            # Reversed connection - connecting road is traversed from end to start
            # So the outgoing road is at the start (predecessor)
            if connecting_road.predecessor and connecting_road.predecessor['elementType'] == 'road':
                return connecting_road.predecessor['elementId']
        else:
            # Normal connection - connecting road is traversed from start to end
            # So the outgoing road is at the end (successor)
            if connecting_road.successor and connecting_road.successor['elementType'] == 'road':
                return connecting_road.successor['elementId']
        return None

    def _get_sumo_lane_index(self, road_id: str, lane_id: int, direction: str = 'forward') -> Optional[Tuple[str, int]]:
        """
        从全局映射表中获取SUMO edge ID和lane index
        
        Args:
            road_id: OpenDRIVE road ID
            lane_id: OpenDRIVE lane ID
            direction: 'forward' 或 'backward'
            
        Returns:
            Tuple of (edge_id, lane_index) if found, None otherwise
        """
        mapping_key = (road_id, lane_id, direction)
        result = self.lane_mapping.get(mapping_key)
        
        if result:
            edge_id, lane_index = result
            logger.debug(f"Found lane mapping: Road {road_id} lane {lane_id} ({direction}) -> {edge_id}:{lane_index}")
            return result
        else:
            logger.debug(f"No lane mapping found for: Road {road_id} lane {lane_id} ({direction})")
            return None

    def _get_outgoing_lane_from_connecting_road(self, connecting_road: OpenDriveRoad, connecting_lane_id: int, contact_point: str) -> Optional[int]:
        """
        Get the outgoing road lane ID from connecting road lane ID
        
        Args:
            connecting_road: The connecting road object
            connecting_lane_id: Lane ID in the connecting road
            contact_point: 'start' or 'end' - connection direction
            
        Returns:
            Lane ID in the outgoing road, or None if not found
        """
        # Find the lane in the connecting road
        target_lane = None
        
        # Check both left and right lanes
        for lane_info in connecting_road.lanes_left + connecting_road.lanes_right:
            if lane_info['id'] == connecting_lane_id:
                target_lane = lane_info
                break
        
        if not target_lane:
            logger.warning(f"Lane {connecting_lane_id} not found in connecting road {connecting_road.id}")
            return None
        
        # Get the lane links based on contact point
        if contact_point == 'end':
            # Reversed connection: use predecessor link
            lane_links = target_lane.get('predecessor', [])
            if isinstance(lane_links, dict):
                lane_links = [lane_links]
        else:
            # Normal connection: use successor link
            lane_links = target_lane.get('successor', [])
            if isinstance(lane_links, dict):
                lane_links = [lane_links]
        
        # Return the first valid lane link
        for link in lane_links:
            if isinstance(link, dict) and 'id' in link:
                outgoing_lane_id = int(link['id'])
                logger.debug(f"Connecting road {connecting_road.id} lane {connecting_lane_id} -> outgoing lane {outgoing_lane_id}")
                return outgoing_lane_id
        
        # If no explicit link found, try fallback mapping
        logger.warning(f"No lane link found for connecting road {connecting_road.id} lane {connecting_lane_id}")
        return None
    
    def _fallback_lane_mapping(self, connecting_road: OpenDriveRoad, connecting_lane_id: int, outgoing_road_id: str, contact_point: str) -> Optional[int]:
        """
        Fallback lane mapping when explicit lane links are not available
        Maps connecting road lanes to outgoing road lanes based on position and type
        """
        outgoing_road = self.road_map.get(outgoing_road_id)
        if not outgoing_road:
            return None
        
        # Get driving lanes from outgoing road
        if connecting_lane_id < 0:
            # Right lane (driving direction)
            outgoing_driving_lanes = [lane for lane in outgoing_road.lanes_right if lane['type'] == 'driving']
        else:
            # Left lane (opposite direction)
            outgoing_driving_lanes = [lane for lane in outgoing_road.lanes_left if lane['type'] == 'driving']
        
        if not outgoing_driving_lanes:
            return None
        
        # Sort lanes by ID
        outgoing_driving_lanes.sort(key=lambda x: abs(x['id']))
        
        # Simple mapping: try to map to similar lane index
        if connecting_lane_id == -1:
            # Rightmost driving lane
            return outgoing_driving_lanes[-1]['id'] if connecting_lane_id < 0 else outgoing_driving_lanes[0]['id']
        elif connecting_lane_id == 1:
            # Leftmost driving lane
            return outgoing_driving_lanes[0]['id'] if connecting_lane_id > 0 else outgoing_driving_lanes[-1]['id']
        else:
            # Try to find similar lane index
            for lane in outgoing_driving_lanes:
                if abs(lane['id']) == abs(connecting_lane_id):
                    return lane['id']
            
            # Default to first driving lane
            return outgoing_driving_lanes[0]['id']
    
    def _get_incoming_edge_and_lane(self, incoming_road: OpenDriveRoad, junction_id: str, lane_id: int) -> Tuple[Optional[str], int]:
        """Get edge and lane index for incoming road to junction"""
        # Determine which edge based on road-junction relationship
        if incoming_road.successor and incoming_road.successor.get('elementId') == junction_id:
            # Road arrives at junction via successor
            if lane_id < 0:  # Right lanes (driving direction)
                edge_id = f"{incoming_road.id}_forward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(incoming_road, lane_id, 'forward')
                if lane_idx is None:
                    logger.warning(f"Road {incoming_road.id}: Could not map incoming lane {lane_id} to SUMO index")
                    return None, 0
            else:  # Left lanes (opposite direction)
                edge_id = f"{incoming_road.id}_backward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(incoming_road, lane_id, 'backward')
                if lane_idx is None:
                    logger.warning(f"Road {incoming_road.id}: Could not map incoming lane {lane_id} to SUMO index")
                    return None, 0
        elif incoming_road.predecessor and incoming_road.predecessor.get('elementId') == junction_id:
            # Road arrives at junction via predecessor (reversed)
            if lane_id < 0:
                edge_id = f"{incoming_road.id}_backward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(incoming_road, lane_id, 'backward')
                if lane_idx is None:
                    logger.warning(f"Road {incoming_road.id}: Could not map incoming lane {lane_id} to SUMO index")
                    return None, 0
            else:
                edge_id = f"{incoming_road.id}_forward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(incoming_road, lane_id, 'forward')
                if lane_idx is None:
                    logger.warning(f"Road {incoming_road.id}: Could not map incoming lane {lane_id} to SUMO index")
                    return None, 0
        else:
            return None, 0
        
        # Check if edge exists
        if not any(e.id == edge_id for e in self.edges):
            return None, 0
        
        return edge_id, lane_idx
    
    def _map_opendrive_lane_to_sumo_index(self, road: OpenDriveRoad, target_lane_id: int, edge_direction: str = 'forward') -> Optional[int]:
        """
        智能映射OpenDRIVE车道ID到SUMO车道索引，正确处理shoulder车道差异
        
        Args:
            road: OpenDRIVE road对象
            target_lane_id: 目标车道ID（OpenDRIVE格式）
            edge_direction: 'forward' 或 'backward'
            
        Returns:
            SUMO车道索引，如果找不到则返回None
        """
        if edge_direction == 'forward' and target_lane_id < 0:
            # 前向边，右侧车道
            lanes = road.lanes_right
        elif edge_direction == 'backward' and target_lane_id > 0:
            # 后向边，左侧车道
            lanes = road.lanes_left
        elif edge_direction == 'forward' and target_lane_id > 0:
            # 前向边，左侧车道（实际用于backward edge）
            lanes = road.lanes_left
        elif edge_direction == 'backward' and target_lane_id < 0:
            # 后向边，右侧车道（实际用于backward edge）
            lanes = road.lanes_right
        else:
            return None
        
        # 1. 首先尝试精确匹配
        driving_lanes = [lane for lane in lanes if lane['type'] == 'driving']
        # 对于右侧车道（负ID），按绝对值从小到大排序（-1, -2, -3...）
        # 对于左侧车道（正ID），按值从小到大排序（1, 2, 3...）
        if target_lane_id < 0:
            # 右侧车道：-1是最内侧，-2, -3...向外，SUMO索引0是最右侧
            # 所以按绝对值倒序排列：-3, -2, -1 → SUMO indices [0, 1, 2]
            sorted_driving_lanes = sorted(driving_lanes, key=lambda x: -x['id'])
        else:
            # 左侧车道：1是最内侧，2, 3...向外
            sorted_driving_lanes = sorted(driving_lanes, key=lambda x: x['id'])
        
        logger.debug(f"Road {road.id} lane mapping: target={target_lane_id}, direction={edge_direction}")
        logger.debug(f"  All lanes: {[lane['id'] for lane in lanes]}")
        logger.debug(f"  Driving lanes: {[lane['id'] for lane in sorted_driving_lanes]}")
        
        # 精确匹配
        for idx, lane_info in enumerate(sorted_driving_lanes):
            if lane_info['id'] == target_lane_id:
                logger.debug(f"  Exact match: lane {target_lane_id} -> index {idx}")
                return idx
        
        # 2. 如果没有精确匹配，使用智能映射策略
        return self._intelligent_lane_mapping(road, target_lane_id, sorted_driving_lanes, edge_direction)
    
    def _intelligent_lane_mapping(self, road: OpenDriveRoad, target_lane_id: int, driving_lanes: List[Dict], edge_direction: str) -> Optional[int]:
        """
        智能车道映射策略，处理shoulder车道和车道不匹配的情况
        
        Args:
            road: OpenDRIVE road对象
            target_lane_id: 目标车道ID
            driving_lanes: 已排序的driving车道列表
            edge_direction: 边的方向
            
        Returns:
            最佳匹配的SUMO车道索引
        """
        if not driving_lanes:
            logger.warning(f"Road {road.id}: No driving lanes found for lane {target_lane_id}")
            return None
        
        logger.info(f"Road {road.id}: Applying intelligent mapping for lane {target_lane_id}")
        
        # 策略1：基于车道相对位置的映射
        if target_lane_id < 0:  # 右侧车道
            # 计算目标车道在所有右侧车道中的相对位置
            all_right_lanes = sorted([lane['id'] for lane in road.lanes_right], key=abs)
            try:
                target_position = all_right_lanes.index(target_lane_id)
                # 将相对位置映射到driving车道
                if target_position < len(driving_lanes):
                    mapped_idx = target_position
                    logger.info(f"  Position-based mapping: lane {target_lane_id} (pos {target_position}) -> index {mapped_idx}")
                    return mapped_idx
            except ValueError:
                pass
        
        # 策略2：基于车道数值的智能匹配
        # 找到最接近的driving车道
        driving_lane_ids = [lane['id'] for lane in driving_lanes]
        
        if target_lane_id < 0:  # 右侧车道
            # 对于右侧车道，找到绝对值最接近的driving车道
            closest_lane_id = min(driving_lane_ids, key=lambda x: abs(abs(x) - abs(target_lane_id)))
            closest_idx = next(idx for idx, lane in enumerate(driving_lanes) if lane['id'] == closest_lane_id)
            logger.info(f"  Closest-match mapping: lane {target_lane_id} -> lane {closest_lane_id} (index {closest_idx})")
            return closest_idx
        else:  # 左侧车道
            closest_lane_id = min(driving_lane_ids, key=lambda x: abs(abs(x) - abs(target_lane_id)))
            closest_idx = next(idx for idx, lane in enumerate(driving_lanes) if lane['id'] == closest_lane_id)
            logger.info(f"  Closest-match mapping: lane {target_lane_id} -> lane {closest_lane_id} (index {closest_idx})")
            return closest_idx
        
        # 策略3：默认映射
        if target_lane_id < 0:
            # 右侧车道默认映射到最右侧driving车道
            default_idx = len(driving_lanes) - 1
            logger.info(f"  Default mapping: lane {target_lane_id} -> rightmost driving lane (index {default_idx})")
            return default_idx
        else:
            # 左侧车道默认映射到最左侧driving车道
            logger.info(f"  Default mapping: lane {target_lane_id} -> leftmost driving lane (index 0)")
            return 0

    def _get_outgoing_edge_and_lane(self, outgoing_road_id: str, junction_id: str, lane_id: int, contact_point: str = 'start') -> Tuple[Optional[str], int]:
        """Get edge and lane index for outgoing road from junction
        
        Args:
            outgoing_road_id: The outgoing road ID
            junction_id: The junction ID
            lane_id: The lane ID from the connecting road's laneLink
            contact_point: 'start' or 'end' - if 'end', the connecting road is reversed
        """
        outgoing_road = self.road_map.get(outgoing_road_id)
        if not outgoing_road:
            return None, 0
        
        # When contact_point='end', the connecting road is traversed in reverse
        # This means the lane IDs are also reversed (positive becomes negative and vice versa)
        if contact_point == 'end':
            lane_id = -lane_id
            logger.debug(f"Reversed lane_id for contact_point='end': {-lane_id} -> {lane_id}")
        
        # Determine which edge based on road-junction relationship
        if outgoing_road.predecessor and outgoing_road.predecessor.get('elementId') == junction_id:
            # Road departs from junction via predecessor
            if lane_id < 0:  # Right lanes
                edge_id = f"{outgoing_road_id}_forward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(outgoing_road, lane_id, 'forward')
                if lane_idx is None:
                    logger.warning(f"Road {outgoing_road_id}: Could not map lane {lane_id} to SUMO index")
                    return None, 0
            else:  # Left lanes
                edge_id = f"{outgoing_road_id}_backward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(outgoing_road, lane_id, 'backward')
                if lane_idx is None:
                    logger.warning(f"Road {outgoing_road_id}: Could not map lane {lane_id} to SUMO index")
                    return None, 0
        elif outgoing_road.successor and outgoing_road.successor.get('elementId') == junction_id:
            # Road departs from junction via successor (reversed)
            if lane_id < 0:
                edge_id = f"{outgoing_road_id}_backward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(outgoing_road, lane_id, 'backward')
                if lane_idx is None:
                    logger.warning(f"Road {outgoing_road_id}: Could not map lane {lane_id} to SUMO index")
                    return None, 0
            else:
                edge_id = f"{outgoing_road_id}_forward"
                lane_idx = self._map_opendrive_lane_to_sumo_index(outgoing_road, lane_id, 'forward')
                if lane_idx is None:
                    logger.warning(f"Road {outgoing_road_id}: Could not map lane {lane_id} to SUMO index")
                    return None, 0
        else:
            return None, 0
        
        # Check if edge exists
        if not any(e.id == edge_id for e in self.edges):
            return None, 0
        
        return edge_id, lane_idx
    
    def _get_lane_geometry(self, road_id: str, lane_section, lane, eps: float = 0.5) -> Optional[List[Tuple[float, float]]]:
        """
        Extract lane geometry using pyOpenDRIVE
        
        Args:
            road_id: Road ID
            lane_section: PyLaneSection object
            lane: PyLane object
            eps: Sampling precision (smaller = more points)
            
        Returns:
            List of (x, y) tuples representing lane centerline
        """
        if not self.use_pyopendrive or road_id not in self.py_roads:
            return None
            
        try:
            py_road = self.py_roads[road_id]
            
            # Get lane mesh from pyOpenDRIVE
            lane_mesh = py_road.get_lane_mesh(lane, eps)
            
            if not lane_mesh or not hasattr(lane_mesh, 'vertices'):
                logger.warning(f"Failed to get mesh for lane {lane.id} on road {road_id}")
                return None
            
            # Extract centerline coordinates from mesh vertices
            # Note: lane mesh vertices include both boundaries and internal points
            # We need to extract a representative centerline
            vertices = lane_mesh.vertices
            
            if not vertices:
                return None
            
            # Simple approach: take every nth vertex to create a centerline
            # This may need refinement based on pyOpenDRIVE mesh structure
            step = max(1, len(vertices) // 50)  # Limit to ~50 points for performance
            centerline_points = []
            
            for i in range(0, len(vertices), step):
                vertex = vertices[i]
                if hasattr(vertex, 'array'):
                    # Vec3D objects have .array attribute
                    coords = vertex.array
                    centerline_points.append((coords[0], coords[1]))
                elif isinstance(vertex, (list, tuple)) and len(vertex) >= 2:
                    # Direct coordinate tuples
                    centerline_points.append((float(vertex[0]), float(vertex[1])))
                else:
                    logger.warning(f"Unknown vertex format in lane mesh: {type(vertex)}")
                    continue
            
            if not centerline_points:
                logger.warning(f"No valid coordinates extracted for lane {lane.id} on road {road_id}")
                return None
                
            return centerline_points
            
        except Exception as e:
            logger.warning(f"Error extracting geometry for lane {lane.id} on road {road_id}: {e}")
            return None
    
    def _get_road_centerline_pyopendrive(self, road_id: str, eps: float = 0.5) -> Optional[List[Tuple[float, float]]]:
        """
        Get road centerline using pyOpenDRIVE coordinate transformation
        
        Args:
            road_id: Road ID
            eps: Step size for sampling along the road
            
        Returns:
            List of (x, y) tuples representing road centerline
        """
        if not self.use_pyopendrive or road_id not in self.py_roads:
            return None
            
        try:
            py_road = self.py_roads[road_id]
            road_length = py_road.length
            
            # Sample points along the road centerline
            centerline_points = []
            s_step = eps
            current_s = 0.0
            
            while current_s <= road_length:
                # Get global coordinates for this s position at the road center (t=0)
                xyz = py_road.get_xyz(current_s, 0.0, 0.0)
                
                if hasattr(xyz, 'array'):
                    coords = xyz.array
                    centerline_points.append((coords[0], coords[1]))
                else:
                    logger.warning(f"Unexpected coordinate format from get_xyz: {type(xyz)}")
                
                current_s += s_step
            
            # Ensure we get the end point
            if current_s - s_step < road_length:
                xyz = py_road.get_xyz(road_length, 0.0, 0.0)
                if hasattr(xyz, 'array'):
                    coords = xyz.array
                    centerline_points.append((coords[0], coords[1]))
            
            return centerline_points if centerline_points else None
            
        except Exception as e:
            logger.warning(f"Error extracting centerline for road {road_id}: {e}")
            return None
    
    def _extract_connecting_road_geometry(self, connecting_road: OpenDriveRoad, contact_point: str) -> Optional[List[Tuple[float, float]]]:
        """Extract complete geometry from connecting road as via points"""
        via_points = []
        
        # Generate all geometry points for the connecting road
        for i, geom in enumerate(connecting_road.geometry):
            current_x = geom['x']
            current_y = geom['y']
            current_hdg = geom['hdg']
            length = geom['length']
            
            if geom['type'] == 'line':
                # For straight lines, add end point
                x_end = current_x + length * math.cos(current_hdg)
                y_end = current_y + length * math.sin(current_hdg)
                via_points.append((x_end, y_end))
                
            elif geom['type'] == 'arc' and 'curvature' in geom:
                # For arcs, sample multiple points
                curvature = geom['curvature']
                if abs(curvature) > 0.0001:
                    # Sample based on curvature
                    num_samples = max(3, min(10, int(abs(length * curvature) * 5)))
                    radius = 1.0 / abs(curvature)
                    
                    for j in range(1, num_samples + 1):
                        t = j / num_samples
                        angle_change = t * length * curvature
                        
                        if curvature > 0:
                            # Left turn
                            cx = current_x - radius * math.sin(current_hdg)
                            cy = current_y + radius * math.cos(current_hdg)
                            angle = current_hdg + angle_change
                            x = cx + radius * math.sin(angle)
                            y = cy - radius * math.cos(angle)
                        else:
                            # Right turn
                            cx = current_x + radius * math.sin(current_hdg)
                            cy = current_y - radius * math.cos(current_hdg)
                            angle = current_hdg + angle_change
                            x = cx - radius * math.sin(angle)
                            y = cy + radius * math.cos(angle)
                        
                        via_points.append((x, y))
                else:
                    # Nearly straight, treat as line
                    x_end = current_x + length * math.cos(current_hdg)
                    y_end = current_y + length * math.sin(current_hdg)
                    via_points.append((x_end, y_end))
            else:
                # Unsupported geometry, approximate with straight line
                x_end = current_x + length * math.cos(current_hdg)
                y_end = current_y + length * math.sin(current_hdg)
                via_points.append((x_end, y_end))
        
        # Reverse points if contact_point is 'end'
        if contact_point == 'end':
            via_points = list(reversed(via_points))
        
        # Remove first and last points (junction boundaries)
        if len(via_points) > 2:
            return via_points[1:-1]
        
        return via_points if via_points else None
    
    def _extract_full_road_geometry(self, road: OpenDriveRoad) -> Optional[List[Tuple[float, float]]]:
        """Extract the full geometry of a road (used for normal roads acting as connecting roads)
        
        Args:
            road: The road to extract geometry from
            
        Returns:
            List of (x, y) tuples representing the complete road geometry
        """
        points = []
        
        for geom in road.geometry:
            current_x = geom['x']
            current_y = geom['y']
            current_hdg = geom['hdg']
            length = geom['length']
            
            if geom['type'] == 'line':
                # For straight lines, add intermediate points for smooth path
                num_samples = max(2, int(length / 10))  # One point every 10 meters
                for i in range(num_samples):
                    t = i / (num_samples - 1) if num_samples > 1 else 0
                    x = current_x + t * length * math.cos(current_hdg)
                    y = current_y + t * length * math.sin(current_hdg)
                    points.append((x, y))
                    
            elif geom['type'] == 'arc' and 'curvature' in geom:
                # For arcs, sample more points based on curvature
                curvature = geom['curvature']
                if abs(curvature) > 0.0001:
                    num_samples = max(5, min(20, int(abs(length * curvature) * 10)))
                    radius = 1.0 / abs(curvature)
                    
                    for i in range(num_samples):
                        t = i / (num_samples - 1) if num_samples > 1 else 0
                        angle_change = t * length * curvature
                        
                        if curvature > 0:
                            # Left turn
                            cx = current_x - radius * math.sin(current_hdg)
                            cy = current_y + radius * math.cos(current_hdg)
                            angle = current_hdg + angle_change
                            x = cx + radius * math.sin(angle)
                            y = cy - radius * math.cos(angle)
                        else:
                            # Right turn
                            cx = current_x + radius * math.sin(current_hdg)
                            cy = current_y - radius * math.cos(current_hdg)
                            angle = current_hdg + angle_change
                            x = cx - radius * math.sin(angle)
                            y = cy + radius * math.cos(angle)
                        
                        points.append((x, y))
                else:
                    # Nearly straight, treat as line
                    x_end = current_x + length * math.cos(current_hdg)
                    y_end = current_y + length * math.sin(current_hdg)
                    points.append((current_x, current_y))
                    points.append((x_end, y_end))
            else:
                # Unsupported geometry type, approximate with line
                x_end = current_x + length * math.cos(current_hdg)
                y_end = current_y + length * math.sin(current_hdg)
                points.append((current_x, current_y))
                points.append((x_end, y_end))
        
        # Remove duplicate points
        if points:
            unique_points = [points[0]]
            for p in points[1:]:
                if abs(p[0] - unique_points[-1][0]) > 0.01 or abs(p[1] - unique_points[-1][1]) > 0.01:
                    unique_points.append(p)
            return unique_points if len(unique_points) >= 2 else None
        
        return None
    
    def _extract_via_points(self, connecting_road: OpenDriveRoad, contact_point: str) -> Optional[List[Tuple[float, float]]]:
        """Extract via points from connecting road geometry
        
        Args:
            connecting_road: The junction internal connecting road
            contact_point: 'start' or 'end' - determines if we reverse the points
            
        Returns:
            List of (x, y) tuples representing the geometry, or None if no geometry
        """
        # Generate shape points from road geometry
        shape_points = self._generate_road_shape(connecting_road)
        
        if not shape_points or len(shape_points) < 2:
            return None
        
        # Reverse points if contact_point is 'end'
        if contact_point == 'end':
            shape_points = list(reversed(shape_points))
        
        # Skip the first and last point (they're the junction boundaries)
        # Keep only intermediate points as via points
        if len(shape_points) > 2:
            return shape_points[1:-1]
        
        return None
    
    def _get_edge_and_lane(self, road_id: str, lane_id: int, direction: str, junction_id: str) -> Tuple[Optional[str], Optional[int]]:
        """Get SUMO edge ID and lane index from OpenDRIVE road and lane IDs
        
        Args:
            road_id: OpenDRIVE road ID
            lane_id: OpenDRIVE lane ID (negative for right/driving, positive for left/oncoming)
            direction: 'incoming' or 'outgoing' relative to junction
            junction_id: Junction ID for context
            
        Returns:
            Tuple of (edge_id, lane_index) or (None, None) if not found
        """
        road = self.road_map.get(road_id)
        if not road:
            return None, None
        
        # Determine edge ID and calculate lane index
        if direction == 'incoming':
            # Check if this road's successor or predecessor is the junction
            if road.successor and road.successor.get('elementId') == junction_id:
                # Road arrives at junction via successor - use forward edge for negative lanes
                if lane_id < 0:
                    edge_id = f"{road_id}_forward"
                    # Find the actual lane index in the sorted lanes list
                    # Sort ascending to match edge creation logic
                    sorted_lanes = sorted(road.lanes_right, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
                else:
                    edge_id = f"{road_id}_backward"
                    sorted_lanes = sorted(road.lanes_left, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
            elif road.predecessor and road.predecessor.get('elementId') == junction_id:
                # Road arrives at junction via predecessor - use backward edge for negative lanes
                if lane_id < 0:
                    edge_id = f"{road_id}_backward"
                    # For backward edge, right lanes are reversed
                    # Sort ascending to match edge creation logic
                    sorted_lanes = sorted(road.lanes_right, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
                else:
                    edge_id = f"{road_id}_forward"
                    sorted_lanes = sorted(road.lanes_left, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
            else:
                return None, None
        else:  # outgoing
            # For outgoing, check which end of the road connects to the junction
            if road.predecessor and road.predecessor.get('elementId') == junction_id:
                # Road departs from junction via predecessor - use forward edge for negative lanes
                if lane_id < 0:
                    edge_id = f"{road_id}_forward"
                    # Sort ascending to match edge creation logic
                    sorted_lanes = sorted(road.lanes_right, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
                else:
                    edge_id = f"{road_id}_backward"
                    sorted_lanes = sorted(road.lanes_left, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
            elif road.successor and road.successor.get('elementId') == junction_id:
                # Road departs from junction via successor - use backward edge for negative lanes
                if lane_id < 0:
                    edge_id = f"{road_id}_backward"
                    # Sort ascending to match edge creation logic
                    sorted_lanes = sorted(road.lanes_right, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
                else:
                    edge_id = f"{road_id}_forward"
                    sorted_lanes = sorted(road.lanes_left, key=lambda x: x['id'])
                    for idx, lane_info in enumerate(sorted_lanes):
                        if lane_info['id'] == lane_id:
                            lane_idx = idx
                            break
                    else:
                        return None, None
            else:
                return None, None
        
        # Check if this edge exists
        edge_exists = any(e.id == edge_id for e in self.edges)
        if not edge_exists:
            # Try without direction suffix for single-direction roads
            edge_id = road_id
            edge_exists = any(e.id == edge_id for e in self.edges)
            if not edge_exists:
                return None, None
        
        return edge_id, lane_idx
    
    def _get_edge_and_lane_from_connecting(self, connecting_road: OpenDriveRoad, lane_id: int, junction_id: str, contact_point: str) -> Tuple[Optional[str], Optional[int]]:
        """Get the outgoing edge and lane from a connecting road
        
        Args:
            connecting_road: The junction internal connecting road
            lane_id: Lane ID in the connecting road
            junction_id: Junction ID
            contact_point: 'start' or 'end' - determines which end connects to outgoing road
            
        Returns:
            Tuple of (edge_id, lane_index) for the outgoing road
        """
        # When contact_point is 'end', the connection is reversed
        # So the outgoing road is at the predecessor
        if contact_point == 'end':
            if connecting_road.predecessor and connecting_road.predecessor['elementType'] == 'road':
                outgoing_road_id = connecting_road.predecessor['elementId']
                # For contactPoint=end, the lane direction might be reversed
                # Convert connecting road lane to outgoing road lane
                # Negative lanes in connecting road map to negative lanes in outgoing road when normal
                # But when contactPoint=end, they might be flipped
                if lane_id > 0:
                    # Positive lane in connecting road (left side)
                    # Map to negative lane (right side) in outgoing road for reversed connection
                    mapped_lane_id = -1  # Use first driving lane
                else:
                    # Negative lane in connecting road
                    mapped_lane_id = -1  # Keep as driving lane
                return self._get_edge_and_lane(outgoing_road_id, mapped_lane_id, 'outgoing', junction_id)
        else:
            # Normal case: outgoing road is at the successor
            if connecting_road.successor and connecting_road.successor['elementType'] == 'road':
                outgoing_road_id = connecting_road.successor['elementId']
                # For normal connection, preserve lane direction
                mapped_lane_id = -1 if lane_id != 0 else -1  # Simplify to first driving lane
                return self._get_edge_and_lane(outgoing_road_id, mapped_lane_id, 'outgoing', junction_id)
        
        return None, None
    
    def _write_plain_xml(self, output_prefix: str):
        """Write Plain XML files"""
        # Write nodes file
        self._write_nodes(f"{output_prefix}.nod.xml")
        
        # Write edges file
        self._write_edges(f"{output_prefix}.edg.xml")
        
        # Write connections file
        if self.connections:
            self._write_connections(f"{output_prefix}.con.xml")
    
    def _write_nodes(self, filename: str):
        """Write nodes file"""
        root = ET.Element('nodes')
        
        for node in self.nodes:
            node_elem = ET.SubElement(root, 'node')
            node_elem.set('id', node.id)
            node_elem.set('x', str(node.x))
            node_elem.set('y', str(node.y))
            node_elem.set('type', node.type)
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space='    ')
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Written nodes to {filename}")
    
    def _write_edges(self, filename: str):
        """Write edges file"""
        root = ET.Element('edges')
        
        for edge in self.edges:
            edge_elem = ET.SubElement(root, 'edge')
            edge_elem.set('id', edge.id)
            edge_elem.set('from', edge.from_node)
            edge_elem.set('to', edge.to_node)
            edge_elem.set('numLanes', str(edge.num_lanes))
            edge_elem.set('speed', str(edge.speed))
            
            if edge.priority != 1:
                edge_elem.set('priority', str(edge.priority))
            if edge.name:
                edge_elem.set('name', edge.name)
            if edge.type:
                edge_elem.set('type', edge.type)
            
            # Add shape if available
            if edge.shape and len(edge.shape) >= 2:
                shape_str = ' '.join([f"{x:.2f},{y:.2f}" for x, y in edge.shape])
                edge_elem.set('shape', shape_str)
            
            # Add lane-specific data if available
            if edge.lane_data:
                for i, lane_data in enumerate(edge.lane_data):
                    lane_elem = ET.SubElement(edge_elem, 'lane')
                    lane_elem.set('index', str(i))
                    if 'width' in lane_data:
                        lane_elem.set('width', str(lane_data['width']))
                    if 'type' in lane_data:
                        lane_elem.set('type', lane_data['type'])
                    if 'allow' in lane_data:
                        lane_elem.set('allow', lane_data['allow'])
                    if 'disallow' in lane_data:
                        lane_elem.set('disallow', lane_data['disallow'])
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space='    ')
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Written edges to {filename}")
    
    def _write_connections(self, filename: str):
        """Write connections file with via points"""
        root = ET.Element('connections')
        
        for conn in self.connections:
            conn_elem = ET.SubElement(root, 'connection')
            conn_elem.set('from', conn.from_edge)
            conn_elem.set('to', conn.to_edge)
            conn_elem.set('fromLane', str(conn.from_lane))
            conn_elem.set('toLane', str(conn.to_lane))
            
            # Add via points if present
            if conn.via and len(conn.via) > 0:
                via_str = ' '.join([f"{x:.2f},{y:.2f}" for x, y in conn.via])
                conn_elem.set('via', via_str)
            
            if conn.dir != 's':
                conn_elem.set('dir', conn.dir)
            if conn.state != 'M':
                conn_elem.set('state', conn.state)
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space='    ')
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Written connections to {filename} (with via points)")
    
    def _run_netconvert(self, output_prefix: str) -> bool:
        """Run netconvert to generate final network with optimized parameters"""
        try:
            cmd = [
                'netconvert',
                '--node-files', f'{output_prefix}.nod.xml',
                '--edge-files', f'{output_prefix}.edg.xml',
                '--output-file', f'{output_prefix}.net.xml',
                
                # Junction handling - optimized for single-node junctions
                '--junctions.join', 'true',
                '--junctions.join-dist', '10',  # Merge nodes within 10m
                '--junctions.corner-detail', '5',  # Maximum corner detail
                '--junctions.internal-link-detail', '5',  # Maximum internal link detail
                '--junctions.limit-turn-speed', '5.5',  # Limit turning speed
                
                # Geometry processing
                '--geometry.remove', 'false',  # Keep all geometry points
                '--geometry.min-dist', '0.5',  # Minimum distance between geometry points
                '--rectangular-lane-cut', 'true',  # Use rectangular lane cutting at junctions
                
                # Connection handling
                '--no-turnarounds', 'true',  # Disable U-turns
                '--no-internal-links', 'false',  # Generate internal links
                '--check-lane-foes.all', 'true',  # Check all lane conflicts
                
                # Edge processing
                '--edges.join', 'false',  # Don't join edges (we want to preserve our structure)
                
                # Output options
                '--output.street-names', 'true',  # Preserve street names
                '--output.original-names', 'true',  # Keep original IDs
            ]
            
            # Add connections file if it exists
            conn_file = f'{output_prefix}.con.xml'
            if os.path.exists(conn_file):
                cmd.extend(['--connection-files', conn_file])
                logger.info(f"Using connections file: {conn_file}")
            
            # Add verbose output if requested
            if self.verbose:
                cmd.append('--verbose')
            
            # Run netconvert
            logger.info("Running netconvert with optimized parameters...")
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info(f"Successfully created {output_prefix}.net.xml")
                
                # Display statistics
                if os.path.exists(f'{output_prefix}.net.xml'):
                    tree = ET.parse(f'{output_prefix}.net.xml')
                    root = tree.getroot()
                    
                    # Count elements
                    junctions = root.findall('.//junction')
                    regular_junctions = [j for j in junctions if j.get('type') != 'internal']
                    edges = root.findall('.//edge')
                    regular_edges = [e for e in edges if not e.get('id', '').startswith(':')]
                    internal_edges = [e for e in edges if e.get('id', '').startswith(':')]
                    connections = root.findall('.//connection')
                    
                    logger.info(f"Network statistics:")
                    logger.info(f"  Junctions: {len(regular_junctions)} regular, {len(junctions) - len(regular_junctions)} internal")
                    logger.info(f"  Edges: {len(regular_edges)} regular, {len(internal_edges)} internal")
                    logger.info(f"  Connections: {len(connections)}")
                
                return True
            else:
                logger.error(f"netconvert failed with return code {result.returncode}")
                if result.stderr:
                    logger.error(f"Error output: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to run netconvert: {e}")
            return False

def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Convert OpenDRIVE to SUMO using Plain XML format (SUMO recommended method)'
    )
    parser.add_argument('--input', '-i',
                     help='Input OpenDRIVE file (.xodr)',
                     default="examples/xodr_sumo_maps/test_map_merge_split.xodr")
    parser.add_argument('--output', '-o',
                     help='Output prefix (default: based on input name)',
                     default="test")
    parser.add_argument('--no-netconvert', action='store_true', 
                       help='Only generate Plain XML files without running netconvert')
    parser.add_argument('--no-pyopendrive', action='store_true', 
                       help='Disable pyOpenDRIVE enhanced geometry processing (use XML parsing only)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    # Determine output prefix
    if args.output:
        output_prefix = args.output
    else:
        # Generate based on input filename
        base_name = os.path.splitext(os.path.basename(args.input))[0]
        output_prefix = base_name
    
    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Execute conversion
    use_pyopendrive = not args.no_pyopendrive  # Invert the flag
    converter = OpenDriveToSumoConverter(verbose=args.verbose, use_pyopendrive=use_pyopendrive)

    if use_pyopendrive and not PYOPENDRIVE_AVAILABLE:
        raise ImportError("pyOpenDRIVE is not installed. Please install it or run with --no-pyopendrive.")

    print("="*60)
    print("OpenDRIVE to SUMO Converter")
    print("Using SUMO Official Plain XML Method")
    print("="*60)
    
    success = converter.convert(
        args.input, 
        output_prefix, 
        use_netconvert=not args.no_netconvert
    )
    
    if success:
        print("\n✓ Conversion completed successfully!")
        print(f"  Plain XML files: {output_prefix}.nod.xml, {output_prefix}.edg.xml, {output_prefix}.con.xml")
        if not args.no_netconvert:
            print(f"  SUMO network: {output_prefix}.net.xml")
    else:
        print("\n✗ Conversion failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()