#!/usr/bin/env python3
"""
OpenDRIVE to SUMO Converter
Using SUMO's officially recommended Plain XML intermediate format

Author: TeraSim Team
License: MIT
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Set
import math
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
    shape: Optional[List[Tuple[float, float]]] = None  # Edge shape points
    
@dataclass
class PlainConnection:
    """SUMO Plain XML connection"""
    from_edge: str
    to_edge: str
    from_lane: int
    to_lane: int
    dir: str = "s"  # s=straight, r=right, l=left, t=turn(u-turn)
    state: str = "M"  # M=major, m=minor, =equal, s=stop, w=allway_stop, y=yield, o=dead_end

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
    
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.nodes: List[PlainNode] = []
        self.edges: List[PlainEdge] = []
        self.connections: List[PlainConnection] = []
        
        # Mapping tables
        self.node_map: Dict[str, str] = {}  # OpenDRIVE junction ID -> Plain node ID
        self.road_map: Dict[str, OpenDriveRoad] = {}  # OpenDRIVE road ID -> Road data
        self.junction_roads: Dict[str, List[str]] = {}  # Junction ID -> List of connecting roads
        
        # Node counter
        self.node_counter = 0
        
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
            logger.error(f"Conversion failed: {e}")
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
                        if lane.get('type') in ['driving', 'entry', 'exit', 'onRamp', 'offRamp']:
                            road.lanes_left.append({
                                'id': int(lane.get('id')),
                                'type': lane.get('type'),
                                'width': self._get_lane_width(lane)
                            })
                
                # Right lanes
                right = lane_section.find('.//right')
                if right is not None:
                    for lane in right.findall('.//lane'):
                        if lane.get('type') in ['driving', 'entry', 'exit', 'onRamp', 'offRamp']:
                            road.lanes_right.append({
                                'id': int(lane.get('id')),
                                'type': lane.get('type'),
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
        """Get lane width"""
        width_elem = lane_elem.find('.//width')
        if width_elem is not None:
            return float(width_elem.get('a', 3.5))
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
        """Create Plain XML nodes"""
        # First pass: Create nodes from non-junction roads
        # This establishes junction positions based on actual road endpoints
        for road_id, road in self.road_map.items():
            if road.junction != '-1':
                continue
            
            # Start node
            start_node = self._get_or_create_node(road, 'start')
            
            # End node
            end_node = self._get_or_create_node(road, 'end')
        
        # Second pass: Create any remaining junction nodes
        # These are junctions that might not be connected by regular roads
        for junction_id, road_ids in self.junction_roads.items():
            if junction_id in self.node_map:
                # Junction already created from road endpoints
                continue
                
            # Use the first internal road's start position as junction position
            junction_x, junction_y = None, None
            
            if road_ids:
                for road_id in road_ids:
                    road = self.road_map[road_id]
                    if road.geometry:
                        geom = road.geometry[0]
                        junction_x = geom['x']
                        junction_y = geom['y']
                        break
            
            # Fallback if no geometry found
            if junction_x is None or junction_y is None:
                junction_x, junction_y = 0, 0
                logger.warning(f"No geometry found for junction {junction_id}, using origin")
            
            # Determine junction type based on complexity
            junction_type = self._determine_junction_type(junction_id, road_ids)
            
            node_id = f"junction_{junction_id}"
            self.nodes.append(PlainNode(
                id=node_id,
                x=junction_x,
                y=junction_y,
                type=junction_type
            ))
            self.node_map[junction_id] = node_id
        
        logger.info(f"Created {len(self.nodes)} nodes")
    
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
                if abs(curvature) < 0.0001:
                    # Nearly straight, treat as line
                    x_start = geom['x']
                    y_start = geom['y']
                    x_end = x_start + geom['length'] * math.cos(geom['hdg'])
                    y_end = y_start + geom['length'] * math.sin(geom['hdg'])
                    if not shape_points:
                        shape_points.append((x_start, y_start))
                    shape_points.append((x_end, y_end))
                else:
                    # Sample arc with points
                    radius = 1.0 / abs(curvature)
                    angle_change = geom['length'] * curvature
                    
                    # Calculate center of arc
                    if curvature > 0:
                        cx = geom['x'] - radius * math.sin(geom['hdg'])
                        cy = geom['y'] + radius * math.cos(geom['hdg'])
                    else:
                        cx = geom['x'] + radius * math.sin(geom['hdg'])
                        cy = geom['y'] - radius * math.cos(geom['hdg'])
                    
                    # Sample points along arc
                    # Use adaptive sampling based on curvature
                    num_samples = max(3, int(abs(angle_change) * 10))  # More samples for sharper curves
                    
                    for i in range(num_samples + 1):
                        t = i / num_samples
                        current_angle = geom['hdg'] + t * angle_change
                        
                        if curvature > 0:
                            x = cx + radius * math.sin(current_angle)
                            y = cy - radius * math.cos(current_angle)
                        else:
                            x = cx - radius * math.sin(current_angle)
                            y = cy + radius * math.cos(current_angle)
                        
                        # Avoid duplicates
                        if not shape_points or (abs(x - shape_points[-1][0]) > 0.1 or 
                                                abs(y - shape_points[-1][1]) > 0.1):
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
    
    def _create_edges(self):
        """Create Plain XML edges"""
        for road_id, road in self.road_map.items():
            if road.junction != '-1':
                continue
            
            from_node = self._get_or_create_node(road, 'start')
            to_node = self._get_or_create_node(road, 'end')
            
            # Generate shape points from road geometry
            shape_points = self._generate_road_shape(road)
            
            # Create forward edge for right lanes (OpenDRIVE right lanes have negative IDs)
            if road.lanes_right:
                edge_id = f"{road_id}_forward"
                self.edges.append(PlainEdge(
                    id=edge_id,
                    from_node=from_node,
                    to_node=to_node,
                    num_lanes=len(road.lanes_right),
                    speed=road.speed_limit,
                    name=road.name,
                    type=road.road_type,
                    shape=shape_points if len(shape_points) > 2 else None
                ))
            
            # Create backward edge for left lanes (OpenDRIVE left lanes have positive IDs)
            if road.lanes_left:
                edge_id = f"{road_id}_backward"
                # Reverse shape points for backward direction
                reversed_shape = list(reversed(shape_points)) if shape_points else None
                self.edges.append(PlainEdge(
                    id=edge_id,
                    from_node=to_node,  # Note: direction is reversed
                    to_node=from_node,
                    num_lanes=len(road.lanes_left),
                    speed=road.speed_limit,
                    name=road.name,
                    type=road.road_type,
                    shape=reversed_shape if reversed_shape and len(reversed_shape) > 2 else None
                ))
        
        logger.info(f"Created {len(self.edges)} edges")
    
    def _create_connections(self):
        """Create Plain XML connections"""
        # Create connections for each junction node
        for node in self.nodes:
            if not node.id.startswith('junction_'):
                continue
            
            # Find edges connected to this junction
            incoming_edges = []
            outgoing_edges = []
            
            for edge in self.edges:
                if edge.to_node == node.id:
                    incoming_edges.append(edge)
                if edge.from_node == node.id:
                    outgoing_edges.append(edge)
            
            # Create connections (excluding U-turns)
            for in_edge in incoming_edges:
                for out_edge in outgoing_edges:
                    # Get road IDs
                    in_road = in_edge.id.split('_')[0]
                    out_road = out_edge.id.split('_')[0]
                    
                    # Don't allow U-turns
                    if in_road == out_road:
                        continue
                    
                    # Connect corresponding lanes
                    max_lanes = min(in_edge.num_lanes, out_edge.num_lanes)
                    for i in range(max_lanes):
                        self.connections.append(PlainConnection(
                            from_edge=in_edge.id,
                            to_edge=out_edge.id,
                            from_lane=i,
                            to_lane=i
                        ))
        
        logger.info(f"Created {len(self.connections)} connections")
    
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
            if edge.shape and len(edge.shape) > 2:
                shape_str = ' '.join([f"{x:.2f},{y:.2f}" for x, y in edge.shape])
                edge_elem.set('shape', shape_str)
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space='    ')
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Written edges to {filename}")
    
    def _write_connections(self, filename: str):
        """Write connections file"""
        root = ET.Element('connections')
        
        for conn in self.connections:
            conn_elem = ET.SubElement(root, 'connection')
            conn_elem.set('from', conn.from_edge)
            conn_elem.set('to', conn.to_edge)
            conn_elem.set('fromLane', str(conn.from_lane))
            conn_elem.set('toLane', str(conn.to_lane))
            
            if conn.dir != 's':
                conn_elem.set('dir', conn.dir)
            if conn.state != 'M':
                conn_elem.set('state', conn.state)
        
        tree = ET.ElementTree(root)
        ET.indent(tree, space='    ')
        tree.write(filename, encoding='utf-8', xml_declaration=True)
        logger.info(f"Written connections to {filename}")
    
    def _run_netconvert(self, output_prefix: str) -> bool:
        """Run netconvert to generate final network"""
        try:
            cmd = [
                'netconvert',
                '--node-files', f'{output_prefix}.nod.xml',
                '--edge-files', f'{output_prefix}.edg.xml',
                '--output-file', f'{output_prefix}.net.xml',
                '--no-turnarounds',
                '--junctions.join', 'true',
                '--junctions.join-dist', '10',
                '--edges.join', 'true'
            ]
            
            # Add connections file (if exists)
            conn_file = f'{output_prefix}.con.xml'
            if os.path.exists(conn_file):
                cmd.extend(['--connection-files', conn_file])
            
            # Run netconvert
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                logger.info(f"Successfully created {output_prefix}.net.xml")
                
                # Display statistics
                if os.path.exists(f'{output_prefix}.net.xml'):
                    tree = ET.parse(f'{output_prefix}.net.xml')
                    root = tree.getroot()
                    
                    junctions = len(root.findall('.//junction'))
                    edges = len([e for e in root.findall('.//edge') if not e.get('id', '').startswith(':')])
                    connections = len(root.findall('.//connection'))
                    
                    logger.info(f"Network statistics: {junctions} junctions, {edges} edges, {connections} connections")
                
                return True
            else:
                logger.error(f"netconvert failed: {result.stderr}")
                return False
                
        except Exception as e:
            logger.error(f"Failed to run netconvert: {e}")
            return False

def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Convert OpenDRIVE to SUMO using Plain XML format (SUMO recommended method)'
    )
    parser.add_argument('input', help='Input OpenDRIVE file (.xodr)')
    parser.add_argument('output', nargs='?', help='Output prefix (default: based on input name)')
    parser.add_argument('--no-netconvert', action='store_true', 
                       help='Only generate Plain XML files without running netconvert')
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
    converter = OpenDriveToSumoConverter(verbose=args.verbose)
    
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