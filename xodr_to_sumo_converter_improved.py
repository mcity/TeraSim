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
    via: Optional[List[Tuple[float, float]]] = None

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
        self.junctions: List[Dict] = []
        
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
                
                self.junctions.append(junction_data)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to parse OpenDRIVE: {e}")
            return False
    
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
        """Get lane width"""
        width_elems = lane_elem.findall('.//width')
        if width_elems:
            # Take the last width element
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
        
        # Create nodes for non-junction roads
        for road_id, road in self.road_map.items():
            if road.junction == '-1':  # Not a junction road
                (start_x, start_y), (end_x, end_y) = self._get_road_start_end(road)
                
                # Create or find start node
                start_node_id = self._create_node(start_x, start_y)
                
                # Create or find end node
                end_node_id = self._create_node(end_x, end_y)
                
                # Store mapping
                self.node_map[f"{road_id}_start"] = start_node_id
                self.node_map[f"{road_id}_end"] = end_node_id
        
        logger.info(f"Created {len(self.nodes)} nodes")
    
    def _create_edges(self):
        """Create edges from roads with precise geometry"""
        logger.info("Creating edges...")
        
        for road_id, road in self.road_map.items():
            if road.junction == '-1':  # Regular road, not junction internal
                # Get nodes
                from_node = self.node_map.get(f"{road_id}_start")
                to_node = self.node_map.get(f"{road_id}_end")
                
                if not from_node or not to_node:
                    logger.warning(f"Missing nodes for road {road_id}")
                    continue
                
                # Calculate number of lanes
                num_lanes = max(len(road.lanes_right), len(road.lanes_left), 1)
                
                # Calculate road shape
                shape = self._calculate_road_shape(road)
                
                # Create edge
                edge = PlainEdge(
                    id=road_id,
                    from_node=from_node,
                    to_node=to_node,
                    num_lanes=num_lanes,
                    speed=road.speed_limit,
                    priority=1,
                    name=road.name,
                    shape=shape if len(shape) > 2 else None
                )
                
                self.edges.append(edge)
        
        logger.info(f"Created {len(self.edges)} edges")
    
    def _create_connections(self):
        """Create connections for junctions"""
        logger.info("Creating connections...")
        
        for junction_id, connections in self.junction_connections.items():
            for conn in connections:
                incoming_road = conn['incomingRoad']
                connecting_road = conn['connectingRoad']
                
                # Get the actual target road
                connecting_road_data = self.road_map.get(connecting_road)
                if not connecting_road_data:
                    continue
                
                # Find the target road based on successor/predecessor
                target_road = None
                if connecting_road_data.successor and \
                   connecting_road_data.successor['elementType'] == 'road':
                    target_road = connecting_road_data.successor['elementId']
                elif connecting_road_data.predecessor and \
                     connecting_road_data.predecessor['elementType'] == 'road':
                    target_road = connecting_road_data.predecessor['elementId']
                
                if not target_road or target_road == incoming_road:
                    continue
                
                # Create connections for each lane link
                for lane_link in conn['laneLinks']:
                    from_lane_idx = abs(lane_link['from']) - 1
                    to_lane_idx = abs(lane_link['to']) - 1
                    
                    connection = PlainConnection(
                        from_edge=incoming_road,
                        to_edge=target_road,
                        from_lane=from_lane_idx,
                        to_lane=to_lane_idx
                    )
                    
                    self.connections.append(connection)
        
        logger.info(f"Created {len(self.connections)} connections")
    
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
        """Write edges to Plain XML file"""
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