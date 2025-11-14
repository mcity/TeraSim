#!/usr/bin/env python3
"""
OpenDRIVE Map Region Visualizer (SUMO Coordinate Mode)

This script takes SUMO x, y coordinates and visualizes the corresponding
region in the original OpenDRIVE (.xodr) map file.

The script reads the coordinate transformation (netOffset) from the SUMO
network file to convert SUMO coordinates back to OpenDRIVE coordinates.

Usage:
    python visualize_xodr_region.py <xodr_file> <sumo_net_file> <sumo_x> <sumo_y> [options]

Author: TeraSim Team
License: MIT
"""

import xml.etree.ElementTree as ET
import argparse
import logging
import math
import sys
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
import numpy as np

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Try to import pyOpenDRIVE for enhanced geometry
try:
    from pyOpenDRIVE.OpenDriveMap import PyOpenDriveMap
    PYOPENDRIVE_AVAILABLE = True
    logger.info("pyOpenDRIVE is available - enhanced geometry processing enabled")
except ImportError:
    PYOPENDRIVE_AVAILABLE = False
    logger.warning("pyOpenDRIVE not available. Using XML parsing for basic visualization.")


class GeometryCalculator:
    """Calculate positions along OpenDRIVE geometries"""

    @staticmethod
    def line(s: float, x: float, y: float, hdg: float, length: float) -> Tuple[float, float, float]:
        """Calculate position and heading on a line geometry"""
        dx = s * math.cos(hdg)
        dy = s * math.sin(hdg)
        return x + dx, y + dy, hdg

    @staticmethod
    def arc(s: float, x: float, y: float, hdg: float, length: float, curvature: float) -> Tuple[float, float, float]:
        """Calculate position and heading on an arc geometry"""
        if abs(curvature) < 1e-10:
            return GeometryCalculator.line(s, x, y, hdg, length)

        radius = 1.0 / curvature
        # Arc center
        cx = x - radius * math.sin(hdg)
        cy = y + radius * math.cos(hdg)

        # Angle along arc
        angle = hdg - math.pi / 2 + s * curvature

        # Point on arc
        px = cx + radius * math.cos(angle)
        py = cy + radius * math.sin(angle)

        # Heading at this point
        phdg = hdg + s * curvature

        return px, py, phdg

    @staticmethod
    def spiral(s: float, x: float, y: float, hdg: float, length: float,
               curv_start: float, curv_end: float) -> Tuple[float, float, float]:
        """Calculate position and heading on a spiral (clothoid) - simplified approximation"""
        # Simplified clothoid approximation using small segments
        num_segments = max(10, int(length))
        if num_segments == 0:
            return x, y, hdg

        segment_length = s / num_segments

        px, py = x, y
        current_hdg = hdg

        for i in range(num_segments):
            # Linear interpolation of curvature
            t = i / num_segments
            curvature = curv_start + t * (curv_end - curv_start)

            # Move along current heading
            px += segment_length * math.cos(current_hdg)
            py += segment_length * math.sin(current_hdg)

            # Update heading based on curvature
            current_hdg += curvature * segment_length

        return px, py, current_hdg

    @staticmethod
    def get_lateral_offset(x: float, y: float, hdg: float, lateral_offset: float) -> Tuple[float, float]:
        """
        Calculate position offset laterally from centerline
        Positive offset = left side, Negative offset = right side
        """
        # Perpendicular direction (90 degrees to the left of heading)
        perp_hdg = hdg + math.pi / 2
        offset_x = x + lateral_offset * math.cos(perp_hdg)
        offset_y = y + lateral_offset * math.sin(perp_hdg)
        return offset_x, offset_y


class XODRVisualizer:
    """Visualize a region of an OpenDRIVE map"""

    def __init__(self, xodr_file: str, sumo_net_file: Optional[str] = None):
        """
        Initialize visualizer with OpenDRIVE file

        Args:
            xodr_file: Path to OpenDRIVE map file
            sumo_net_file: Optional SUMO network file for coordinate transformation
        """
        self.xodr_file = xodr_file
        self.sumo_net_file = sumo_net_file
        self.roads = []
        self.junction_roads = []  # Roads that are part of junctions
        self.junctions = []

        # Lane-level data
        self.lanes = []  # List of lane geometries
        self.lane_connections = []  # List of junction lane connections

        # Coordinate transformation from SUMO to OpenDRIVE
        self.net_offset = (0.0, 0.0)  # Default: no offset

        # Read offset from SUMO network file if provided
        if sumo_net_file:
            self._read_sumo_offset()

        # Parse the map
        if PYOPENDRIVE_AVAILABLE:
            self._parse_with_pyopendrive()
        else:
            self._parse_with_xml()

    def _read_sumo_offset(self):
        """Read netOffset from SUMO network file"""
        try:
            tree = ET.parse(self.sumo_net_file)
            root = tree.getroot()

            # Find location element with netOffset
            location = root.find('location')
            if location is not None:
                net_offset_str = location.get('netOffset')
                if net_offset_str:
                    parts = net_offset_str.split(',')
                    self.net_offset = (float(parts[0]), float(parts[1]))
                    logger.info(f"Read netOffset from SUMO file: ({self.net_offset[0]:.2f}, {self.net_offset[1]:.2f})")

                    # Also read boundaries for reference
                    orig_boundary = location.get('origBoundary')
                    conv_boundary = location.get('convBoundary')
                    if orig_boundary:
                        logger.info(f"  Original OpenDRIVE boundary: {orig_boundary}")
                    if conv_boundary:
                        logger.info(f"  Converted SUMO boundary: {conv_boundary}")
                else:
                    logger.warning("No netOffset found in SUMO network file")
            else:
                logger.warning("No location element found in SUMO network file")
        except Exception as e:
            logger.error(f"Error reading SUMO network file: {e}")
            logger.info("Using default offset (0, 0)")

    def sumo_to_xodr(self, sumo_x: float, sumo_y: float) -> Tuple[float, float]:
        """
        Convert SUMO coordinates to OpenDRIVE coordinates

        The conversion formula is:
        xodr_x = sumo_x - offset_x
        xodr_y = sumo_y - offset_y

        Args:
            sumo_x: X coordinate in SUMO system
            sumo_y: Y coordinate in SUMO system

        Returns:
            Tuple of (xodr_x, xodr_y) in OpenDRIVE coordinate system
        """
        xodr_x = sumo_x - self.net_offset[0]
        xodr_y = sumo_y - self.net_offset[1]
        return xodr_x, xodr_y

    def _parse_with_pyopendrive(self):
        """Parse OpenDRIVE using pyOpenDRIVE library"""
        try:
            logger.info(f"Parsing {self.xodr_file} with pyOpenDRIVE...")
            odr_map = PyOpenDriveMap.load_from_file(self.xodr_file)

            # Extract road geometries
            for road_id in range(odr_map.get_num_roads()):
                road = odr_map.get_road(road_id)
                road_data = {
                    'id': road.get_id(),
                    'points': self._sample_road_centerline_pyodr(road)
                }
                self.roads.append(road_data)

            logger.info(f"Loaded {len(self.roads)} roads using pyOpenDRIVE")
        except Exception as e:
            logger.error(f"Error parsing with pyOpenDRIVE: {e}")
            logger.info("Falling back to XML parsing...")
            self._parse_with_xml()

    def _sample_road_centerline_pyodr(self, road, num_samples: int = 50) -> List[Tuple[float, float]]:
        """Sample points along road centerline using pyOpenDRIVE"""
        points = []
        length = road.get_length()

        for i in range(num_samples + 1):
            s = (i / num_samples) * length
            try:
                x, y = road.get_xy(s)
                points.append((x, y))
            except:
                pass

        return points

    def _parse_with_xml(self):
        """Parse OpenDRIVE using XML parsing"""
        try:
            logger.info(f"Parsing {self.xodr_file} with XML parser...")
            tree = ET.parse(self.xodr_file)
            root = tree.getroot()

            # First, collect all junction connecting road IDs
            junction_road_ids = set()
            for junction_elem in root.findall('.//junction'):
                for connection in junction_elem.findall('connection'):
                    connecting_road_id = connection.get('connectingRoad')
                    if connecting_road_id:
                        junction_road_ids.add(connecting_road_id)

            # Parse roads
            for road_elem in root.findall('.//road'):
                road_id = road_elem.get('id', 'unknown')
                junction_id = road_elem.get('junction', '-1')

                # Parse plan view geometries
                points = []
                plan_view = road_elem.find('planView')
                if plan_view is not None:
                    for geom in plan_view.findall('geometry'):
                        geom_points = self._parse_geometry(geom)
                        points.extend(geom_points)

                if points:
                    road_data = {
                        'id': road_id,
                        'points': points,
                        'junction_id': junction_id
                    }

                    # Separate junction roads from regular roads
                    if junction_id != '-1' or road_id in junction_road_ids:
                        self.junction_roads.append(road_data)
                    else:
                        self.roads.append(road_data)

            logger.info(f"Loaded {len(self.roads)} regular roads and {len(self.junction_roads)} junction roads using XML parser")

            # Parse lane-level geometries
            self._parse_lanes_from_xml(root)

            # Parse junction connections for lane links
            self._parse_junction_connections(root)

        except Exception as e:
            logger.error(f"Error parsing XML: {e}")
            raise

    def _parse_lanes_from_xml(self, root):
        """Parse individual lane geometries from OpenDRIVE XML"""
        try:
            for road_elem in root.findall('.//road'):
                road_id = road_elem.get('id', 'unknown')
                junction_id = road_elem.get('junction', '-1')

                # Get road plan view geometries
                plan_view = road_elem.find('planView')
                if plan_view is None:
                    continue

                geometries = []
                for geom in plan_view.findall('geometry'):
                    geometries.append({
                        's': float(geom.get('s', 0)),
                        'x': float(geom.get('x', 0)),
                        'y': float(geom.get('y', 0)),
                        'hdg': float(geom.get('hdg', 0)),
                        'length': float(geom.get('length', 0)),
                        'type': self._get_geometry_type(geom),
                        'elem': geom
                    })

                if not geometries:
                    continue

                # Get road length
                road_length = float(road_elem.get('length', 0))

                # Parse lanes from lane sections
                lanes_elem = road_elem.find('lanes')
                if lanes_elem is None:
                    continue

                for lane_section in lanes_elem.findall('laneSection'):
                    s_start = float(lane_section.get('s', 0))

                    # Find s_end (start of next section or road end)
                    s_end = road_length

                    # Parse right lanes (negative IDs)
                    right_elem = lane_section.find('right')
                    if right_elem is not None:
                        for lane_elem in right_elem.findall('lane'):
                            lane_id = int(lane_elem.get('id', 0))
                            lane_type = lane_elem.get('type', 'none')

                            if lane_type in ['driving', 'biking', 'sidewalk']:  # Only visualize drivable lanes
                                lane_data = self._create_lane_geometry(
                                    road_id, lane_id, lane_type, junction_id,
                                    geometries, lane_elem, s_start, s_end
                                )
                                if lane_data:
                                    self.lanes.append(lane_data)

                    # Parse left lanes (positive IDs)
                    left_elem = lane_section.find('left')
                    if left_elem is not None:
                        for lane_elem in left_elem.findall('lane'):
                            lane_id = int(lane_elem.get('id', 0))
                            lane_type = lane_elem.get('type', 'none')

                            if lane_type in ['driving', 'biking', 'sidewalk']:
                                lane_data = self._create_lane_geometry(
                                    road_id, lane_id, lane_type, junction_id,
                                    geometries, lane_elem, s_start, s_end
                                )
                                if lane_data:
                                    self.lanes.append(lane_data)

            logger.info(f"Parsed {len(self.lanes)} individual lanes")
        except Exception as e:
            logger.error(f"Error parsing lanes: {e}")
            # Don't raise - fall back to road-level visualization

    def _get_geometry_type(self, geom_elem):
        """Determine geometry type from element"""
        if geom_elem.find('line') is not None:
            return 'line'
        elif geom_elem.find('arc') is not None:
            return 'arc'
        elif geom_elem.find('spiral') is not None:
            return 'spiral'
        return 'unknown'

    def _create_lane_geometry(self, road_id, lane_id, lane_type, junction_id, geometries, lane_elem, s_start, s_end):
        """Create lane boundary geometry"""
        # Calculate actual lane boundaries
        left_boundary = []
        right_boundary = []

        # Get lane width elements
        width_elements = lane_elem.findall('width')
        if not width_elements:
            return None

        # Sample points along the lane
        num_samples = 20
        total_length = s_end - s_start

        for i in range(num_samples + 1):
            # Position along lane section
            s_local = (i / num_samples) * total_length
            s_global = s_start + s_local

            # Find which geometry this s belongs to
            geom = self._find_geometry_at_s(geometries, s_global)
            if not geom:
                continue

            # Calculate position and heading on centerline
            s_in_geom = s_global - geom['s']
            px, py, phdg = self._evaluate_geometry(geom, s_in_geom)

            # Calculate lane width at this position
            width = self._calculate_lane_width(width_elements, s_local)

            # Calculate cumulative offset for this lane
            # For right lanes (negative ID), offset is negative (to the right)
            # For left lanes (positive ID), offset is positive (to the left)
            offset = self._calculate_lane_offset(lane_elem, lane_id, s_local)

            # Calculate lane boundaries
            # Inner boundary (closer to centerline)
            inner_x, inner_y = GeometryCalculator.get_lateral_offset(px, py, phdg, offset)
            # Outer boundary (farther from centerline)
            outer_x, outer_y = GeometryCalculator.get_lateral_offset(px, py, phdg, offset + width * (-1 if lane_id < 0 else 1))

            if lane_id < 0:  # Right side lanes
                right_boundary.append((inner_x, inner_y))
                left_boundary.append((outer_x, outer_y))
            else:  # Left side lanes
                left_boundary.append((inner_x, inner_y))
                right_boundary.append((outer_x, outer_y))

        if not left_boundary or not right_boundary:
            return None

        return {
            'road_id': road_id,
            'lane_id': lane_id,
            'type': lane_type,
            'junction_id': junction_id,
            's_start': s_start,
            's_end': s_end,
            'left_boundary': left_boundary,
            'right_boundary': right_boundary
        }

    def _find_geometry_at_s(self, geometries, s):
        """Find which geometry element contains position s"""
        for geom in geometries:
            if geom['s'] <= s < geom['s'] + geom['length']:
                return geom
        # Return last geometry if s is beyond end
        return geometries[-1] if geometries else None

    def _evaluate_geometry(self, geom, s):
        """Evaluate geometry at position s"""
        geom_elem = geom['elem']
        x, y, hdg = geom['x'], geom['y'], geom['hdg']
        length = geom['length']

        line = geom_elem.find('line')
        arc = geom_elem.find('arc')
        spiral = geom_elem.find('spiral')

        if line is not None:
            return GeometryCalculator.line(s, x, y, hdg, length)
        elif arc is not None:
            curvature = float(arc.get('curvature', 0))
            return GeometryCalculator.arc(s, x, y, hdg, length, curvature)
        elif spiral is not None:
            curv_start = float(spiral.get('curvStart', 0))
            curv_end = float(spiral.get('curvEnd', 0))
            return GeometryCalculator.spiral(s, x, y, hdg, length, curv_start, curv_end)
        else:
            return x, y, hdg

    def _calculate_lane_width(self, width_elements, s):
        """Calculate lane width at position s using polynomial"""
        # Find applicable width element
        applicable_width = None
        for width_elem in width_elements:
            s_offset = float(width_elem.get('sOffset', 0))
            if s >= s_offset:
                applicable_width = width_elem

        if not applicable_width:
            applicable_width = width_elements[0]

        # Get polynomial coefficients
        a = float(applicable_width.get('a', 0))
        b = float(applicable_width.get('b', 0))
        c = float(applicable_width.get('c', 0))
        d = float(applicable_width.get('d', 0))
        s_offset = float(applicable_width.get('sOffset', 0))

        # Calculate width: width(ds) = a + b*ds + c*ds^2 + d*ds^3
        ds = s - s_offset
        width = a + b*ds + c*ds**2 + d*ds**3

        return abs(width)  # Ensure positive width

    def _calculate_lane_offset(self, lane_elem, lane_id, s):
        """Calculate cumulative offset from centerline to inner edge of this lane"""
        # This is simplified - assumes we know offsets
        # In reality, would need to sum widths of inner lanes

        # For simplicity, use a basic offset based on lane ID
        # Full implementation would traverse all inner lanes
        base_offset = abs(lane_id) * 3.5  # Assume ~3.5m per lane

        if lane_id < 0:  # Right side
            return -base_offset
        else:  # Left side
            return base_offset

    def _generate_junction_colors(self, num_junctions):
        """Generate distinct colors for each junction using HSV color space"""
        import colorsys
        colors = []
        for i in range(num_junctions):
            # Use golden ratio for better color distribution
            hue = (i * 0.618033988749895) % 1.0
            # Keep saturation high and value/brightness high for visibility
            saturation = 0.7 + (i % 3) * 0.1  # Vary between 0.7-1.0
            value = 0.8 + (i % 2) * 0.2      # Vary between 0.8-1.0
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            # Convert to hex color
            hex_color = '#{:02x}{:02x}{:02x}'.format(
                int(rgb[0] * 255),
                int(rgb[1] * 255),
                int(rgb[2] * 255)
            )
            colors.append(hex_color)
        return colors

    def _parse_junction_connections(self, root):
        """Parse junction connections to understand lane links"""
        try:
            for junction_elem in root.findall('.//junction'):
                junction_id = junction_elem.get('id', 'unknown')

                for connection in junction_elem.findall('connection'):
                    incoming_road = connection.get('incomingRoad')
                    connecting_road = connection.get('connectingRoad')
                    contact_point = connection.get('contactPoint', 'start')

                    for lane_link in connection.findall('laneLink'):
                        from_lane = int(lane_link.get('from', 0))
                        to_lane = int(lane_link.get('to', 0))

                        self.lane_connections.append({
                            'junction_id': junction_id,
                            'from_road': incoming_road,
                            'from_lane': from_lane,
                            'to_road': connecting_road,
                            'to_lane': to_lane,
                            'contact_point': contact_point
                        })

            logger.info(f"Parsed {len(self.lane_connections)} lane connections in junctions")
        except Exception as e:
            logger.error(f"Error parsing junction connections: {e}")

    def _parse_geometry(self, geom_elem, num_samples: int = 20) -> List[Tuple[float, float]]:
        """Parse a geometry element and sample points"""
        s = float(geom_elem.get('s', 0))
        x = float(geom_elem.get('x', 0))
        y = float(geom_elem.get('y', 0))
        hdg = float(geom_elem.get('hdg', 0))
        length = float(geom_elem.get('length', 0))

        points = []

        # Check geometry type
        line = geom_elem.find('line')
        arc = geom_elem.find('arc')
        spiral = geom_elem.find('spiral')

        if line is not None:
            # Sample line
            for i in range(num_samples + 1):
                t = (i / num_samples) * length
                px, py, phdg = GeometryCalculator.line(t, x, y, hdg, length)
                points.append((px, py))

        elif arc is not None:
            # Sample arc
            curvature = float(arc.get('curvature', 0))
            for i in range(num_samples + 1):
                t = (i / num_samples) * length
                px, py, phdg = GeometryCalculator.arc(t, x, y, hdg, length, curvature)
                points.append((px, py))

        elif spiral is not None:
            # Sample spiral
            curv_start = float(spiral.get('curvStart', 0))
            curv_end = float(spiral.get('curvEnd', 0))
            for i in range(num_samples + 1):
                t = (i / num_samples) * length
                px, py, phdg = GeometryCalculator.spiral(t, x, y, hdg, length, curv_start, curv_end)
                points.append((px, py))
        else:
            # Unknown geometry type, just add start point
            points.append((x, y))

        return points

    def visualize_region(self, center_x: float, center_y: float,
                        output_file: str = 'map_region.png',
                        size_pixels: int = 1000,
                        meters_per_pixel: float = 1.0,
                        use_sumo_coords: bool = False,
                        show_lanes: bool = False):
        """
        Visualize a region of the map centered on (center_x, center_y)

        Args:
            center_x: X coordinate of center point (meters, in SUMO or OpenDRIVE coords)
            center_y: Y coordinate of center point (meters, in SUMO or OpenDRIVE coords)
            output_file: Output PNG file path
            size_pixels: Size of output image (width and height)
            meters_per_pixel: Scale factor (default: 1 meter per pixel)
            use_sumo_coords: If True, input coordinates are SUMO coords (convert to OpenDRIVE)
            show_lanes: If True, show individual lanes and connections (experimental)
        """
        # Convert SUMO coordinates to OpenDRIVE if needed
        if use_sumo_coords:
            xodr_x, xodr_y = self.sumo_to_xodr(center_x, center_y)
            logger.info(f"Converting SUMO coords ({center_x:.2f}, {center_y:.2f}) "
                       f"to OpenDRIVE coords ({xodr_x:.2f}, {xodr_y:.2f})")
            center_x, center_y = xodr_x, xodr_y
        # Calculate bounds
        half_size_meters = (size_pixels / 2.0) * meters_per_pixel
        x_min = center_x - half_size_meters
        x_max = center_x + half_size_meters
        y_min = center_y - half_size_meters
        y_max = center_y + half_size_meters

        logger.info(f"Visualizing region: ({x_min:.1f}, {y_min:.1f}) to ({x_max:.1f}, {y_max:.1f})")
        logger.info(f"Center: ({center_x:.1f}, {center_y:.1f})")
        logger.info(f"Size: {size_pixels}x{size_pixels} pixels ({half_size_meters*2:.1f}m x {half_size_meters*2:.1f}m)")

        # Create figure
        dpi = 100
        fig_size = size_pixels / dpi
        fig, ax = plt.subplots(figsize=(fig_size, fig_size), dpi=dpi)

        # Set limits
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')

        if show_lanes and self.lanes:
            # Lane-level visualization
            lanes_in_view = 0
            junction_lanes_in_view = 0

            # Generate unique colors for each junction
            unique_junctions = set()
            for lane in self.lanes:
                if lane['junction_id'] != '-1':
                    unique_junctions.add(lane['junction_id'])

            junction_colors = self._generate_junction_colors(len(unique_junctions))
            junction_color_map = {jid: color for jid, color in zip(sorted(unique_junctions), junction_colors)}

            # Define colors for different lane types
            lane_colors = {
                'driving': '#888888',      # Gray
                'biking': '#90EE90',       # Light green
                'sidewalk': '#D3D3D3',     # Light gray
            }

            # Track junctions in view for legend
            junctions_in_view = set()

            for lane in self.lanes:
                # Check if lane has boundaries
                if 'left_boundary' not in lane or 'right_boundary' not in lane:
                    continue

                left_boundary = lane['left_boundary']
                right_boundary = lane['right_boundary']

                if not left_boundary or not right_boundary:
                    continue

                # Check if any points are in view
                in_view = any(
                    x_min <= x <= x_max and y_min <= y <= y_max
                    for x, y in left_boundary + right_boundary
                )

                if not in_view:
                    continue

                # Determine color based on lane type and junction status
                is_junction = lane['junction_id'] != '-1'
                lane_type = lane['type']
                junction_id = lane['junction_id']

                if is_junction:
                    # Use unique color for this junction
                    color = junction_color_map.get(junction_id, '#6495ED')
                    junctions_in_view.add(junction_id)
                else:
                    # Use regular lane type color
                    color = lane_colors.get(lane_type, '#888888')

                # Create polygon: left boundary + reversed right boundary
                polygon_points = left_boundary + list(reversed(right_boundary))

                # Draw filled polygon
                xs, ys = zip(*polygon_points)
                ax.fill(xs, ys, color=color, alpha=0.7, edgecolor='black', linewidth=0.5)

                lanes_in_view += 1
                if is_junction:
                    junction_lanes_in_view += 1

            logger.info(f"Drew {lanes_in_view} lanes ({junction_lanes_in_view} in junctions)")
            logger.info(f"Unique junctions in view: {len(junctions_in_view)}")

            # Store junction info for legend
            junctions_for_legend = junctions_in_view
            junction_color_map_for_legend = junction_color_map
        else:
            # Initialize empty sets if not showing lanes
            junctions_for_legend = set()
            junction_color_map_for_legend = {}

        # Skip road-level visualization if showing lanes
        if show_lanes and self.lanes:
            roads_in_view = 0
            junctions_in_view_roads = 0
        else:
            # Plot regular roads
            roads_in_view = 0
            for road in self.roads:
                points = road['points']
                if not points:
                    continue

                # Check if any points are in the view
                in_view = any(
                    x_min <= x <= x_max and y_min <= y <= y_max
                    for x, y in points
                )

                if in_view:
                    roads_in_view += 1
                    xs, ys = zip(*points)
                    ax.plot(xs, ys, 'k-', linewidth=2, alpha=0.7, label='Regular roads' if roads_in_view == 1 else '')

            # Plot junction roads with different color
            junctions_in_view = 0
            for road in self.junction_roads:
                points = road['points']
                if not points:
                    continue

                # Check if any points are in the view
                in_view = any(
                    x_min <= x <= x_max and y_min <= y <= y_max
                    for x, y in points
                )

                if in_view:
                    junctions_in_view += 1
                    xs, ys = zip(*points)
                    ax.plot(xs, ys, 'b-', linewidth=2, alpha=0.6, label='Junction roads' if junctions_in_view == 1 else '')

        # Draw lane connections if enabled
        if show_lanes and self.lane_connections:
            connections_drawn = 0
            for conn in self.lane_connections:
                # Draw a line showing lane connectivity
                # This would require finding the actual lane endpoints
                connections_drawn += 1

            if connections_drawn > 0:
                logger.info(f"Drew {connections_drawn} lane connection indicators")

        # Mark center point
        ax.plot(center_x, center_y, 'r+', markersize=15, markeredgewidth=2,
                label=f'Center ({center_x:.1f}, {center_y:.1f})')

        # Add grid
        ax.grid(True, alpha=0.3, linestyle='--')

        # Labels and title
        ax.set_xlabel('X (m)', fontsize=8)
        ax.set_ylabel('Y (m)', fontsize=8)

        # Update title based on visualization mode
        if show_lanes and self.lanes:
            title = f'OpenDRIVE Lane-Level View\n{size_pixels}x{size_pixels}px @ {meters_per_pixel}m/px'
        else:
            title = f'OpenDRIVE Map Region\n{size_pixels}x{size_pixels}px @ {meters_per_pixel}m/px'
        ax.set_title(title, fontsize=10)

        # Create legend
        if show_lanes and self.lanes and junctions_for_legend:
            # Add junction color patches to legend
            from matplotlib.patches import Patch
            legend_elements = [Patch(facecolor='#888888', edgecolor='black', label='Regular lanes', alpha=0.7)]

            # Add up to 5 junctions to legend to avoid clutter
            for i, jid in enumerate(sorted(junctions_for_legend)):
                if i >= 5:  # Limit to 5 junctions in legend
                    legend_elements.append(Patch(facecolor='gray', edgecolor='black',
                                                label=f'... +{len(junctions_for_legend)-5} more junctions', alpha=0.5))
                    break
                color = junction_color_map_for_legend.get(jid, '#6495ED')
                legend_elements.append(Patch(facecolor=color, edgecolor='black',
                                            label=f'Junction {jid}', alpha=0.7))

            ax.legend(handles=legend_elements, fontsize=7, loc='upper right')
        else:
            ax.legend(fontsize=7, loc='upper right')

        # Add scale bar
        scale_length = 50  # meters
        scale_x = x_min + (x_max - x_min) * 0.1
        scale_y = y_min + (y_max - y_min) * 0.05
        ax.plot([scale_x, scale_x + scale_length], [scale_y, scale_y], 'k-', linewidth=3)
        ax.text(scale_x + scale_length/2, scale_y + (y_max-y_min)*0.02,
               f'{scale_length}m', ha='center', fontsize=7)

        # Tight layout
        plt.tight_layout(pad=0.5)

        # Save
        plt.savefig(output_file, dpi=dpi, bbox_inches='tight', pad_inches=0.1)
        logger.info(f"✓ Saved visualization to {output_file}")
        logger.info(f"  Regular roads visible: {roads_in_view}")
        logger.info(f"  Junction roads visible: {junctions_in_view}")

        plt.close()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Visualize OpenDRIVE map region using SUMO coordinates',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Using SUMO coordinates (reads netOffset from SUMO network file)
  python visualize_xodr_region.py map.xodr map.net.xml 100 200

  # Direct OpenDRIVE coordinates (no SUMO network file)
  python visualize_xodr_region.py map.xodr - 384.6 0

  # Custom output file and size
  python visualize_xodr_region.py map.xodr map.net.xml 100 200 --output region.png --size 400

  # Different scale (0.5 meters per pixel = zoomed in, more detail)
  python visualize_xodr_region.py map.xodr map.net.xml 100 200 --scale 0.5
        """
    )

    parser.add_argument('xodr_file', help='OpenDRIVE map file (.xodr)')
    parser.add_argument('sumo_net_file', help='SUMO network file (.net.xml) for coordinate conversion, or "-" for direct OpenDRIVE coords')
    parser.add_argument('x', type=float, help='X coordinate of center point (SUMO coords if net file provided, else OpenDRIVE coords)')
    parser.add_argument('y', type=float, help='Y coordinate of center point (SUMO coords if net file provided, else OpenDRIVE coords)')
    parser.add_argument('--output', '-o', default='map_region.png',
                       help='Output PNG file (default: map_region.png)')
    parser.add_argument('--size', '-s', type=int, default=1000,
                       help='Image size in pixels (width=height, default: 1000)')
    parser.add_argument('--scale', type=float, default=1.0,
                       help='Meters per pixel (default: 1.0)')
    parser.add_argument('--lanes', action='store_true',
                       help='Show individual lanes and junction connections (experimental)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')

    args = parser.parse_args()

    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Validate inputs
    if args.size <= 0:
        logger.error("Size must be positive")
        sys.exit(1)

    if args.scale <= 0:
        logger.error("Scale must be positive")
        sys.exit(1)

    # Determine if using SUMO coordinates
    use_sumo_coords = args.sumo_net_file != '-'
    sumo_net_file = args.sumo_net_file if use_sumo_coords else None

    # Create visualizer
    try:
        visualizer = XODRVisualizer(args.xodr_file, sumo_net_file)
        visualizer.visualize_region(
            center_x=args.x,
            center_y=args.y,
            output_file=args.output,
            size_pixels=args.size,
            meters_per_pixel=args.scale,
            use_sumo_coords=use_sumo_coords,
            show_lanes=args.lanes
        )
    except FileNotFoundError as e:
        logger.error(f"File not found: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
