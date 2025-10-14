from pathlib import Path
from typing import Union

from shapely import LineString, Point

from sumolib.net import *
from sumolib.net.connection import Connection
from sumolib.net.edge import Edge
from sumolib.net.lane import Lane
from sumolib.net.node import Node

from waymo_open_dataset.protos import map_pb2, scenario_pb2

from terasim_datazoo.utils import *
from terasim_datazoo.utils.geometry import interpolate
from terasim_datazoo.utils.generic import ms_to_mph
import numpy as np

class SUMO2Waymo:
    """
    # SUMO -> Waymo

    A SUMO to Waymo map converter.

    ### Usage

    1. Instantiate a class object ``converter = SUMO2Waymo(sumo_file)``.
    """

    def __init__(self, sumo_file) -> None:

        self.sumonet: Net = readNet(sumo_file, withInternal=True)

        # SUMO stuffs
        self.normal_edges: dict[str, Edge] = {}
        self.internal_edges: dict[str, Edge] = {}
        self.crosswalk_edges: dict[str, Edge] = {}

        # Waymo stuffs
        self.feature_counter: int = 1
        self.lane_centers: dict = {}  # feature id, LaneCenter
        self.road_lines: dict = {}  # feature id, RoadLine
        self.road_edges: dict = {}  # feature id, RoadEdge
        self.crosswalks: dict = {}  # feature id, Crosswalk
        # self.junctions: dict = {}  # feature id, Junction
        self.map_lane2featureid: dict = {}  # Lane -> feature_id

    def parse(self, starter_edges: list[str] = None, have_road_edges=False, have_road_lines=False):
        """
        Parses the edges and performs several steps to process them.

        Args:
            starter_edges (list[str], optional): List of starter edges. Defaults to None.
        """
        print("Converting SUMO network format to Waymo map format...")

        if starter_edges:
            edge_set: set[Edge] = self._find_neighboring_edges(starter_edges)
        else:
            edge_set = set(self.sumonet.getEdges())

        # I. parse edges
        for edge in edge_set:
            # Skip crosswalks
            if edge.allows("pedestrian") and not edge.allows("passenger"):
                if edge.getFunction() == "crossing":
                    self.crosswalk_edges[edge.getID()] = edge
                else:
                    continue
            if not edge.isSpecial():
                self.normal_edges[edge.getID()] = edge
            else:
                self.internal_edges[edge.getID()] = edge

        # II. normal edges
        for _, edge in self.normal_edges.items():
            # 1. create lanecenter for each lane
            lanes: list[Lane] = edge.getLanes()
            # Filter out sidewalk lanes
            vehicle_lanes = [lane for lane in lanes if not (lane.allows("pedestrian") and not lane.allows("passenger"))]
            
            for lane in vehicle_lanes:
                self._create_lanecenter(lane)

            # 2. add neighbor information between them
            for i in range(0, len(vehicle_lanes) - 1):
                self._set_neighborship(vehicle_lanes[i], vehicle_lanes[i + 1])

            # 3. create road edge for this edge
            if have_road_edges and len(vehicle_lanes) > 0:
                self._create_roadedge(vehicle_lanes[0], side="right")
                self._create_roadedge(vehicle_lanes[-1], side="left")

            # 4. create road line
            if have_road_lines:
                for i in range(0, len(vehicle_lanes) - 1):
                    self._create_roadline(vehicle_lanes[i], side="left")

        # III. internal edges
        for _, edge in self.internal_edges.items():
            # 1. create lanecenter for each lane
            lanes: list[Lane] = edge.getLanes()
            # Filter out lanes that only allow pedestrians
            lanes = [lane for lane in lanes if not (lane.allows("pedestrian") and not lane.allows("passenger"))]
            for lane in lanes:
                self._create_lanecenter(lane)

            # 2. add neighbor information between them
            for i in range(0, len(lanes) - 1):
                self._set_neighborship(lanes[i], lanes[i + 1])

            # entry lanes and exit lanes information
            all_conns: list[Connection] = list(edge.getOutgoing().values()) + list(
                edge.getIncoming().values()
            )
            for conns in all_conns:
                for con in conns:
                    assert type(con) == Connection
                    self._set_connectionship(con.getFromLane(), con.getToLane())

        # IV. crosswalks
        for _, edge in self.crosswalk_edges.items():
            # Create crosswalk polygon from edge shape
            crosswalk = map_pb2.Crosswalk()
            center_line = edge.getShape()
            lane_width = edge.getLanes()[0].getWidth()  # Get width from first lane
            # Create parallel offset lines on both sides
            left_line = LineString(center_line).parallel_offset(lane_width/2, 'left')
            right_line = LineString(center_line).parallel_offset(lane_width/2, 'right')
            # Combine into polygon
            polygon = list(left_line.coords) + list(reversed(right_line.coords))

            z_default = edge.getShape3D()[0][2]
            
            # Convert 2D polygon points to 3D by adding z coordinate
            polygon_3d = []
            for point in polygon:
                # Get z coordinate from the first point if available, otherwise use 0
                z = point[2] if len(point) > 2 else z_default
                polygon_3d.append(map_pb2.MapPoint(x=point[0], y=point[1], z=z))
            
            crosswalk.polygon.extend(polygon_3d)
            
            # Store crosswalk in map features
            self.crosswalks[self.feature_counter] = crosswalk
            self.feature_counter += 1
            

        # V. junctions (nodes)
        all_node_types = [node.getType() for node in self.sumonet.getNodes()]
        for node in self.sumonet.getNodes():
            self._create_junction_outline_shape(node)

        print("Sumo2Waymo parsing completed")


    def _create_junction_outline_shape(self, node: Node):
        """
        given a sumo node, create a closed polyline of the node's outline
        """
        node_shape = node.getShape3D()
        roadedge = map_pb2.RoadEdge()
        # Make a closed polyline by appending the first point to the end
        if len(node_shape) > 0:
            node_shape = list(node_shape)  # Convert to list if it's not already
            node_shape.append(node_shape[0])  # Append first point to end

        # get all connection edges of this node
        all_conns: list[Connection] = node.getOutgoing() + node.getIncoming()
        all_conns_polyline_start_ending_points = []
        for edge in all_conns:
            edge_shape = edge.getShape3D()
            all_conns_polyline_start_ending_points.append(edge_shape[0])
            all_conns_polyline_start_ending_points.append(edge_shape[-1])
        
        if node.getType() == "priority" or node.getType() == "priority_stop": # works for normal roundabouts and pure connections
            # Create polylines from node_shape points (two points per polyline)
            external_polylines = []
            for i in range(len(node_shape) - 1):
                polyline = [node_shape[i], node_shape[i + 1]]
                # Check if this polyline overlaps with any point in node_shape_polyline_list
                is_internal = False
                for pt in all_conns_polyline_start_ending_points:
                    # Calculate distance from point to line segment
                    line = LineString([(polyline[0][0], polyline[0][1]), (polyline[1][0], polyline[1][1])])
                    point = Point(pt[0], pt[1])
                    if line.distance(point) < 0.1:
                        is_internal = True
                        break
                if not is_internal:
                    external_polylines.append(polyline)
        elif node.getType() == "traffic_light" or node.getType() == "allway_stop":
            all_connections = node.getConnections()
            all_right_connections = [conn for conn in all_connections if conn.getDirection() == "r" or conn.getDirection() == "R"]
            right_connection_lanes = [conn.getFromLane() for conn in all_right_connections]
            right_connection_internal_lanes = [lane for lane in right_connection_lanes if lane.getID().startswith(":")]
            # right_connection_internal_lanes = [lane for lane in right_connection_lanes if lane.getID() in node.getInternal()]
            right_connection_shapes = [lane.getShape3D() for lane in right_connection_internal_lanes]
            right_connection_polylines = [LineString(shape) for shape in right_connection_shapes]
            right_connection_polyline_right_half_width = [polyline.parallel_offset(lane.getWidth()/2, 'right') for polyline, lane in zip(right_connection_polylines, right_connection_internal_lanes)]
            external_polylines = [[(x,y) for x,y in zip(*polyline.xy)] for polyline in right_connection_polyline_right_half_width]
            for i, polyline in enumerate(external_polylines):
                for j, pt in enumerate(polyline):
                    external_polylines[i][j] = (pt[0], pt[1], right_connection_shapes[i][j][2])
        else:
            external_polylines = [node_shape]

        # right boundary: type == ROAD_EDGE_BOUNDARY | left boundary: type == ROAD_EDGE_MEDIAN
        for polyline in external_polylines:
            roadedge = map_pb2.RoadEdge()
            roadedge.polyline.extend([map_pb2.MapPoint(x=pt[0], y=pt[1], z=pt[2]) for pt in polyline])
            self.road_edges[self.feature_counter] = roadedge
            self.feature_counter += 1


    def _create_junction_outline_shape_new(self, node: Node):
        """
        given a sumo node, create a closed polyline of the node's outline
        """
        node_shape = node.getShape3D()
        roadedge = map_pb2.RoadEdge()

        # get all internal lanes of this node
        internal_lanes = node.getInternal()
        # Get lane object from lane id
        lane_objects = []
        for lane_id in internal_lanes:
            lane = self.sumonet.getLane(lane_id)
            if lane is not None:
                lane_objects.append(lane)
        

        # Make a closed polyline by appending the first point to the end
        if len(node_shape) > 0:
            node_shape = list(node_shape)  # Convert to list if it's not already
            node_shape.append(node_shape[0])  # Append first point to end

        node_connections = node.getConnections()
        right_node_connections = []
        for conn in node_connections:
            direction = conn.getDirection()
            if direction == "r" or "R":
                right_node_connections.append(conn)
        


        # all_connection_edge_shapes_union_polygon = Polygon(all_edge_polygons_union)
        

        external_polylines = [node_shape]

        # right boundary: type == ROAD_EDGE_BOUNDARY | left boundary: type == ROAD_EDGE_MEDIAN
        for polyline in external_polylines:
            roadedge = map_pb2.RoadEdge()
            roadedge.polyline.extend([map_pb2.MapPoint(x=pt[0], y=pt[1], z=pt[2]) for pt in polyline])
            self.road_edges[self.feature_counter] = roadedge
            self.feature_counter += 1

    def _create_lanecenter(self, lane: Lane):
        """
        given a sumo lane, create a LaneCenter() and store it in self.lane_centers
        """
        lanecenter = map_pb2.LaneCenter()
        polyline = interpolate(lane.getShape3D(), lane.getLength())
        assert len(polyline) >= 2
        lanecenter.polyline.extend([map_pb2.MapPoint(x=pt[0], y=pt[1], z=pt[2]) for pt in polyline])
        lanecenter.speed_limit_mph = ms_to_mph(lane.getSpeed())
        lanecenter.type = 2

        self.map_lane2featureid[lane] = self.feature_counter
        self.lane_centers[self.feature_counter] = lanecenter
        self.feature_counter += 1


    def _create_roadline(self, lane: Lane, side: str):
        """
        given a sumo lane, create a RoadLine() on its left or right, and store it in self.road_lines
        return the created road_line

        side = left | right
        """

        assert lane in self.map_lane2featureid
        lanecenter = self.lane_centers[self.map_lane2featureid[lane]]
        polyline = lanecenter.polyline[1:-1]
        if len(polyline) <= 1:
            return
            
        # Create 2D LineString for offset calculation
        polyline_2d = LineString([(pt.x, pt.y) for pt in polyline])
        offset = lane.getWidth() / 2 if side == "left" else -lane.getWidth() / 2
        boundary_2d = polyline_2d.offset_curve(offset, join_style=1)
        boundary_2d = list(boundary_2d.coords)
        
        # Interpolate z coordinates for the offset curve
        z_coords = [pt.z for pt in polyline]
        distances_orig = [0]
        for i in range(1, len(polyline)):
            dist = ((polyline[i].x - polyline[i-1].x)**2 + 
                   (polyline[i].y - polyline[i-1].y)**2)**0.5
            distances_orig.append(distances_orig[-1] + dist)
            
        distances_new = [0]
        for i in range(1, len(boundary_2d)):
            dist = ((boundary_2d[i][0] - boundary_2d[i-1][0])**2 + 
                   (boundary_2d[i][1] - boundary_2d[i-1][1])**2)**0.5
            distances_new.append(distances_new[-1] + dist)
            
        # Scale distances to match original
        scale = distances_orig[-1] / distances_new[-1] if distances_new[-1] > 0 else 1
        distances_new = [d * scale for d in distances_new]
        
        # Interpolate z coordinates
        new_z = np.interp(distances_new, distances_orig, z_coords)
        boundary = [(x, y, z) for (x, y), z in zip(boundary_2d[::-1], new_z[::-1])]

        roadline = map_pb2.RoadLine()
        roadline.type = 1  # default to be DASH WHITE LINE
        roadline.polyline.extend([map_pb2.MapPoint(x=pt[0], y=pt[1], z=pt[2]) for pt in boundary])

        self.road_lines[self.feature_counter] = roadline
        self.feature_counter += 1

    def _create_roadedge(self, lane: Lane, side: str):
        """
        given a sumo lane, create a RoadEdge() on its left or right, and store it in self.road_edges
        return the created road_line

        side = left | right
        """

        assert lane in self.map_lane2featureid
        lanecenter = self.lane_centers[self.map_lane2featureid[lane]]
        polyline = lanecenter.polyline[1:-1]
        if len(polyline) <= 1:
            return
            
        # Check if the adjacent lane is a sidewalk
        edge = lane.getEdge()
        lanes = edge.getLanes()
        lane_index = lanes.index(lane)
        
        # For right side, check if next lane is sidewalk
        if side == "right" and lane_index < len(lanes) - 1:
            next_lane = lanes[lane_index + 1]
            if next_lane.allows("pedestrian") and not next_lane.allows("passenger"):
                return
                
        # For left side, check if previous lane is sidewalk
        if side == "left" and lane_index > 0:
            prev_lane = lanes[lane_index - 1]
            if prev_lane.allows("pedestrian") and not prev_lane.allows("passenger"):
                return
            
        # Create 2D LineString for offset calculation
        polyline_2d = LineString([(pt.x, pt.y) for pt in polyline])
        offset = lane.getWidth() / 2 if side == "left" else -lane.getWidth() / 2
        try:
            boundaries_2d = polyline_2d.offset_curve(offset, join_style=1)
        except:
            return
            
        if type(boundaries_2d) == LineString:
            boundaries_2d = [boundaries_2d]
        else:
            boundaries_2d = list(boundaries_2d.geoms)

        for bd_2d in boundaries_2d:
            boundary_2d = list(bd_2d.coords)
            
            # Interpolate z coordinates for the offset curve
            z_coords = [pt.z for pt in polyline]
            
            # Interpolate z coordinates based on relative distances
            new_z = np.interp(
                np.linspace(0, 1, len(boundary_2d)), 
                np.linspace(0, 1, len(z_coords)), 
                z_coords
            )
            boundary = [(x, y, z) for (x, y), z in zip(boundary_2d[::-1], new_z[::-1])]

            roadedge = map_pb2.RoadEdge()
            roadedge.type = 2 if side == "left" else 1
            # right boundary: type == ROAD_EDGE_BOUNDARY | left boundary: type == ROAD_EDGE_MEDIAN
            roadedge.polyline.extend([map_pb2.MapPoint(x=pt[0], y=pt[1], z=pt[2]) for pt in boundary])

            self.road_edges[self.feature_counter] = roadedge
            self.feature_counter += 1

    def _set_neighborship(self, left_lane: Lane, right_lane: Lane):
        """
        add neighborship information on the corresponding feature of left_lane and right_lane
        left_lane is one of the left neighbors of right_lane, and vice versa

        create_lanecenter() should have been called on self_lane and neighbor_lane
        """

        neighbor1 = map_pb2.LaneNeighbor()
        neighbor1.feature_id = self.map_lane2featureid[right_lane]
        neighbor1.self_start_index = 0
        neighbor1.neighbor_start_index = 0
        neighbor1.self_end_index = len(left_lane.getShape3D()) - 1
        neighbor1.neighbor_end_index = len(right_lane.getShape3D()) - 1

        neighbor2 = map_pb2.LaneNeighbor()
        neighbor2.feature_id = self.map_lane2featureid[left_lane]
        neighbor2.self_start_index = 0
        neighbor2.neighbor_start_index = 0
        neighbor2.self_end_index = len(right_lane.getShape3D()) - 1
        neighbor2.neighbor_end_index = len(left_lane.getShape3D()) - 1

        left_lanecenter = self.lane_centers[self.map_lane2featureid[left_lane]]
        right_lanecenter = self.lane_centers[self.map_lane2featureid[right_lane]]
        left_lanecenter.right_neighbors.append(neighbor1)
        right_lanecenter.left_neighbors.append(neighbor2)

    def _set_connectionship(self, from_lane: Lane, to_lane: Lane):
        """
        add connectionship information on the corresponding features of from_lane and to_lane,
        to_lane is one of from_lane's exit lanes, from_lane is one of to_lane's entry lanes

        create_lanecenter() should have been called on from_lane and to_lane, otherwise do nothing
        """

        if from_lane in self.map_lane2featureid and to_lane in self.map_lane2featureid:
            from_feature_id = self.map_lane2featureid[from_lane]
            to_feature_id = self.map_lane2featureid[to_lane]
            self.lane_centers[to_feature_id].entry_lanes.append(from_feature_id)
            self.lane_centers[from_feature_id].exit_lanes.append(to_feature_id)

    def _find_neighboring_edges(self, starter_edge_ids: list[str], radius: float = 50):
        edge_set = set()

        for edge_id in starter_edge_ids:
            edge: Edge = self.sumonet.getEdge(edge_id)
            lane_sample: Lane = edge.getLane(edge.getLaneNumber() // 2)
            lane_shape: list[tuple] = lane_sample.getShape3D()

            edge_set1 = self.sumonet.getNeighboringEdges(
                lane_shape[0][0], lane_shape[0][1], r=radius, includeJunctions=True
            )
            edge_set2 = self.sumonet.getNeighboringEdges(
                lane_shape[-1][0], lane_shape[-1][1], r=radius, includeJunctions=True
            )
            edge_set.update([item[0] for item in edge_set1])
            edge_set.update([item[0] for item in edge_set2])

        return edge_set
    
    def plot_map(self, save_path: str = None, scenario_id: str = "test", plot_map: bool = False):
        # Plot all HDMap elements for visualization
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(12, 8))
        colors = ['r', 'g', 'b', 'c', 'm', 'y']
        
        # Plot lane centers
        for i, (feature_id, lane_center) in enumerate(self.lane_centers.items()):
            polyline = np.array([(p.x, p.y) for p in lane_center.polyline])
            plt.plot(polyline[:, 0], polyline[:, 1], color=colors[0], alpha=0.5, label='lane' if i==0 else '')
            
        # Plot road lines
        for i, (feature_id, road_line) in enumerate(self.road_lines.items()):
            polyline = np.array([(p.x, p.y) for p in road_line.polyline])
            plt.plot(polyline[:, 0], polyline[:, 1], color=colors[1], alpha=0.5, label='road_line' if i==0 else '')
            
        # Plot road edges    
        for i, (feature_id, road_edge) in enumerate(self.road_edges.items()):
            polyline = np.array([(p.x, p.y) for p in road_edge.polyline])
            plt.plot(polyline[:, 0], polyline[:, 1], color=colors[2], alpha=0.5, label='road_edge' if i==0 else '')
            
        plt.title(f'HDMap Elements Visualization - {scenario_id}')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        
        # Save plot with higher DPI for better quality
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()

    def convert_to_scenario(self, scenario_id: str = "test"):
        scenario = scenario_pb2.Scenario()
        scenario.scenario_id = scenario_id
        for id, lanecenter in self.lane_centers.items():
            feature = map_pb2.MapFeature()
            feature.id = id
            feature.lane.CopyFrom(lanecenter)
            scenario.map_features.append(feature)

        for id, roadedge in self.road_edges.items():
            feature = map_pb2.MapFeature()
            feature.id = id
            feature.road_edge.CopyFrom(roadedge)
            scenario.map_features.append(feature)

        for id, roadline in self.road_lines.items():
            feature = map_pb2.MapFeature()
            feature.id = id
            feature.road_line.CopyFrom(roadline)
            scenario.map_features.append(feature)

        for id, crosswalk in self.crosswalks.items():
            feature = map_pb2.MapFeature()
            feature.id = id
            feature.crosswalk.CopyFrom(crosswalk)
            scenario.map_features.append(feature)

        return scenario

    def save_scenario(self, scenario_id: str = "test", output_dir: str = None, plot_map: bool = False):
        """
        save the parsed data into a scenario file
        """

        scenario = self.convert_to_scenario(scenario_id=scenario_id)

        if output_dir is not None and plot_map:
            self.plot_map(save_path=f"{output_dir}/{scenario_id}_map.png", scenario_id=scenario_id)
        
        serialized_scenario = scenario.SerializeToString()
        if output_dir is not None:
            output_dir = Path(output_dir)
        else:
            output_dir = Path(f"map/{scenario_id}")

        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{scenario_id}_waymo"
        with open(output_path, "wb") as f:
            f.write(serialized_scenario)

if __name__ == "__main__":

    converter = SUMO2Waymo("terasim_demo/e7078100-3635-4e58-a497-64e5528f08e8/map.net.xml")
    converter.parse(have_road_edges=True, have_road_lines=True)
    scenario = converter.convert_to_scenario(scenario_id="test")
    with open("test_map.pb", "wb") as f:
        f.write(scenario.SerializeToString())
    # converter.save_scenario(scenario_id="test", output_dir="terasim_demo/e7078100-3635-4e58-a497-64e5528f08e8")