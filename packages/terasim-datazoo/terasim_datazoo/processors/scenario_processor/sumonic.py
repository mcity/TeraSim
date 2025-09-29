from typing import Union
import numpy as np
from pathlib import Path
import os
import subprocess

import xml.dom.minidom
import xml.etree.ElementTree as ET
import yaml

import numpy as np
import matplotlib.pyplot as plt

from shapely import LineString, Polygon, MultiPolygon, unary_union
from waymo_open_dataset.protos import scenario_pb2

from .sumonizer import Sumonizer
from .sumonic_tlsgen import SumonicTLSGenerator
from terasim_datazoo.core.sumo import *
from terasim_datazoo.core.waymo import *
from terasim_datazoo.utils import Direction, shape_str, polyline_length


class Sumonic(Sumonizer, SumonicTLSGenerator):
    def __init__(self, scenario, lanecenters: dict[int, LaneCenter], sumonize_config: dict = {}) -> None:

        self.scenario = scenario
        Sumonizer.__init__(self, scenario, lanecenters, **sumonize_config)
        SumonicTLSGenerator.__init__(self, scenario, self.features, self.nodes, self.edges)

    def plot_sumonic_map(
        self,
        base_dir: Union[str, Path, None] = None,
        filename: Union[str, None] = None,
        dpi=300,
        vis_nodeshape: bool = False,
    ) -> None:
        """
        Plots a map of the scenario using SUMO network data and saves it as an image file.
        Args:
            base_dir (Union[str, Path, None], optional): The base directory where the map image will be saved.
                If not provided, defaults to "map/{self.scenario.scenario_id}".
            filename (Union[str, None], optional): The name of the file to save the map image as.
                If not provided, defaults to "{self.scenario.scenario_id}-sumonic.png".
            dpi (int, optional): The resolution of the saved image in dots per inch. Defaults to 300.
            vis_nodeshape (bool, optional): Whether to visualize the shape of the nodes. Defaults to False.
        Returns:
            None
        """

        if not base_dir:
            base_dir = Path(f"map/{self.scenario.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir,exist_ok=True)
        if not filename:
            filename = f"{self.scenario.scenario_id}-sumonic.png"
        file_path: Path = base_dir / filename

        colors = ["b", "g", "r", "c", "m", "y", "k", "#50DC9F", "#8A0FC3", "#0F92C3"]
        plt.figure(figsize=(20, 20))

        # plot nodes
        for node in self.nodes.values():
            if not node.shape:
                plt.scatter(node.x, node.y, color="grey", marker="o")
            elif vis_nodeshape:
                xs = [pt.x for pt in node.shape]
                ys = [pt.y for pt in node.shape]
                plt.fill(xs, ys, "grey", alpha=0.2)

            # plot connections
            for i, conn in enumerate(node.connections):
                if not conn.shape:
                    continue
                color = "black"
                linestyle = {Direction.S: "-", Direction.L: "--", Direction.R: "-.", None: "-"}

                plt.plot(
                    [point.x for point in conn.shape],
                    [point.y for point in conn.shape],
                    c=color,
                    linewidth=1,
                    alpha=0.5,
                    linestyle=linestyle[conn.direction],
                )
                # plt.scatter(conn.shape[-1].x, conn.shape[-1].y, color="black", s=20)

            # plot traffic light
            # if node.tl_program:
            #     for i in range(len(node.tl_program.states)):
            #         tl_colors = {"r": "red", "G": "#9ACD32", "o": "black", "s": "purple", "g": "green"}
            #         state = node.tl_program.states[i]
            #         pos_x = node.tl_program.connections[i].shape[0].x
            #         pos_y = node.tl_program.connections[i].shape[0].y
            #         plt.scatter(pos_x, pos_y, color=tl_colors[state], marker="s")

        # plot edges and lanes
        for i, edge in self.edges.items():
            color = colors[i % len(colors)]
            for lane in edge.lanes:
                plt.plot(
                    [point.x for point in lane.shape],
                    [point.y for point in lane.shape],
                    linestyle="--" if lane.type == LaneTypeSUMO.BIKELANE else "-",
                    c=color,
                )
            mid: int = len(edge.lanes) // 2
            mid_lane_mid = edge.lanes[mid].shape[len(edge.lanes[mid].shape) // 2]
            plt.text(
                mid_lane_mid.x,
                mid_lane_mid.y,
                str(edge.id),
                fontsize=5,
            )

        plt.gca().set_aspect("equal", adjustable="box")
        plt.savefig(file_path, dpi=dpi)
        plt.clf()
        plt.close()
        print(f"[File Output] Plotted sumonic map: {file_path} ...")

    def save_SUMO_netfile(
        self,
        base_dir: Union[str, Path, None] = None,
        filename: Union[str, None] = None,
        default_tl_layout: str = "opposites",
        verbose: bool = False,
        message_log: bool = False,
    ) -> None:
        """
        Save the SUMO network file based on the current scenario.
        Args:
            base_dir (Union[str, Path, None], optional): The base directory where the files will be saved. Defaults to "map/{scenario_id}".
            filename (Union[str, None], optional): The name of the output network file. Defaults to "{scenario_id}.net.xml".
            default_tl_layout (str, optional): The default traffic light layout. Defaults to "opposites".
            verbose (bool, optional): If True, prints detailed error messages. Defaults to False.
            message_log (bool, optional): If True, saves the conversion log to a file. Defaults to False.
        Raises:
            Exception: If an error occurs during the netconvert process.
        Returns:
            None
        """

        if not base_dir:
            base_dir = Path(f"map/{self.scenario.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not filename:
            filename = f"{self.scenario.scenario_id}.net.xml"

        nodes_path = base_dir / f"{self.scenario.scenario_id}.nod.xml"
        edges_path = base_dir / f"{self.scenario.scenario_id}.edg.xml"
        connections_path = base_dir / f"{self.scenario.scenario_id}.con.xml"
        tls_path = base_dir / f"{self.scenario.scenario_id}.tll.xml"
        net_path = base_dir / filename

        nodes_root = ET.Element("nodes")
        edges_root = ET.Element("edges")
        connections_root = ET.Element("connections")
        tls_root = ET.Element("tlLogics")

        # node, connections and prohibitions
        for node in self.nodes.values():

            # node
            node_element = ET.SubElement(nodes_root, "node")
            node_element.set("id", f"node-{node.id}")
            node_element.set("type", node.type)
            node_element.set("x", str(node.x))
            node_element.set("y", str(node.y))
            if node.z:
                node_element.set("z", str(node.z))
            if node.shape:
                node_element.set("shape", shape_str(node.shape))
            if not node.keep_clear:
                node_element.set("keepClear", "false")

            # connections
            for conn in node.connections:
                connection_element = ET.SubElement(connections_root, "connection")
                connection_element.set("from", str(conn.from_edge))
                connection_element.set("to", str(conn.to_edge))
                connection_element.set("fromLane", str(conn.from_lane))
                connection_element.set("toLane", str(conn.to_lane))
                if conn.speed:
                    connection_element.set("speed", str(conn.speed))
                if conn.shape and conn.apply_shape:
                    connection_element.set("shape", shape_str(conn.shape))

                if conn.can_pass:
                    connection_element.set("pass", "true")
                if not node.keep_clear:
                    connection_element.set("keepClear", "false")

            # prohibitions
            for prohibition in node.prohibitions:
                prohibition_element = ET.SubElement(connections_root, "prohibition")
                prohibition_element.set(
                    "prohibitor", f"{prohibition.prohibitor_from}->{prohibition.prohibitor_to}"
                )
                prohibition_element.set(
                    "prohibited", f"{prohibition.prohibited_from}->{prohibition.prohibited_to}"
                )

            # walking areas
            for walkingarea in node.walking_areas:
                walkingarea_element = ET.SubElement(connections_root, "walkingArea")
                walkingarea_element.set("node", f"node-{node.id}")
                walkingarea_element.set("edges", ' '.join(map(str,walkingarea.edges)))
                if walkingarea.shape:
                    walkingarea_element.set("shape", shape_str(walkingarea.shape))
            
            # crossings
            for i, crossing in enumerate(node.crossings):
                crossing_element = ET.SubElement(connections_root, "crossing")
                crossing_element.set("node", f"node-{node.id}")
                crossing_element.set("edges", ' '.join(map(str,crossing.edges)))
                crossing_element.set("priority", "true")
                crossing_element.set("width", str(crossing.width))
                crossing_element.set("linkIndex", str(i+100))
                crossing_element.set("linkIndex2", str(i+100))
                # crossing_element.set("shape", shape_str(crossing.shape))

            # traffic light
            if node.tl_program:
                tl_element = ET.SubElement(tls_root, "tlLogic")
                tl_element.set("id", f"node-{node.id}")
                tl_element.set("type", "static")
                tl_element.set("programID", "0")
                tl_element.set("offset", "0")
                for phase in node.tl_program:
                    phase_element = ET.SubElement(tl_element, "phase")
                    phase_element.set("duration", str(phase.duration))
                    phase_element.set("state", "".join([st.value for st in phase.states]))
                    for i, conn in enumerate(phase.connections):
                        from_edge, from_lane, to_edge, to_lane = conn
                        conn_element = ET.SubElement(tls_root, "connection")
                        conn_element.set("from", str(from_edge))
                        conn_element.set("to", str(to_edge))
                        conn_element.set("fromLane", str(from_lane))
                        conn_element.set("toLane", str(to_lane))
                        conn_element.set("tl", f"node-{node.id}")
                        conn_element.set("linkIndex", str(i))

        # edge file
        for _, edge in self.edges.items():
            edge_element = ET.SubElement(edges_root, "edge")
            edge_element.set("id", str(edge.id))
            edge_element.set("from", f"node-{edge.from_node_id}")
            edge_element.set("to", f"node-{edge.to_node_id}")
            edge_element.set("numLanes", str(len(edge.lanes)))
            edge_element.set("spreadType", "center")

            for i, lane in enumerate(edge.lanes):
                lane_element = ET.SubElement(edge_element, "lane")
                lane_element.set("index", str(i))
                lane_element.set("speed", str(lane.speed))
                lane_element.set("width", str(lane.width))
                lane_element.set("shape", shape_str(lane.shape))
                if lane.type == LaneTypeSUMO.NORMAL:
                    lane_element.set("disallow", "pedestrian")
                if lane.type == LaneTypeSUMO.BIKE:
                    lane_element.set("allow", "bicycle")
                elif lane.type == LaneTypeSUMO.SIDEWALK:
                    lane_element.set("allow", "pedestrian")

        edges_xml = xml.dom.minidom.parseString(ET.tostring(edges_root)).toprettyxml()
        nodes_xml = xml.dom.minidom.parseString(ET.tostring(nodes_root)).toprettyxml()
        connections_xml = xml.dom.minidom.parseString(ET.tostring(connections_root)).toprettyxml()
        tls_xml = xml.dom.minidom.parseString(ET.tostring(tls_root)).toprettyxml()
        with open(edges_path, "w") as f:
            f.write(edges_xml)
        with open(nodes_path, "w") as f:
            f.write(nodes_xml)
        with open(connections_path, "w") as f:
            f.write(connections_xml)
        with open(tls_path, "w") as f:
            f.write(tls_xml)

        # Generate netfile with netconvert
        commands = [
            "netconvert",
            # input & output
            f"--node-files={nodes_path}",
            f"--edge-files={edges_path}",
            f"--connection-files={connections_path}",
            f"--tllogic-files={tls_path}",
            f"--output-file={net_path}",
            # report
            "--ignore-errors=false",
            "--ignore-errors.connections=true",
            # junctions
            "--no-turnarounds=true",
            "--no-left-connections=true",
            "--junctions.join-same=true",
            "--junctions.limit-turn-speed=5",
            "--junctions.limit-turn-speed.warn.turn=0.1",
            "--default.junctions.radius=20",
            "--junctions.internal-link-detail=20",
            # processing
            "--plain.extend-edge-shape=false",
            "--offset.disable-normalization=true",
            "--geometry.avoid-overlap=false",
            "--geometry.max-grade.fix=false",
            "--opposites.guess=true",
            "--fringe.guess=true",
            # pedestrian
            # "--sidewalks.guess=true",
            "--walkingareas=true",
            # tls
            f"--tls.layout={default_tl_layout}",
        ]
        if message_log:
            commands.append(f"--message-log={base_dir}/conversion_log.txt")
        process = subprocess.Popen(commands, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()
        stdout = stdout.decode("utf-8")
        stderr = stderr.decode("utf-8")
        if verbose:
            print(stderr)

        success = True
        if "Error" in stderr:
            success = False
            if not verbose:
                print(stderr)

        os.remove(nodes_path)
        os.remove(edges_path)
        os.remove(connections_path)
        os.remove(tls_path)

        if not success:
            raise Exception("Error occurred in netconvert")
        print(f"[File Output] Generated SUMO map file: {net_path} ...")
        return stderr

    def save_SUMO_waymo_config_file(self, base_dir: Union[str, Path, None]= None, file_name: Union[str, None] = None) -> None:
        """
        Save a .yaml configuration file that contains necessary mapping information between the SUMO file and the original Waymo data. This is used for NDE simulation.

        Args:
        - base_dir: the directory of the saved yaml file
        - file_name: the name of the file (without .yaml suffix), default to be the scenario id

        The file contains the following:
        - SUMO_waymo_dest_mapping (a dict)
        - tl_groups (a list of strings)
        - SUMO_waymo_map_offset (a tuple of float)
        """

        if not base_dir:
            base_dir = Path(f"map/{self.scenario.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not file_name:
            file_name = self.scenario.scenario_id

        config_dest_mapping = {}

        config_tl_groups = [
            "node-" + str(node.id)
            for node in self.nodes.values()
            if node.type in ["traffic_light", "traffic_light_right_on_red"]
        ]

        map_offset = [0.0, 0.0]

        config = {
            "SUMO_waymo_dest_mapping": config_dest_mapping,
            "tl_groups": config_tl_groups,
            "SUMO_waymo_map_offset": map_offset,
        }

        config_path = base_dir / f"{file_name}.yaml"

        with open(config_path, "w") as f:
            yaml.dump(config, f)
        print(f"[File Output] Saved SUMO waymo config file: {config_path} ...")

    def analyze_drivable_area(
        self, start_features: list[int], end_features: list[int]
    ) -> Union[Polygon, MultiPolygon]:
        """
        Find the drivable area, given a list of start_features and end_features.
        plot the area, save a figure, and return the area polygon
        """

        print("Anayzing drivable area...")

        def _find_edgeid_by_featureid(feature_id: int, side: str) -> int:
            while feature_id in self.ft_del_mapping:
                feature_id = (
                    self.ft_del_mapping[feature_id][0]
                    if side == "start"
                    else self.ft_del_mapping[feature_id][1]
                )
            return self.features[feature_id].to_SUMO_edge

        def _path_len_by_id(path: list[int]) -> float:
            len_list = [polyline_length(self.edges[id].lanes[0].shape) for id in path]
            return sum(len_list)

        ## I. convert starting and ending point
        start_edges: set[int] = {_find_edgeid_by_featureid(id, "start") for id in start_features}
        end_edges: set[int] = {_find_edgeid_by_featureid(id, "end") for id in end_features}
        print(f"start_edges: {start_edges}")
        print(f"end_edges: {end_edges}")

        # II. collect for a list of all possible paths, and then edegset
        edgeset: set[int] = set()
        for input_edge in start_edges:
            for output_edge in end_edges:

                result_path: list[list[int]] = []
                init_len = polyline_length(self.edges[input_edge].lanes[0].shape)
                self._search_path(np.inf, output_edge, [input_edge], init_len, result_path)
                result_path.sort(key=_path_len_by_id, reverse=False)
                shortest_length: float = _path_len_by_id(result_path[0])
                for path in result_path:
                    if _path_len_by_id(path) <= shortest_length + 10:  # certain buffer
                        edgeset.update(path)
                    else:
                        break

        if not edgeset:
            print(f"Warning: no path between {start_features} and {end_features}, return None")
            return None

        # III. convert them to polygon
        all_edge_polygons = [create_edge_polygon(self.edges[edge_id]) for edge_id in edgeset]
        drivable_area_polygon = unary_union(all_edge_polygons).buffer(
            self._EDGE_POLYGON_BUFFER_DISTANCE + 0.01
        )
        # plot_polygon(drivable_area_polygon, add_points=False)

        return drivable_area_polygon

    def _search_path(
        self,
        shortest_len: float,
        output_edge: int,
        path: list[int],
        curr_len: float,
        result_path: list[list[int]],
    ):
        """
        backtracking on searching the path from start_edge to one of the end_edges
        currently on curr_edge, through "path".
        if a complete path is found, add edges in the path to edgeset.

        All params are in form of edge ids

        i (<=20) : number of paths found, this is to avoid too long time on backtracking search
        return the number of paths found
        """

        # END CONDITION
        if path[-1] == output_edge:
            result_path.append(path[:])
            return curr_len

        # PRUNE (PROMISING)
        # condition1: too long
        if curr_len >= shortest_len + 10:  # there is a certain buffer value
            return shortest_len
        # condition2: we have found 20 paths
        if len(result_path) == 30:
            return shortest_len

        # RECURSION
        for edge_id in self.nodes[self.edges[path[-1]].to_node_id].outgoing_SUMO_edges:
            # if there have been 20 valid paths found, don't search anymore
            # recursively search
            if edge_id not in path:
                path.append(edge_id)
                next_len = curr_len + polyline_length(self.edges[edge_id].lanes[0].shape)
                shortest_len = self._search_path(shortest_len, output_edge, path, next_len, result_path)
                path.pop()

        return shortest_len
