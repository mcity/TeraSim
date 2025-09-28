from typing import Union
from collections import deque
import copy
import numpy as np
import shapely
from shapely import Point, unary_union

from terasim_datazoo.core.sumo import *
from terasim_datazoo.core.waymo import *
from terasim_datazoo.utils import *


class Sumonizer:
    def __init__(
        self,
        scenario,
        lanecenters: dict[int, LaneCenter],
        default_tl_type="traffic_light_right_on_red",
        default_node_type="priority",
        edge_polygon_buffer_distance: float = 0.1,
        custom_nodeshape: bool = True,
        consider_roadedges_in_nodeshape: bool = False,
        custom_lanewidth: bool = True,
        default_lanewidth: float = 3.8,
        add_sidewalk: bool = False,
        # many configs
    ) -> None:

        self._LENGTH_DIFFERENCE_THRESHOLD = 3
        """
        (meters) Given two lanes feature1 and feature2, how much longer must feature1 be than feature2 in order to be considered longer than feature2?
        """

        self._INDEX_SNAP_THRESHOLD = 4
        """
        (index) given a truncation index, how close must it to either start point or end point in order to be ignored? This is to avoid infinite loop.
        """

        self._EDGE_POLYGON_BUFFER_DISTANCE = edge_polygon_buffer_distance
        """(meters)"""

        self._CONNECTION_DISCARD_LENGTH = 85
        """(meters)"""

        self._POINT_CLOSE_THRESHOLD = 5
        """
        (meters) given two points point1 & point2, how much less must the distance between them in order to be considered close?
        """

        # configurations
        self._TL_TYPE = default_tl_type
        self._DEFAULT_NODE_TYPE = default_node_type
        self._custom_nodeshape = custom_nodeshape
        self._consider_roadedge_in_nodeshape = consider_roadedges_in_nodeshape
        self._custom_lanewidth = custom_lanewidth
        self._default_lanewidth = default_lanewidth

        # data structures
        self.features: dict[int, LaneCenter] = copy.deepcopy(lanecenters)
        """
        A dict of all features, keyed with feature ids
        """

        self.nodes: dict[int, Node] = {}
        """
        A list of SUMO nodes. Nodes data are to be within .nod file.
        """

        self.edges: dict[int, Edge] = {}
        """
        A dict of SUMO edges, keyed with edge ids.
        Edges data are to be within .edg file.
        """

        self._node_counter: int = 0
        self._edge_counter: int = 0
        self._new_ft_counter: int = -1

        # below: for debugging
        self.ft_del_mapping: dict[int, list[int]] = {}
        """
        In truncation, a feature is splitted to two new features.
        This dictionary records the mapping between the old one and the two new one,
        for the ease of debugging.
        """
        self.ok: bool = True

        # I. make truncations on features
        self.truncate_features()

        # II. combine features into edges
        self.generate_edges()

        # III. grouping edges
        self.consolidate_edges_into_junctions()

        # V. create nodes and connections for all remaining edges
        for edge in self.edges.values():
            self.link_edge_to_junctions(edge)

        # VI. make minor corrections
        self.remove_selfloop_edges()

        self.concatenate_edges()

        if add_sidewalk:
            self.add_sidewalks()

            # add crossings
            for feature in scenario.map_features:
                feature_data_type = feature.WhichOneof("feature_data")
                if feature_data_type == "crosswalk":
                    self.add_crossing(feature.crosswalk.polygon)

    """
    ################################## Utility funcitons ##################################
    """

    def _create_node(self, point: Pt, type: str = "", shape: list[Pt] = None, keep_clear: bool = True) -> int:
        """
        Create a SUMO node and store it in the global variable "nodes".

        Args:
            point (Pt): The point representing the node's coordinates.
            type (str, optional): The type of the node. Defaults to an empty string, which will be replaced by the default node type.
            shape (list[Pt], optional): The shape of the node as a list of points. Defaults to None.
            keep_clear (bool, optional): A flag indicating whether to keep the node clear. Defaults to True.

        Returns:
            int: The ID of the created node.
        """

        if not type:
            type = self._DEFAULT_NODE_TYPE

        self.nodes[self._node_counter] = Node(
            self._node_counter, point.x, point.y, point.z, type, shape, keep_clear
        )
        self._node_counter += 1

        return self._node_counter - 1

    def _create_edge(self, all_lanes: list[Lane], from_node_id: int = None, to_node_id: int = None) -> int:
        """
        Create a SUMO edge and store it in the global variable "edges".

        Args:
            all_lanes (list[Lane]): A list of Lane objects that belong to the edge.
            from_node_id (int, optional): The ID of the starting node of the edge. Defaults to None.
            to_node_id (int, optional): The ID of the ending node of the edge. Defaults to None.

        Returns:
            int: The ID of the created edge.
        """

        self.edges[self._edge_counter] = Edge(
            self._edge_counter, all_lanes, from_node_id=from_node_id, to_node_id=to_node_id
        )
        self._edge_counter += 1

        return self._edge_counter - 1

    def _create_feature(self, lane, needs_stop: bool = False, tl_state_record: list = None) -> int:
        """
        Creates a new feature for the given lane and returns its unique identifier.

        Args:
            lane: The lane object for which the feature is being created.
            needs_stop (bool, optional): Indicates whether the feature requires a stop. Defaults to False.
            tl_state_record (list, optional): Traffic light state record associated with the feature. Defaults to None.

        Returns:
            int: The unique identifier of the newly created feature.
        """

        id = self._new_ft_counter
        self._new_ft_counter -= 1

        self.features[id] = LaneCenter(id, lane, needs_stop, tl_state_record)
        return id

    def _is_connection_group(self, elements: list[int]) -> bool:
        """
        Determines if a group of elements represents a connection group.

        A connection group is defined as:
        - A group containing more than one edge.
        - A group where any lane in the first edge has diverge or merge lanes.

        Args:
            elements (list[int]): A list of element indices.

        Returns:
            bool: True if the group is a connection group, False otherwise.
        """

        # if the group contains more than one edge
        if len(elements) > 1:
            return True
        # or if there is any diverge/merging lane pairs in the group
        for lane in self.edges[elements[0]].lanes:
            if (
                self.features[lane.to_WAYMOfeature].lane.diverge_lanes
                or self.features[lane.to_WAYMOfeature].lane.merge_lanes
            ):
                return True
        # [deleted] or there is a stop sign in the group
        # for element in elements:
        #     for lane in self.edges[element].lanes:
        #         if self.features[lane.to_WAYMOfeature].needs_stop:
        #             return True
        # [deleted] or there is a traffic light record in the group
        # if any(tls != WaymonicTLS.ABSENT for tls in self.features[lane.to_WAYMOfeature].record_tls):
        #     return True
        return False

    def _get_connected_edges_of_edge_group(
        self, edge_group: list[int], side: str, self_excluded: bool = True
    ) -> set[int]:
        """
        Retrieve the connected edges of a given edge group based on the specified side.

        Args:
            edge_group (list[int]): A list of edge IDs representing the edge group.
            side (str): The side to consider for connections, either "entry" or "exit".
            self_excluded (bool, optional): If True, exclude edges within the edge group from the result. Defaults to True.

        Returns:
            set[int]: A set of connected edge IDs.
        """

        connected_edges: set[int] = set()
        for edge_id in edge_group:
            if edge_id not in self.edges:
                continue
            for lane in self.edges[edge_id].lanes:
                connected_lanes = (
                    self.features[lane.to_WAYMOfeature].lane.entry_lanes
                    if side == "entry"
                    else self.features[lane.to_WAYMOfeature].lane.exit_lanes
                )
                for connected_fid in connected_lanes:
                    connected_edge_id = self.features[connected_fid].to_SUMO_edge
                    if (
                        (connected_edge_id not in edge_group) or (not self_excluded)
                    ) and connected_edge_id in self.edges:
                        connected_edges.add(connected_edge_id)

        return connected_edges

    def _create_edges_polygon(
        self, edge_group: list[int], allow_multipolygon: bool = True
    ) -> Union[Polygon, MultiPolygon, None]:
        """
        Create a polygon or multipolygon from a group of edges.

        Args:
            edge_group (list[int]): A list of edge IDs to be included in the polygon.
            allow_multipolygon (bool, optional): If True, allows the creation of a MultiPolygon
                                                 if the edges form disjoint shapes. Defaults to True.

        Returns:
            Union[Polygon, MultiPolygon, None]: A Polygon or MultiPolygon object if the edges form a valid shape, otherwise None.
        """

        return create_edges_polygon(
            self.features,
            (self.edges[edge_id] for edge_id in edge_group),
            use_boundary=self._consider_roadedge_in_nodeshape,
            buffer_distance=self._EDGE_POLYGON_BUFFER_DISTANCE,
            allow_multipolygon=allow_multipolygon,
        )

    def _get_ways_from_node(
        self, node: Node, type: str = "all"
    ) -> Union[list[list[Edge]], list[tuple[list[Edge, list[Edge]]]]]:
        """
        Retrieves ways (paths) from a given node based on the specified type.

        Args:
            node (Node): The node from which to retrieve the ways.
            type (str, optional): The type of ways to retrieve. Can be 'incoming', 'outgoing', or 'all'. Defaults to 'all'.

        Returns:
            Union[list[list[Edge]], list[tuple[list[Edge], list[Edge]]]]:
                - If type is 'incoming' or 'outgoing', returns a list of lists of Edge objects.
                - If type is 'all', returns a list of tuples, where each tuple contains two lists of Edge objects:
                  one for incoming edges and one for outgoing edges.
        """

        edge_ids: list[int] = []
        edge_vectors: list[np.ndarray] = []
        if type in ["incoming", "all"]:
            for edge_id in node.incoming_SUMO_edges:
                edge_ids.append(edge_id)
                edge_vectors.append(self.edges[edge_id].get_estimation_vector("entry"))
        if type in ["outgoing", "all"]:
            for edge_id in node.outgoing_SUMO_edges:
                edge_ids.append(edge_id)
                edge_vectors.append(-self.edges[edge_id].get_estimation_vector("exit"))
        groups = group_vectors_by_angles(edge_vectors)

        if type in ["incoming", "outgoing"]:
            ways: list[list[Edge]] = [[self.edges[edge_ids[i]] for i in group] for group in groups]
        else:
            ways: list[tuple[list[Edge], list[Edge]]] = []
            for group in groups:
                ways.append(
                    (
                        [self.edges[edge_ids[i]] for i in group if edge_ids[i] in node.incoming_SUMO_edges],
                        [self.edges[edge_ids[i]] for i in group if edge_ids[i] in node.outgoing_SUMO_edges],
                    )
                )

        return ways

    """
    ################################## Truncate features ##################################
    """

    def truncate_features(self) -> None:
        """
        Repeatedly make truncations on features, until exhausted.

        This method sorts the features in descending order based on their polyline lengths.
        It then processes each feature by truncating it and re-queues any resulting features
        for further truncation. The process continues until no more features can be truncated
        or a safeguard limit is reached to prevent infinite loops.

        The method also removes features that were truncated from the features dictionary.

        Attributes:
            self.features (dict): A dictionary where keys are feature IDs and values are feature objects.
            self.ft_del_mapping (set): A set of feature IDs that have been truncated and should be deleted.
            self._new_ft_counter (int): A counter used to detect potential infinite loops.
        """

        def sort_features_by_length(item) -> float:
            id, feature = item
            return polyline_length(feature.lane.polyline)

        self.features = dict(sorted(self.features.items(), key=sort_features_by_length, reverse=True))
        process_count = {f_id: 10 for f_id in self.features.keys()}

        queue = deque(self.features.keys())
        while queue:
            id = queue.popleft()
            result: list[int] = self._truncate_feature(self.features[id], process_count)
            queue.extend(result)
            # force quit if infinite loop
            if self._new_ft_counter < -800:
                break

        # delete features that were truncated
        self.features = {
            id: feature for id, feature in self.features.items() if id not in self.ft_del_mapping
        }
        self.features = dict(sorted(self.features.items(), key=sort_features_by_length, reverse=True))

    def _truncate_feature(self, feature: LaneCenter, process_count: list[int, int]) -> list[int]:
        """
        Make truncation on a feature (if necessary).

        This method truncates a given feature (lane center) if certain conditions are met.
        If the feature is truncated, it is marked as deleted and two new features are created
        to represent the truncated segments. The method also updates the relationships and
        boundaries of the new features and their neighbors.

        Args:
            feature (LaneCenter): The lane center feature to be truncated.
            process_count (list[int, int]): A list to keep track of the processing count for each feature.

        Returns:
            list[int]: A list of feature IDs that need to be pushed to the queue for inspection again.
        """

        # ignore deleted features
        if feature.id in self.ft_del_mapping:
            return []

        result: list[int] = []

        for neighbor in feature.lane.left_neighbors + feature.lane.right_neighbors:
            # ignore deleted neighbors
            if neighbor.feature_id in self.ft_del_mapping:
                continue
            nbr_type = real_neighbor_type(
                feature.lane.polyline,
                self.features[neighbor.feature_id].lane.polyline,
                POINT_CLOSE_THRESHOLD=self._POINT_CLOSE_THRESHOLD,
                LENGTH_DIFFERENCE_THRESHOLD=self._LENGTH_DIFFERENCE_THRESHOLD,
            )
            if nbr_type == "complete":
                continue

            # if type is not "complete", should ensure that only consider neighbors that are shorter than self
            if (
                polyline_length(self.features[neighbor.feature_id].lane.polyline)
                > polyline_length(feature.lane.polyline) - self._LENGTH_DIFFERENCE_THRESHOLD
            ):
                continue

            if nbr_type in ["side-start", "side-end"]:

                # I. find truncation and create two new features
                # find the index to truncate at
                reference_point = (
                    self.features[neighbor.feature_id].lane.polyline[-1]
                    if nbr_type == "side-start"
                    else self.features[neighbor.feature_id].lane.polyline[0]
                )
                truncate_index = find_polyline_nearest_point(feature.lane.polyline, reference_point)
                # if the truncation index is close to either start point or end point, ignore it
                if (
                    truncate_index < self._INDEX_SNAP_THRESHOLD
                    or truncate_index > len(feature.lane.polyline) - self._INDEX_SNAP_THRESHOLD
                ):
                    continue

                # create start lane
                start_lane = WaymoLane(
                    feature.lane.speed_limit_mph,
                    feature.lane.type,
                    feature.lane.polyline[: truncate_index + 1],
                    feature.lane.interpolating,
                    feature.lane.entry_lanes,
                    [],
                    feature.lane.left_neighbors,  # info loss
                    feature.lane.right_neighbors,  # info loss
                    [],
                    [],
                )
                start_feature_idx = self._create_feature(
                    start_lane, needs_stop=feature.needs_stop, tl_state_record=feature.record_tls[:]
                )
                process_count[start_feature_idx] = 10
                result.append(start_feature_idx)

                # create end lane
                end_lane = WaymoLane(
                    feature.lane.speed_limit_mph,
                    feature.lane.type,
                    feature.lane.polyline[truncate_index:],
                    feature.lane.interpolating,
                    [],
                    feature.lane.exit_lanes,
                    feature.lane.left_neighbors,  # info loss
                    feature.lane.right_neighbors,  # info loss
                    [],
                    [],
                )
                end_feature_idx = self._create_feature(
                    end_lane, needs_stop=feature.needs_stop, tl_state_record=feature.record_tls[:]
                )
                process_count[end_feature_idx] = 10
                result.append(end_feature_idx)

                # II. correct some information of the two new features
                # exit & entry lanes
                self.features[start_feature_idx].lane.exit_lanes.append(end_feature_idx)
                self.features[end_feature_idx].lane.entry_lanes.append(start_feature_idx)

                # bifurcated & merged lanes
                self.features[start_feature_idx].lane.diverge_lanes = feature.lane.diverge_lanes
                self.features[end_feature_idx].lane.merge_lanes = feature.lane.merge_lanes

                # boundaries
                for boundary in feature.lane.left_boundaries + feature.lane.right_boundaries:
                    if boundary.lane_end_index <= truncate_index:
                        self.features[start_feature_idx].lane.left_boundaries.append(
                            Boundary(
                                boundary.lane_start_index,
                                boundary.lane_end_index,
                                boundary.type,
                                boundary.feature_id,
                                boundary.polyline,
                            )
                        )
                    elif boundary.lane_start_index >= truncate_index:
                        self.features[end_feature_idx].lane.left_boundaries.append(
                            Boundary(
                                boundary.lane_start_index - truncate_index,
                                boundary.lane_end_index - truncate_index,
                                boundary.type,
                                boundary.feature_id,
                                boundary.polyline,
                            )
                        )
                    else:
                        boundary_truncate_idx = find_polyline_nearest_point(
                            boundary.polyline, feature.lane.polyline[truncate_index]
                        )
                        if boundary_truncate_idx >= self._INDEX_SNAP_THRESHOLD:
                            self.features[start_feature_idx].lane.left_boundaries.append(
                                Boundary(
                                    boundary.lane_start_index,
                                    truncate_index,
                                    boundary.type,
                                    boundary.feature_id,
                                    boundary.polyline[: boundary_truncate_idx + 1],
                                )
                            )
                        if len(boundary.polyline) - boundary_truncate_idx >= self._INDEX_SNAP_THRESHOLD:
                            self.features[end_feature_idx].lane.right_boundaries.append(
                                Boundary(
                                    0,
                                    boundary.lane_end_index - truncate_index,
                                    boundary.type,
                                    boundary.feature_id,
                                    boundary.polyline[boundary_truncate_idx:],
                                )
                            )

                # III. correct some information in surrounding lanes

                # left neighbors
                for inner_neighbor in feature.lane.left_neighbors:
                    self.features[inner_neighbor.feature_id].lane.right_neighbors = [
                        nb
                        for nb in self.features[inner_neighbor.feature_id].lane.right_neighbors
                        if nb.feature_id != feature.id
                    ]  # delete old feature
                    # very simple: both new features are counted as neighbors
                    new_nb_start = Neighbor(start_feature_idx)
                    new_nb_end = Neighbor(end_feature_idx)
                    self.features[inner_neighbor.feature_id].lane.right_neighbors.append(new_nb_start)
                    self.features[inner_neighbor.feature_id].lane.right_neighbors.append(new_nb_end)

                # right neighbors
                for inner_neighbor in feature.lane.right_neighbors:
                    self.features[inner_neighbor.feature_id].lane.left_neighbors = [
                        nb
                        for nb in self.features[inner_neighbor.feature_id].lane.left_neighbors
                        if nb.feature_id != feature.id
                    ]  # delete old feature
                    # very simple: both new features are counted as neighbors
                    new_nb_start = Neighbor(start_feature_idx)
                    new_nb_end = Neighbor(end_feature_idx)
                    self.features[inner_neighbor.feature_id].lane.left_neighbors.append(new_nb_start)
                    self.features[inner_neighbor.feature_id].lane.left_neighbors.append(new_nb_end)

                # bifuracted lanes
                for diverge_lane in feature.lane.diverge_lanes:
                    self.features[diverge_lane].lane.diverge_lanes.remove(feature.id)
                    self.features[diverge_lane].lane.diverge_lanes.add(start_feature_idx)

                # merged lanes
                for merge_lane in feature.lane.merge_lanes:
                    self.features[merge_lane].lane.merge_lanes.remove(feature.id)
                    self.features[merge_lane].lane.merge_lanes.add(end_feature_idx)

                # 3.2 connected entry lanes and exit lanes
                # entry lanes
                for entry_lane_id in feature.lane.entry_lanes:
                    self.features[entry_lane_id].lane.exit_lanes = [
                        id for id in self.features[entry_lane_id].lane.exit_lanes if id != feature.id
                    ]
                    self.features[entry_lane_id].lane.exit_lanes.append(start_feature_idx)

                # exit lanes
                for exit_lane_id in feature.lane.exit_lanes:
                    self.features[exit_lane_id].lane.entry_lanes = [
                        id for id in self.features[exit_lane_id].lane.entry_lanes if id != feature.id
                    ]
                    self.features[exit_lane_id].lane.entry_lanes.append(end_feature_idx)

                # IV. mark this feature as "to be deleted"
                self.ft_del_mapping[feature.id] = [start_feature_idx, end_feature_idx]
                break

        if process_count[feature.id] > 0:
            result.append(feature.id)

        process_count[feature.id] -= 1
        return result

    """
    ################################## Generate edges ##################################
    """

    def generate_edges(self) -> None:
        """
        Generates edges for the scenario processor.

        This method iterates over the features in the scenario processor and generates edges for each feature.
        It keeps track of the generated edges using the `is_edge_generated` dictionary.

        Attributes:
            is_edge_generated (dict[int, bool]): A dictionary that tracks whether an edge has been generated for each feature ID.

        Returns:
            None
        """

        is_edge_generated: dict[int, bool] = {f_id: False for f_id in self.features.keys()}
        for f_id, feature in self.features.items():
            if not is_edge_generated[f_id]:
                result = self._generate_edge(feature)
                for f_id in result:
                    is_edge_generated[f_id] = True

    def _generate_edge(self, feature: LaneCenter) -> list[int]:
        """
        Given the feature, find all its parallel features using _find_parallel_lanes(),
        then create an edge with those features as its lanes
        (temporaily without from-node and to-node information),

        returns this list of f_ids that are are turned into an edge
        """

        # find all the parallel lanes
        all_parallel_lanes = self._find_parallel_lanes(feature)

        all_lanes: list[Lane] = []
        for i, f_id in enumerate(all_parallel_lanes):
            speed = mph_to_ms(self.features[f_id].lane.speed_limit_mph)
            shape = self.features[f_id].lane.polyline
            if self._custom_lanewidth:
                width = self._determine_lane_width(self.features[f_id])
            else:
                width = self._default_lanewidth
            if self.features[f_id].lane.type == LaneType.BIKELANE.value:
                type = LaneTypeSUMO.BIKE
            else:
                type = LaneTypeSUMO.NORMAL
            lane = Lane(speed=speed, shape=shape, width=min(width, 4.0), type=type)
            all_lanes.append(lane)

        edge_id = self._create_edge(all_lanes)
        # record mapping information: lane <-> feature
        for lane_idx, feature_id in enumerate(all_parallel_lanes):  # from right to left
            self.features[feature_id].to_SUMO_edge = edge_id
            self.features[feature_id].to_SUMO_lane = lane_idx
            self.edges[edge_id].lanes[lane_idx].to_WAYMOfeature = feature_id

        return all_parallel_lanes

    def _find_parallel_lanes(self, feature: LaneCenter) -> list[int]:
        """
        Finds all parallel lanes to the given feature.

        This method identifies all lanes that are parallel to the specified lane
        center feature by traversing both right and left neighbors.

        Args:
            feature (LaneCenter): The lane center feature for which to find parallel lanes.

        Returns:
            list[int]: A list of lane IDs that are parallel to the given feature.
        """

        parallel_list = [feature.id]
        # right neighbors
        while True:
            neighbor_list = self.features[parallel_list[0]].lane.right_neighbors
            selected_lane_id = self._find_parallel_feature(self.features[parallel_list[0]], neighbor_list)
            if selected_lane_id != None and selected_lane_id not in parallel_list:
                parallel_list.insert(0, selected_lane_id)
            else:
                break

        # left neighbors
        while True:
            neighbor_list = self.features[parallel_list[-1]].lane.left_neighbors
            selected_lane_id = self._find_parallel_feature(self.features[parallel_list[-1]], neighbor_list)
            if selected_lane_id != None and selected_lane_id not in parallel_list:
                parallel_list.append(selected_lane_id)
            else:
                break

        return parallel_list

    def _find_parallel_feature(self, feature: LaneCenter, neighbor_list: list[Neighbor]) -> Union[int, None]:
        """
        Finds a parallel feature to the given feature from a list of neighbors.

        This method identifies potential parallel features by comparing the polyline
        of the given feature with the polylines of its neighbors. If multiple potential
        parallel features are found, it selects the one with the minimum end-to-end distance.

        Args:
            feature (LaneCenter): The feature for which to find a parallel feature.
            neighbor_list (list[Neighbor]): A list of neighboring features to consider.

        Returns:
            Union[int, None]: The feature ID of the parallel feature if found, otherwise None.
        """
        potential_parallel_list: list[int] = [
            neighbor.feature_id
            for neighbor in neighbor_list
            if real_neighbor_type(
                feature.lane.polyline,
                self.features[neighbor.feature_id].lane.polyline,
                POINT_CLOSE_THRESHOLD=self._POINT_CLOSE_THRESHOLD,
                LENGTH_DIFFERENCE_THRESHOLD=self._LENGTH_DIFFERENCE_THRESHOLD,
            )
            == "complete"
        ]

        if len(potential_parallel_list) == 0:
            return None
        elif len(potential_parallel_list) == 1:
            return potential_parallel_list[0]
        else:

            def calculate_end_to_end_distance(potential_feature: LaneCenter) -> float:
                return distance_between_points(
                    feature.lane.polyline[0], potential_feature.lane.polyline[0]
                ) + distance_between_points(feature.lane.polyline[-1], potential_feature.lane.polyline[-1])

            end_to_end_distances = [
                calculate_end_to_end_distance(self.features[f_id]) for f_id in potential_parallel_list
            ]

            return potential_parallel_list[np.argmin(end_to_end_distances)]

    def _determine_lane_width(self, feature: LaneCenter) -> float:
        """
        Determines the width of a lane based on the provided LaneCenter feature.

        This method calculates the lane width by evaluating the distances between
        the lane's left and right boundaries and its centerline. It returns the
        maximum of twice the minimum distance found and a default lane width.

        Args:
            feature (LaneCenter): The lane center feature containing the lane
                                  boundaries and centerline information.

        Returns:
            float: The calculated lane width.
        """

        boundaries = feature.lane.left_boundaries + feature.lane.right_boundaries
        min_distances = []
        for boundary in boundaries:
            avg_dis, max_dis, min_dis = polyline_distance(
                boundary.polyline,
                feature.lane.polyline[boundary.lane_start_index : boundary.lane_end_index + 1],
            )
            min_distances.append(min_dis)
        if len(min_distances):
            min_dis = min(min_distances)
            return max(2 * min_dis, self._default_lanewidth)
        else:
            return self._default_lanewidth

    """
    ################################ Consolidate edges into junctions ################################
    """

    def consolidate_edges_into_junctions(self) -> None:
        """
        Consolidates edges into junctions by grouping and merging related edges.

        This method performs multiple rounds of grouping to identify and merge edges
        that should be treated as a single junction. The grouping is done based on
        various criteria such as bifurcated/merged lanes, adjacency, overlapping,
        common incoming/outgoing edges, and geometric relationships.

        Steps:
        1. Group pairs of bifurcated/merged lanes.
        2. Group adjacent edge groups.
        3. Merge overlapping edge groups.
        4. Include external lanes that overlap with existing edge groups.
        5. Merge edge groups sharing a common incoming/outgoing edge.
        6. Ensure all out-edges of in-edges and in-edges of out-edges are in the same group.
        7. Include edges where both from-node and to-node are the same node.

        After grouping, the method creates joined junctions and deletes edges that
        have become internal edges.

        Returns:
            None
        """

        uf = UnionFind(len(self.edges))
        # grouping round 1: group pairs of bifurcated/merged lanes.
        self._union_lanepairs(uf)
        for _ in range(2):
            # grouping round 2: some groups are just next to each other, we should treat it as a single group
            self._union_adjacent_groups(uf)
            # grouping round 3: for pair of edge groups overlaping together, consider them as a single node
            self._union_overlapping_groups(uf)
            # grouping round 4: for an external lane that overlaps with an existing edge group geometrically, include it into the group
            self._union_overlapping_edges(uf)
            # grouping round 5: for pair of edge groups sharing a common incoming/outgoing edge, consider them as a single node
            self._union_groups_with_common_ioedge(uf)
            # grouping round 6: (to avoid error) for an edge group, all out-edges of all their in-edges should be also in this group. and all in-edges of all their out-edges should be also in this group
            self._union_friend_edges(uf)
            # grouping round 7: (to avoid error) some edges are lying in a position such that both its from-node and to-node are the same node. then we include these edges into that node
            self._union_wrapped_lanes(uf)

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]

        # creating joined junctions
        for edge_group in connection_groups:
            self._consolidate_edges_into_a_junction(edge_group)

        # delete edges that have become internal edges
        for edge_group in connection_groups:
            for edge_id in edge_group:
                self.edges.pop(edge_id)

    def _union_lanepairs(self, uf: UnionFind) -> None:
        """
        Unions lane pairs in the UnionFind structure based on the diverge and merge lanes.

        This method iterates over all edges and their lanes, and for each lane, it unions the current edge
        with the edges corresponding to the diverge and merge lanes in the UnionFind structure.

        Args:
            uf (UnionFind): The UnionFind structure used to union the edges.

        """

        for edge_id, edge in self.edges.items():
            for lane in edge.lanes:
                for feature_id in self.features[lane.to_WAYMOfeature].lane.diverge_lanes:
                    union_edge_id = self.features[feature_id].to_SUMO_edge
                    uf.union(edge_id, union_edge_id)

                for feature_id in self.features[lane.to_WAYMOfeature].lane.merge_lanes:
                    union_edge_id = self.features[feature_id].to_SUMO_edge
                    uf.union(edge_id, union_edge_id)

    def _union_overlapping_groups(self, uf: UnionFind) -> None:
        """
        Unions overlapping groups in a UnionFind structure based on their polygonal representation.

        This method processes groups formed by the UnionFind structure and identifies those that
        represent connection groups. It then creates polygons for each of these groups and checks
        for intersections between the polygons. If two polygons intersect with an area greater than
        3, their corresponding groups are unioned in the UnionFind structure.

        Args:
            uf (UnionFind): The UnionFind structure containing the groups to be processed.

        Returns:
            None
        """

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]
        group_polygons = [
            self._create_edges_polygon(group, allow_multipolygon=True) for group in connection_groups
        ]

        for i in range(len(group_polygons)):
            for j in range(len(group_polygons)):
                if (
                    i != j
                    and group_polygons[i]
                    and group_polygons[j]
                    and group_polygons[i].intersection(group_polygons[j]).area > 3
                ):
                    uf.union(connection_groups[i][0], connection_groups[j][0])

    def _union_adjacent_groups(self, uf: UnionFind) -> None:
        """
        Unions adjacent groups of edges in the UnionFind structure based on their connections.

        This method processes the UnionFind structure to identify and union adjacent groups of edges
        that are connected through their lanes. It first forms groups from the UnionFind structure,
        filters out the connection groups, and then iterates through the internal edges of these groups.
        For each edge, it checks the exit and entry lanes to determine if they belong to the internal
        edges and unions them accordingly.

        Args:
            uf (UnionFind): The UnionFind structure containing the edges and their groups.

        Returns:
            None
        """

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]
        internal_edges = [edge_id for group in connection_groups for edge_id in group]

        for edge_id in internal_edges:
            for lane in self.edges[edge_id].lanes:
                feature = self.features[lane.to_WAYMOfeature]
                for exit_lane in feature.lane.exit_lanes:
                    if self.features[exit_lane].to_SUMO_edge in internal_edges:
                        uf.union(edge_id, self.features[exit_lane].to_SUMO_edge)
                for entry_lane in feature.lane.entry_lanes:
                    if self.features[entry_lane].to_SUMO_edge in internal_edges:
                        uf.union(edge_id, self.features[entry_lane].to_SUMO_edge)

    def _union_overlapping_edges(self, uf: UnionFind) -> None:
        """
        Unions overlapping edges in the given UnionFind structure.

        This method processes the edges in the UnionFind structure to identify and union overlapping edges.
        It first forms groups of connected edges, then creates polygons for these groups. It identifies
        internal and external edges and checks for intersections between external edges and group polygons.
        If an intersection with an area greater than 40 is found, the external edge is unioned with the group.

        Args:
            uf (UnionFind): The UnionFind structure containing the edges to be processed.

        Returns:
            None
        """

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]
        group_polygons = [
            self._create_edges_polygon(group, allow_multipolygon=True) for group in connection_groups
        ]
        internal_edges = [edge_id for group in connection_groups for edge_id in group]
        external_edges = set(self.edges.keys()).difference(internal_edges)

        for edge_id in external_edges:
            edge_polygon = create_edge_polygon(
                self.features, self.edges[edge_id], use_boundary=self._consider_roadedge_in_nodeshape
            )
            for i in range(len(connection_groups)):
                if (
                    group_polygons[i]
                    and edge_polygon
                    and group_polygons[i].intersection(edge_polygon).area > 40
                ):
                    uf.union(edge_id, connection_groups[i][0])

    def _union_groups_with_common_ioedge(self, uf: UnionFind) -> None:
        """
        Unions groups with common incoming or outgoing edges.

        This method takes a UnionFind data structure and identifies groups that have common incoming or outgoing edges.
        It then unions these groups together.

        Args:
            uf (UnionFind): The UnionFind data structure containing the groups to be processed.

        Returns:
            None
        """

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]

        for group_i in connection_groups:
            for group_j in connection_groups:
                if group_i != group_j:
                    incoming_edges_of_group_i = self._get_connected_edges_of_edge_group(group_i, side="entry")
                    outgoing_edges_of_group_i = self._get_connected_edges_of_edge_group(group_i, side="exit")
                    incoming_edges_of_group_j = self._get_connected_edges_of_edge_group(group_j, side="entry")
                    outgoing_edges_of_group_j = self._get_connected_edges_of_edge_group(group_j, side="exit")
                    common_incoming_edges = incoming_edges_of_group_i.intersection(incoming_edges_of_group_j)
                    common_outgoing_edges = outgoing_edges_of_group_i.intersection(outgoing_edges_of_group_j)
                    if len(common_incoming_edges) or len(common_outgoing_edges):
                        uf.union(group_i[0], group_j[0])

    def _union_friend_edges(self, uf: UnionFind):
        """
        Unions friend edges within the UnionFind structure based on connection groups.

        This method processes the UnionFind structure to identify and union edges that are
        considered "friend edges". It first forms groups from the UnionFind structure and
        filters them to find connection groups. For each connection group, it identifies
        the connected edges on both the entry and exit sides and unions them with the
        representative edge of the group.

        Args:
            uf (UnionFind): The UnionFind structure containing the edges and their groups.

        Returns:
            None
        """

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]

        for group in connection_groups:
            in_edges = self._get_connected_edges_of_edge_group(group, side="entry")
            out_edges_of_in_edges = self._get_connected_edges_of_edge_group(
                in_edges, side="exit", self_excluded=False
            )
            for edge_id in out_edges_of_in_edges:
                uf.union(edge_id, group[0])
            out_edges = self._get_connected_edges_of_edge_group(group, side="exit")
            in_edges_of_out_edges = self._get_connected_edges_of_edge_group(
                out_edges, side="entry", self_excluded=False
            )
            for edge_id in in_edges_of_out_edges:
                uf.union(edge_id, group[0])

    def _union_wrapped_lanes(self, uf: UnionFind) -> None:
        """
        Unions wrapped lanes in the Union-Find structure.

        This method processes the edges in the scenario and unions the wrapped lanes
        based on their connections. It first forms groups using the Union-Find structure
        and identifies connection groups. It then determines the internal and external edges.
        For each external edge, it checks if the feature has both exit and entry lanes and
        unions the edge with its corresponding exit and entry edges if they belong to the same group.

        Args:
            uf (UnionFind): The Union-Find structure used to manage the groups of edges.

        Returns:
            None
        """

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if self._is_connection_group(elements)]
        internal_edges = [edge_id for group in connection_groups for edge_id in group]
        external_edges = set(self.edges.keys()).difference(internal_edges)

        for edge_id in external_edges:
            feature = self.features[self.edges[edge_id].lanes[0].to_WAYMOfeature]
            if feature.lane.exit_lanes and feature.lane.entry_lanes:
                exit_edge = self.features[feature.lane.exit_lanes[0]].to_SUMO_edge
                entry_edge = self.features[feature.lane.entry_lanes[0]].to_SUMO_edge
                if uf.find(exit_edge) == uf.find(entry_edge):
                    uf.union(edge_id, exit_edge)
                    uf.union(edge_id, entry_edge)

    def _consolidate_edges_into_a_junction(self, edge_group: list[int]) -> None:
        """
        Given a group of edges `edge_group`, create a joined junction, and turn all the lanes in these edges into connections. The concrete steps are:

        1. Create the junction
          1.1 Compute the position coordinates of the junction
            This is done by averaging points in all lanes of all edges
          1.2 Compute the shape of the junction.
            This is done by unioning the shapes of all lanes of all edges, and then removing the holes
        2. Linking relevant incoming and outgoing edges
           Find out the set of incoming edges, and set their to-node to be this junction
           Find out the set of outgoing edges, and set their from-node to be this junction
        3. Create connections
            A single lane itself is not necessarily a complete connection, so for each lane, all possible sets of connected lanes are searched exhaustively, and connections are created for each such set
        4. Set junction type
          4.1 If a junction is a traffic light junction, set up the traffic light
          4.2 If a junction has stop signs, set up necessary prohibitions

        Args:
            edge_group (list[int]): A list of edge IDs to be consolidated into a junction.

        Returns:
            None
        """

        # I. create junction
        # junction coordinates
        pt_set: list[Pt] = [
            pt for edge_id in edge_group for lane in self.edges[edge_id].lanes for pt in lane.shape
        ]
        avg_x = sum(pt.x for pt in pt_set) / len(pt_set)
        avg_y = sum(pt.y for pt in pt_set) / len(pt_set)
        avg_z = sum(pt.z for pt in pt_set) / len(pt_set)

        # junction shape
        if not self._custom_nodeshape:
            shape = None
        else:
            merged_polygon = self._create_edges_polygon(edge_group, allow_multipolygon=False)
            shape = [Pt(*pt) for pt in merged_polygon.exterior.coords] if merged_polygon else None

        # keepclear attribute
        keep_clear = not (len(edge_group) == 1)
        node_id = self._create_node(Point(avg_x, avg_y, avg_z), shape=shape, keep_clear=keep_clear)

        # II. link connected edges to this node
        # incoming
        in_edges: set[int] = self._get_connected_edges_of_edge_group(edge_group, side="entry")
        for in_edge_id in in_edges:
            if self.edges[in_edge_id].to_node_id == None:
                self.edges[in_edge_id].to_node_id = node_id
                self.nodes[node_id].incoming_SUMO_edges.add(in_edge_id)  # mapping: node->edge

        # outgoing
        out_edges: set[int] = self._get_connected_edges_of_edge_group(edge_group, side="exit")
        for out_edge_id in out_edges:
            if self.edges[out_edge_id].from_node_id == None:
                self.edges[out_edge_id].from_node_id = node_id
                self.nodes[node_id].outgoing_SUMO_edges.add(out_edge_id)  # mapping: node->edge

        # III. set connections lane-2-lane
        all_feature_ids: list[int] = [
            lane.to_WAYMOfeature for edge_id in edge_group for lane in self.edges[edge_id].lanes
        ]
        all_paths: list[list[int]] = self._find_all_connection_paths(all_feature_ids, in_edges, out_edges)
        for path in all_paths:

            try:
                incomming_feature = self.features[path[0]].lane.entry_lanes[0]
                outgoing_feature = self.features[path[-1]].lane.exit_lanes[0]
            except KeyError:
                # sometime key error occurs
                continue

            # compute necessary connection attributes
            from_edge = self.features[incomming_feature].to_SUMO_edge
            to_edge = self.features[outgoing_feature].to_SUMO_edge
            from_lane = self.features[incomming_feature].to_SUMO_lane
            to_lane = self.features[outgoing_feature].to_SUMO_lane
            speed = sum(map(mph_to_ms, [self.features[f_id].lane.speed_limit_mph for f_id in path])) / len(
                path
            )
            shape: list[Pt] = [point for fid in path for point in self.features[fid].lane.polyline]
            need_stop = any([self.features[f_id].needs_stop for f_id in path])

            tl_states_at_each_step = [
                [
                    self.features[f_id].record_tls[i]
                    for f_id in path
                    if self.features[f_id].record_tls[i] != TLS.ABSENT
                ]
                for i in range(91)
            ]
            tl_state_of_this_conn = [
                (sublist[0] if sublist else TLS.ABSENT) for sublist in tl_states_at_each_step
            ]

            conn = Connection(
                from_edge,
                to_edge,
                from_lane,
                to_lane,
                # speed=speed,
                shape=shape,
                need_stop=need_stop,
                WAYMO_features=path,
                rec_tl_state=tl_state_of_this_conn,
            )
            self.nodes[node_id].connections.add(conn)
            # if any connection is too long in shape, we consider the conversion not successful
            if polyline_length(shape) > self._CONNECTION_DISCARD_LENGTH:
                self.ok = False
                print(
                    f"<bad> connection {from_edge}_{from_lane}->{to_edge}_{to_lane} too long {polyline_length(shape)}m"
                )

        # V. set junction type based on found information of connections
        # traffic light node
        if any(
            tl_state != WaymonicTLS.ABSENT  # not -1 when has signal
            for edge_id in edge_group
            for lane in self.edges[edge_id].lanes
            for tl_state in self.features[lane.to_WAYMOfeature].record_tls
        ):
            self.nodes[node_id].type = self._TL_TYPE
        # all-way stop node
        elif all(conn.need_stop for conn in self.nodes[node_id].connections):
            self.nodes[node_id].type = "allway_stop"
        # priority stop node
        elif any(conn.need_stop for conn in self.nodes[node_id].connections):
            self.nodes[node_id].type = "priority_stop"
            for conn_i in self.nodes[node_id].connections:
                # for minor links, add prohibitions
                if conn_i.need_stop:
                    for conn_j in self.nodes[node_id].connections:
                        if not conn_j.need_stop:
                            self.nodes[node_id].prohibitions.add(
                                Prohibition(
                                    conn_j.from_edge, conn_j.to_edge, conn_i.from_edge, conn_i.to_edge
                                )
                            )
                # for straight major links, set pass=true
                # turing links are still considered minor even if there is no associated stop sign
                elif conn_i.direction == Direction.S:
                    conn_i.can_pass = True
        else:
            self.nodes[node_id].type = self._DEFAULT_NODE_TYPE

    def _find_all_connection_paths(
        self, all_feature_ids: list[int], in_edges: set[int], out_edges: set[int]
    ) -> list[list[int]]:
        """
        Given a list of feature ids as connections in a junction,
        with in_edges and out_edges as the junction's all incoming edges and outgoing edges,
        find all paths of features that form valid connections of this junction, using DFS strategy.

        Args:
            all_feature_ids (list[int]): List of all feature IDs.
            in_edges (set[int]): Set of incoming edge IDs.
            out_edges (set[int]): Set of outgoing edge IDs.

        Returns:
            list[list[int]]: A list of found connection paths. Each return path (as a sublist) is an ordered list of feature IDs.
        """

        # first filter start and end nodes
        start_features: list[int] = [
            f_id
            for f_id in all_feature_ids
            if any(
                (self.features[id].to_SUMO_edge in in_edges) for id in self.features[f_id].lane.entry_lanes
            )
        ]
        end_features: list[int] = [
            f_id
            for f_id in all_feature_ids
            if any(
                (self.features[id].to_SUMO_edge in out_edges) for id in self.features[f_id].lane.exit_lanes
            )
        ]

        # dfs function
        def _dfs(
            start_fid: int, end_fid: int, path: list[int], paths: list[list[int]], visited: dict[str, bool]
        ):
            path.append(start_fid)
            visited[start_fid] = True

            if start_fid == end_fid:
                paths.append(path[:])
            else:
                for f_id in self.features[start_fid].lane.exit_lanes:
                    if f_id in all_feature_ids and not visited[f_id]:
                        _dfs(f_id, end_fid, path, paths, visited)

            path.pop()
            visited[start_fid] = False

        # recursively search for all paths
        visited = {f_id: False for f_id in all_feature_ids}
        paths: list[list[int]] = []
        for start_fid in start_features:
            for end_fid in end_features:
                path: list[int] = []
                _dfs(start_fid, end_fid, path, paths, visited)

        return paths[:]

    """
    ################################ Link edges into junctions ################################
    """

    def link_edge_to_junctions(self, edge: Edge) -> None:
        """
        Links an edge to its corresponding junction nodes by updating the edge's from_node_id and to_node_id.
        This method also establishes connections between lanes of the edge and lanes of other edges at the junctions.

        Args:
            edge (Edge): The edge to be linked to junction nodes.

        Returns:
            None
        """

        # check from-node of the edge
        if edge.from_node_id == None:
            edge.from_node_id = self._find_junction_id_of_an_edge(edge, side="entry")
            self.nodes[edge.from_node_id].outgoing_SUMO_edges.add(edge.id)

            # add connection in this node (might be duplicated, but does not matter as node.connections is a set)
            for lane_idx, lane in enumerate(edge.lanes):
                feature = self.features[lane.to_WAYMOfeature]
                for entry_feature in feature.lane.entry_lanes:
                    entry_edge = self.features[entry_feature].to_SUMO_edge
                    entry_lane = self.features[entry_feature].to_SUMO_lane
                    connection = Connection(entry_edge, edge.id, entry_lane, lane_idx)
                    self.nodes[edge.from_node_id].connections.add(connection)

        # check to-node of the edge
        if edge.to_node_id == None:
            edge.to_node_id = self._find_junction_id_of_an_edge(edge, side="exit")
            self.nodes[edge.to_node_id].incoming_SUMO_edges.add(edge.id)

            # add connection in this node
            for lane_idx, lane in enumerate(edge.lanes):
                feature = self.features[lane.to_WAYMOfeature]
                for exit_feature in feature.lane.exit_lanes:
                    exit_edge = self.features[exit_feature].to_SUMO_edge
                    exit_lane = self.features[exit_feature].to_SUMO_lane
                    connection = Connection(edge.id, exit_edge, lane_idx, exit_lane)
                    self.nodes[edge.to_node_id].connections.add(connection)

    def _find_junction_id_of_an_edge(self, edge: Edge, side: str) -> int:
        """
        Finds the junction ID of a given edge based on the specified side (entry or exit).

        Args:
            edge (Edge): The edge for which the junction ID is to be found.
            side (str): The side of the edge to consider, either "entry" or "exit".

        Returns:
            int: The junction ID of the edge.

        Notes:
            - If the side is "entry", the method first tries to find an existing node from its entry lanes.
              If no node is found, it tries to find another exit lane which has a from-node.
              If still no node is found, it creates a new node.
            - If the side is "exit", the method first tries to find an existing node from its exit lanes.
              If no node is found, it tries to find another exit lane which has a to-node.
              If still no node is found, it creates a new node.
        """

        connected_edges = self._get_connected_edges_of_edge_group([edge.id], side=side)
        friend_edges = self._get_connected_edges_of_edge_group(
            connected_edges, side=("entry" if side == "exit" else "exit")
        )
        if side == "entry":
            # try to find an existing node from its entry lanes
            # the edge of the entry lane might already have a to-node
            for edge_id in connected_edges:
                if self.edges[edge_id].to_node_id != None:
                    return self.edges[edge_id].to_node_id
            # if not, the edge of the entry lane might have another exit_lane which has from-node
            for edge_id in friend_edges:
                if self.edges[edge_id].from_node_id != None:
                    return self.edges[edge_id].from_node_id
            # if nothing found, create one
            return self._create_node(edge.lanes[0].shape[0], keep_clear=False)
        else:
            # try to find an existing node fome its exit lanes
            # the edge of the entry lane might already have a to-node
            for edge_id in connected_edges:
                if self.edges[edge_id].from_node_id != None:
                    return self.edges[edge_id].from_node_id
            # if not, the edge of the entry lane might have another exit_lane which has from-node
            for edge_id in friend_edges:
                if self.edges[edge_id].to_node_id != None:
                    return self.edges[edge_id].to_node_id
            return self._create_node(edge.lanes[0].shape[-1], keep_clear=False)

    """
    ################################ Remove seld-loop edges ################################
    """

    def remove_selfloop_edges(self) -> None:
        """
        Removes self-loop edges from the graph.

        This method removes edges that have the same from_node_id and to_node_id, indicating a self-loop.
        It also removes connections and prohibitions that are connected to the deleted edges.
        """

        # remove self-loop edges
        self.edges = {
            edge_id: edge for edge_id, edge in self.edges.items() if edge.from_node_id != edge.to_node_id
        }

        # remove connections and prohibitions that are connected to deleted edges
        for node in self.nodes.values():
            node.connections = set(
                conn
                for conn in node.connections
                if conn.from_edge in self.edges and conn.to_edge in self.edges
            )
            node.incoming_SUMO_edges = set(
                edge_id for edge_id in node.incoming_SUMO_edges if edge_id in self.edges
            )
            node.outgoing_SUMO_edges = set(
                edge_id for edge_id in node.outgoing_SUMO_edges if edge_id in self.edges
            )
            node.prohibitions = set(
                prohibition
                for prohibition in node.prohibitions
                if all(
                    edge_id in self.edges
                    for edge_id in [
                        prohibition.prohibited_from,
                        prohibition.prohibited_to,
                        prohibition.prohibitor_from,
                        prohibition.prohibitor_to,
                    ]
                )
            )

    """
    ################################ Concatenate egdes ################################
    """

    def concatenate_edges(self) -> None:
        """
        Concatenates edges in the scenario processor.

        This method finds and concatenates edges in the scenario processor based on certain conditions.
        It iterates over the nodes in the processor and checks if the node has been inspected.
        If the node has not been inspected, it finds the concatenation edges for that node using the _find_concatenation_edges method.
        If concatenation edges are found, they are added to the concatenation_edges_groups list.
        Finally, the _concatenate_edges_op method is called for each group of concatenation edges.
        """

        is_node_inspected = {node_id: False for node_id in self.nodes.keys()}
        concatenation_edges_groups: list[list[int]] = []
        for node_id, node in self.nodes.items():
            if not is_node_inspected[node_id]:
                concatenation_edges = self._find_concat_edges(node, is_node_inspected)
                if concatenation_edges:
                    concatenation_edges_groups.append(concatenation_edges)

        for edge_id_seq in concatenation_edges_groups:
            self._do_edges_concat(edge_id_seq)

    def _find_concat_edges(self, node: Node, is_node_inspected: dict[int, bool]) -> list[int]:
        """
        Finds the edges that can be concatenated starting from the given node.

        This method inspects the given node and recursively checks its connected nodes
        to find all edges that can be concatenated. It ensures that nodes are not
        inspected more than once and avoids self-looping edges.

        Args:
            node (Node): The starting node.
            is_node_inspected (dict[int, bool]): A dictionary to keep track of inspected nodes.

        Returns:
            list[int]: A list of edge IDs that can be concatenated.
        """

        # mark this node as having been inspected
        is_node_inspected[node.id] = True
        if not self._can_be_concatenated(node):
            return []

        entry_edge_id = list(node.incoming_SUMO_edges)[0]
        exit_edge_id = list(node.outgoing_SUMO_edges)[0]
        concat_edges = [entry_edge_id, exit_edge_id]

        # recursively find next edge to concat
        next_node = self.nodes[self.edges[exit_edge_id].to_node_id]
        while self._can_be_concatenated(next_node):
            if is_node_inspected[next_node.id]:
                break
            is_node_inspected[next_node.id] = True
            next_exit_edge_id = list(next_node.outgoing_SUMO_edges)[0]
            concat_edges.append(next_exit_edge_id)
            next_node = self.nodes[self.edges[next_exit_edge_id].to_node_id]

        # recursively find prior edges to concat
        prev_node = self.nodes[self.edges[entry_edge_id].from_node_id]
        while self._can_be_concatenated(prev_node):
            if is_node_inspected[prev_node.id]:
                break
            is_node_inspected[prev_node.id] = True
            prev_entry_edge_id = list(prev_node.incoming_SUMO_edges)[0]
            concat_edges.insert(0, prev_entry_edge_id)
            prev_node = self.nodes[self.edges[prev_entry_edge_id].from_node_id]

        # avoid self-looping
        while concat_edges[-1] == concat_edges[0]:
            concat_edges.pop()
        if concat_edges[0] in self.nodes[self.edges[concat_edges[-1]].to_node_id].outgoing_SUMO_edges:
            concat_edges.pop()

        return concat_edges

    def _can_be_concatenated(self, node: Node) -> bool:
        """
        Check if a given node can be concatenated.

        A node can be concatenated if it meets the following prerequisites:
        - The node should not have a shape.
        - The node should have exactly one incoming edge and one outgoing edge.
        - The incoming and outgoing edges should have the same number of lanes.
        - Each pair of lanes between the incoming and outgoing edges should have a connection,
          and their properties (type and speed) should be similar.

        Args:
            node (Node): The node to be checked.

        Returns:
            bool: True if the node can be concatenated, False otherwise.
        """

        # prerequisite: should be a node with shape
        if node.shape != None:
            return False
        # prerequisite: should only have 1 incoming edge and 1 outgoing edge
        if not (len(node.incoming_SUMO_edges) == 1 and len(node.outgoing_SUMO_edges) == 1):
            return False

        # prerequisite: should have equal number of lanes
        edge1 = self.edges[list(node.incoming_SUMO_edges)[0]]
        edge2 = self.edges[list(node.outgoing_SUMO_edges)[0]]
        if len(edge1.lanes) != len(edge2.lanes):
            return False

        # each pair of lane should have connection, and their properties are similar
        connection_set: set[tuple[int]] = {(conn.from_lane, conn.to_lane) for conn in node.connections}
        for i, (lane_i, lane_j) in enumerate(zip(edge1.lanes, edge2.lanes)):
            if (
                (i, i) not in connection_set  # there is a connection connecting both lane
                or lane_i.type != lane_j.type  # lane type the same
                or abs(lane_i.speed - lane_j.speed) >= 5  # lane speed similar
            ):
                return False
        return True

    def _do_edges_concat(self, edge_id_seq: list[int]) -> None:
        """
        Concatenates multiple edges into a single edge.

        This method takes a sequence of edge IDs, concatenates their lanes to form a new edge,
        and updates the connections and prohibitions in the start and end nodes accordingly.
        Intermediate nodes and edges are deleted after concatenation.

        Args:
            edge_id_seq (list[int]): An ordered list of edge IDs to be concatenated.

        Returns:
            None
        """

        edges_seq = [self.edges[edge_id] for edge_id in edge_id_seq]
        # concatenate each lane, and form a new edge
        new_lanes: list[Lane] = []
        for i in range(len(edges_seq[0].lanes)):
            new_lanes.append(
                Lane(
                    speed=np.mean([edge.lanes[i].speed for edge in edges_seq]),
                    shape=[pt for edge in edges_seq for pt in edge.lanes[i].shape],
                    width=np.mean([edge.lanes[i].width for edge in edges_seq]),
                    type=edges_seq[0].lanes[i].type,
                )
            )
        new_edge_id = self._create_edge(new_lanes, edges_seq[0].from_node_id, edges_seq[-1].to_node_id)

        # make modification to the information stored in start and end node
        def _replace_id_in_conn(connection: Connection, before: int, after: int) -> None:
            for attr in ['from_edge', 'to_edge']:
                if getattr(connection, attr) == before:
                    setattr(connection, attr, after)

        def _replace_id_in_proh(prohibition: Prohibition, before: int, after: int) -> None:
            for attr in ['prohibited_from', 'prohibited_to', 'prohibitor_from', 'prohibitor_to']:
                if getattr(prohibition, attr) == before:
                    setattr(prohibition, attr, after)

        # start node
        for conn in self.nodes[edges_seq[0].from_node_id].connections:
            _replace_id_in_conn(conn, edges_seq[0].id, new_edge_id)
        for proh in self.nodes[edges_seq[0].from_node_id].prohibitions:
            _replace_id_in_proh(proh, edges_seq[0].id, new_edge_id)
        self.nodes[edges_seq[0].from_node_id].outgoing_SUMO_edges.remove(edges_seq[0].id)
        self.nodes[edges_seq[0].from_node_id].outgoing_SUMO_edges.add(new_edge_id)

        # end node
        for conn in self.nodes[edges_seq[-1].to_node_id].connections:
            _replace_id_in_conn(conn, edges_seq[-1].id, new_edge_id)
        for proh in self.nodes[edges_seq[-1].to_node_id].prohibitions:
            _replace_id_in_proh(proh, edges_seq[-1].id, new_edge_id)
        self.nodes[edges_seq[-1].to_node_id].incoming_SUMO_edges.remove(edges_seq[-1].id)
        self.nodes[edges_seq[-1].to_node_id].incoming_SUMO_edges.add(new_edge_id)

        # delete intermediate node and edges
        # intermediate edges
        for edge in edges_seq[:-1]:
            if edge.to_node_id in self.nodes:
                del self.nodes[edge.to_node_id]
        # intermediate nodes
        for edge_id in edge_id_seq:
            if edge_id in self.edges:
                del self.edges[edge_id]

    """
    ################################## Add sidewalks ##################################
    """

    def add_sidewalks(self) -> None:
        """
        Adds sidewalks to the scenario by performing the following steps:

        1. Filters sidewalk edges and stores their widths in a dictionary.
        2. Increments the lane count for connections connected to sidewalk edges.
        3. Computes the sidewalk shape and creates a sidewalk lane for each sidewalk edge.
        4. Adjusts the shape of nodes with a non-empty shape attribute.
        """

        sidewalk_widths: dict[int, float] = {edge_id: None for edge_id in self.edges.keys()}

        # step 1
        for node in self.nodes.values():
            self._filter_sidewalk_edges(node, sidewalk_widths)
        assert not any(width == None for width in sidewalk_widths.values())

        # step 2
        for node in self.nodes.values():
            for conn in node.connections:
                if conn.from_edge in sidewalk_widths:
                    conn.from_lane += 1
                if conn.to_edge in sidewalk_widths:
                    conn.to_lane += 1

        # step 3
        DEFAULT_SIDEWALK_SPEED = 2.78
        for edge_id, sidewalk_width in sidewalk_widths.items():
            sidewalk_shape = self._compute_sidewalk_shape(self.edges[edge_id], sidewalk_width)
            sidewalk_lane = Lane(
                DEFAULT_SIDEWALK_SPEED, sidewalk_shape, sidewalk_width, type=LaneTypeSUMO.SIDEWALK
            )
            self.edges[edge_id].lanes.insert(0, sidewalk_lane)

        # step 4
        for node in self.nodes.values():
            if node.shape:
                node.shape = self._adjust_node_shape(node)

    def _filter_sidewalk_edges(self, node: Node, sidewalk_widths: dict[int, float]) -> None:
        """
        Filters the edges connected to a given node to determine which edges can have sidewalks.

        This method processes the edges connected to the specified node and updates the `sidewalk_widths`
        dictionary to reflect which edges can have sidewalks and their respective widths. It ensures that
        only the rightmost edge in each way has the possibility of having a sidewalk and that the sidewalk
        does not overlap with other edges.

        Args:
            node (Node): The node whose connected edges are to be filtered.
            sidewalk_widths (dict[int, float]): A dictionary mapping edge IDs to their respective sidewalk widths.

        Returns:
            None
        """

        def _filter_sidewalk_edges_in_a_way(way: list[Edge], side: str) -> None:
            # only the rightmost edge in this way has the possibility to have a sidewalk
            rightmost_edge = self._find_rightmost_edge(way, side)

            # other edges are definitely no sidewalk
            for edge in way:
                if edge.id != rightmost_edge.id and edge.id in sidewalk_widths:
                    del sidewalk_widths[edge.id]
            # maybe this rightmost edge has been proven to have no sidewalk
            if rightmost_edge.id not in sidewalk_widths:
                return

            # determine a width such that it is as wide as possible but not overlapping with other edges
            rightmost_edge_has_sidewalk = False
            sidewalk_width = 4.0
            while sidewalk_width >= 2.0:

                sidewalk_shape = self._compute_sidewalk_shape(rightmost_edge, sidewalk_width)
                sidewalk_polygon = LineString([pt.to_list() for pt in sidewalk_shape]).buffer(
                    sidewalk_width / 2,
                    cap_style=shapely.BufferCapStyle.flat,
                    join_style=shapely.BufferJoinStyle.round,
                )
                all_edges_polygons = [
                    create_edge_polygon(self.features, self.edges[id], use_boundary=False)
                    for id in self.edges.keys()
                ]
                if any(
                    sidewalk_polygon.intersection(other_polygon).area >= 8
                    for other_polygon in all_edges_polygons
                ):
                    sidewalk_width -= 0.2
                else:
                    rightmost_edge_has_sidewalk = True
                    break

            if rightmost_edge_has_sidewalk:
                sidewalk_widths[rightmost_edge.id] = sidewalk_width
            else:
                del sidewalk_widths[rightmost_edge.id]

        incoming_ways = self._get_ways_from_node(node, type="incoming")
        outgoing_ways = self._get_ways_from_node(node, type="outgoing")
        for way in incoming_ways:
            _filter_sidewalk_edges_in_a_way(way, side="entry")
        for way in outgoing_ways:
            _filter_sidewalk_edges_in_a_way(way, side="exit")

    @staticmethod
    def _compute_sidewalk_shape(edge: Edge, sidewalk_width: float) -> list[Pt]:
        """
        Computes the shape of the sidewalk for a given edge and sidewalk width.

        This method calculates the shape of the sidewalk by offsetting the shape of the first lane
        of the edge by half the width of the lane plus the sidewalk width. The resulting shape is
        returned as a list of Pt objects.

        Args:
            edge (Edge): The edge for which the sidewalk shape is to be computed.
            sidewalk_width (float): The width of the sidewalk.

        Returns:
            list[Pt]: A list of Pt objects representing the shape of the sidewalk.
        """

        first_lane = edge.lanes[0]
        lane_shape = LineString([pt.to_list() for pt in first_lane.shape])
        sidewalk_linestring = lane_shape.offset_curve(distance=-(first_lane.width + sidewalk_width) / 2)
        assert isinstance(sidewalk_linestring, LineString)

        def _index_into_laneshape(i):
            return min(
                i * len(first_lane.shape) // len(sidewalk_linestring.coords), len(first_lane.shape) - 1
            )

        sidewalk_linestring_zs = [
            first_lane.shape[_index_into_laneshape(i)].z for i in range(len(sidewalk_linestring.coords))
        ]

        sidewalk_shape = [Pt(*pt, z) for pt, z in zip(sidewalk_linestring.coords, sidewalk_linestring_zs)]
        return sidewalk_shape

    @staticmethod
    def _find_rightmost_edge(edges: list[Edge], side: str) -> Edge:
        """
        Finds the rightmost edge from a list of edges based on the specified side.

        Args:
            edges (list[Edge]): A list of Edge objects to evaluate.
            side (str): The side to consider for the estimation vector.

        Returns:
            Edge: The rightmost Edge object from the list.
        """

        edge_vectors = [edge.get_estimation_vector(side)[:2] for edge in edges]
        avg_vector = np.mean(np.array(edge_vectors), axis=0)
        avg_vector = avg_vector / np.linalg.norm(avg_vector)
        perp_vector = np.array([avg_vector[1], -avg_vector[0]])

        max_projection = -np.inf
        rightmost_edge = edges[0]
        for edge in edges:
            first_lane = edge.lanes[0]
            for pt in first_lane.shape:
                point_array = np.array(pt.to_list())[:2]
                x_proj = np.dot(point_array, perp_vector)
                if x_proj > max_projection:
                    max_projection = x_proj
                    rightmost_edge = edge
        return rightmost_edge

    def _adjust_node_shape(self, node: Node) -> list[Pt]:
        """
        Adjusts the shape of a given node by considering its incoming edges and their connections.

        This method processes the incoming edges of the node, specifically focusing on edges that are of type SIDEWALK.
        It identifies connected edges and calculates the appropriate shape adjustments to ensure proper connectivity
        and alignment of sidewalks. The adjusted shape is then used to update the node's walking areas.

        Args:
            node (Node): The node whose shape is to be adjusted.

        Returns:
            list[Pt]: A list of points representing the adjusted shape of the node.
        """

        addiitonal_polygons = []

        for edge_id in node.incoming_SUMO_edges:
            if self.edges[edge_id].lanes[0].type == LaneTypeSUMO.SIDEWALK:
                sidewalk_connected_edge_ids: list[Connection] = []

                for conn in node.connections:
                    if (
                        conn.from_edge == edge_id
                        and conn.from_lane == 1
                        and conn.to_lane == 1
                        and self.edges[conn.to_edge].lanes[0].type == LaneTypeSUMO.SIDEWALK
                    ):
                        sidewalk_connected_edge_ids.append(conn)

                if len(sidewalk_connected_edge_ids) > 1:

                    select_connected_edge_vectors = [
                        self.edges[conn.to_edge].get_estimation_vector("exit")
                        for conn in sidewalk_connected_edge_ids
                    ]

                    vector_headings = [vector_heading(vector) for vector in select_connected_edge_vectors]
                    self_vector = -self.edges[edge_id].get_estimation_vector("entry")
                    self_heading = vector_heading(self_vector)

                    counterclockwise_diff = [
                        (other_heading - self_heading) % (np.pi * 2) for other_heading in vector_headings
                    ]
                    select_idx = np.argmin(counterclockwise_diff)

                elif len(sidewalk_connected_edge_ids) == 1:
                    select_idx = 0
                else:
                    continue

                select_conn = sidewalk_connected_edge_ids[select_idx]
                select_connected_edge = sidewalk_connected_edge_ids[select_idx].to_edge
                # print(f"choose {select_connected_edge}")
                connection_linestring = LineString([pt.to_list() for pt in select_conn.shape])
                vehicle_lane_max_width = min(
                    self.edges[edge_id].lanes[1].width, self.edges[select_connected_edge].lanes[1].width
                )
                sidewalk_max_width = min(
                    self.edges[edge_id].lanes[0].width, self.edges[select_connected_edge].lanes[0].width
                )
                whole_offset = connection_linestring.offset_curve(
                    -(vehicle_lane_max_width + sidewalk_max_width) / 2
                )

                assert isinstance(whole_offset, LineString)
                whole_offset = [Pt(*pt, 0) for pt in whole_offset.coords]
                if not len(whole_offset):
                    # print("warning: offset is empty 1")
                    continue

                start_idx = find_polyline_nearest_point(
                    whole_offset, Pt(*(select_conn.shape[0].to_list()[:2]), 0)
                )
                end_idx = find_polyline_nearest_point(
                    whole_offset, Pt(*(select_conn.shape[-1].to_list()[:2]), 0)
                )
                if start_idx + 1 >= end_idx:
                    # print("warning: offset is empty 2")
                    continue

                partial_offset = whole_offset[start_idx : end_idx + 1]
                partial_offset = LineString([pt.to_list() for pt in partial_offset])
                addiitonal_polygon = partial_offset.buffer(
                    sidewalk_max_width / 2,
                    cap_style=shapely.BufferCapStyle.flat,
                    join_style=shapely.BufferJoinStyle.mitre,
                    mitre_limit=10,
                )
                addiitonal_polygons.append(addiitonal_polygon)

                if isinstance(addiitonal_polygon, MultiPolygon):
                    addiitonal_polygon = multipolygon_force_union(addiitonal_polygon)
                node.walking_areas.append(
                    WalkingArea(
                        edges=[edge_id, conn.to_edge],
                        shape=(
                            [Pt(*pt) for pt in addiitonal_polygon.exterior.coords]
                            if addiitonal_polygon
                            else None
                        ),
                    )
                )

        new_polygon = unary_union(addiitonal_polygons + [Polygon([pt.to_list()[:2] for pt in node.shape])])
        new_polygon = polygon_remove_holes(new_polygon)
        if isinstance(new_polygon, MultiPolygon):
            new_polygon = multipolygon_force_union(new_polygon)
        if not new_polygon:
            return node.shape

        return [Pt(*pt) for pt in new_polygon.exterior.coords]

    """
    ################################ Add crossings ################################
    """

    def add_crossing(self, crossing_feature) -> None:
        """
        Adds a crossing to the scenario by determining the appropriate node and edges to associate with the crossing feature.

        Args:
            crossing_feature (list[Pt]): A list of points representing the crossing feature.

        The method performs the following steps:
        1. Identifies the node to which the crossing should be assigned by calculating the intersection area between the crossing polygon and node polygons.
        2. Determines the edges that the crossing should span by calculating the distance between the crossing polygon and the edges connected to the node.
        3. Checks if a crossing already exists at the node with the same edges.
        4. If not, it adds the crossing to the node's crossings list with the appropriate shape and width.

        Notes:
            - The method uses the Shapely library to handle geometric operations.
            - The crossing is only added if the intersection area is significant and the distance to the edges is within a threshold.
        """

        crosswalk_polygon = Polygon([[pt.x, pt.y] for pt in crossing_feature])
        node_polygons = {
            node_id: Polygon([pt.to_list()[:2] for pt in node.shape])
            for node_id, node in self.nodes.items()
            if node.shape != None
        }

        assign_node_id = None
        overlap_area = 0
        for node_id, node_polygon in node_polygons.items():
            intersection = crosswalk_polygon.intersection(node_polygon)
            if intersection.area > 5 and intersection.area > overlap_area:
                assign_node_id = node_id
                overlap_area = intersection.area

        if assign_node_id == None:
            return

        assign_node = self.nodes[assign_node_id]
        ways: list[tuple[list[Edge], list[Edge]]] = self._get_ways_from_node(self.nodes[assign_node_id])
        ways: list[list[Edge]] = [[*way[0], *way[1]] for way in ways]

        # the extracted polyline of the crossing
        xyz = np.array([[p.x, p.y, p.z] for p in crossing_feature])
        polygon_idx = np.linspace(0, xyz.shape[0], 4, endpoint=False, dtype=int)
        # print(xyz)
        pl_polygon = self._get_polylines_from_polygon(xyz[polygon_idx])
        # print(pl_polygon)
        center_polyline = np.mean(np.array(pl_polygon), axis=0)
        # print(center_polyline)
        assert center_polyline.shape == (2, 3)

        def _way_distance(way: list[Edge]) -> float:
            points: list[Pt] = []
            for edge in way:
                idx = 0 if edge.id in assign_node.outgoing_SUMO_edges else -1
                for lane in edge.lanes:
                    points.append(lane.shape[idx])

            mean_distance = np.mean(
                [shapely.distance(crosswalk_polygon, Point(point.x, point.y)) for point in points]
            )
            max_distance = np.max(
                [shapely.distance(crosswalk_polygon, Point(point.x, point.y)) for point in points]
            )
            if max_distance > 5:
                return np.inf
            return mean_distance

        way_distance = [_way_distance(way) for way in ways]
        if np.min(way_distance) > 5:
            return
        connection_edges = [edge.id for edge in ways[np.argmin(way_distance)]]
        if any(set(connection_edges) == set(crosswalk.edges) for crosswalk in assign_node.crossings):
            print(" crosswalk already exist")
            return

        print(f" crosswalk at node {assign_node_id} - edge {connection_edges}")
        node_centroid = shapely.centroid(node_polygons[assign_node_id])
        ap = np.array([node_centroid.x, node_centroid.y]) - center_polyline[0][:2]
        ab = (center_polyline[1] - center_polyline[0])[:2]
        if np.cross(ab, ap) > 0:
            pass
        else:
            center_polyline = center_polyline[::-1]

        assign_node.crossings.append(
            Crossing(edges=connection_edges, width=4, shape=[Pt(*pt) for pt in center_polyline])
        )

    @staticmethod
    def _get_polylines_from_polygon(polygon: np.ndarray):
        # polygon: [4, 3]
        l1 = np.linalg.norm(polygon[1, :2] - polygon[0, :2])
        l2 = np.linalg.norm(polygon[2, :2] - polygon[1, :2])
        if l1 > l2:
            pl1 = np.array([polygon[0], polygon[1]])
            pl2 = np.array([polygon[2], polygon[3]])
        else:
            pl1 = np.array([polygon[0], polygon[3]])
            pl2 = np.array([polygon[2], polygon[1]])
        return [pl1, pl2[::-1]]
