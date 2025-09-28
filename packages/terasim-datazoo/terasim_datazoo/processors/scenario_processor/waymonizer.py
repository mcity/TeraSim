from typing import Union

from terasim_datazoo.core.waymo import *
from terasim_datazoo.utils import (
    UnionFind,
    polyline_length,
    distance_between_points,
    two_lines_parallel,
    find_polyline_nearest_point,
    real_neighbor_type,
)


class Waymonizer:
    def __init__(self, scenario) -> None:

        self._LANE_SHORT_THRESHOLD = 2

        self._POINT_CLOSE_THRESHOLD = 5
        """
        (meters) given two points point1 & point2, how much less must the distance between them in order to be considered close?
        """

        self._OVERLAP_PROPORTION_THRESHOLD = 0.25
        """
        (0-1) Given two lanes feature1 and feature2, how much overlap do they have to be in order to be considered "real neighbors"?
        """

        self._INDEX_SNAP_THRESHOLD = 4
        """
        (index) given a truncation index, how close must it to either start point or end point in order to be ignored? This is to avoid infinite loop.
        """

        self._LINE_PARALLEL_THRESHOLD = 15

        self.lanecenters: dict[int, LaneCenter] = {}
        self.traffic_lights: set[Pt] = set()
        self.stop_signs: set[Pt] = set()

        self._load_scenario(scenario)
        self._clean_features_information()
        self.signalized_intersections = self._find_special_intersections("signalized_intersection")
        self.stop_intersections = self._find_special_intersections("stop_intersection")

    def _load_scenario(self, scenario) -> None:
        """
        Loads and processes the scenario data into the appropriate internal structures.
        Args:
            scenario: The scenario object containing map features and dynamic map states.
        This method performs the following steps:
        1. Loads lane, road line, and road edge features from the scenario and stores them in internal dictionaries.
        2. Embeds stop sign data into lane features, marking lanes that require a stop.
        3. Embeds traffic light data into lane features, recording the traffic light states.
        4. Embeds road edge and road line data into lane features, converting polylines to internal point structures.
        The processed data is stored in the following attributes:
        - self.lanecenters: A dictionary mapping lane IDs to LaneCenter objects.
        - self.stop_signs: A set of points representing stop sign positions.
        - self.traffic_lights: A set of points representing traffic light positions.
        """

        # LOAD SCENARIO
        roadedge_roadline_features: dict = {}
        for feature in scenario.map_features:
            feature_data_type = feature.WhichOneof("feature_data")
            if feature_data_type == "lane":
                if LaneType(feature.lane.type) in [LaneType.FREEWAY, LaneType.SURFACE_STREET]:
                    self.lanecenters[feature.id] = LaneCenter(feature.id, feature.lane)
            elif feature_data_type in ["road_line", "road_edge"]:
                roadedge_roadline_features[feature.id] = feature

        # embed stop sign data into lane features
        for feature in scenario.map_features:
            feature_data_type = feature.WhichOneof("feature_data")
            if feature_data_type == "stop_sign":
                for id in feature.stop_sign.lane:
                    if id in self.lanecenters:
                        self.lanecenters[id].needs_stop = True
                self.stop_signs.add(Pt(feature.stop_sign.position.x, feature.stop_sign.position.y))

        # embde traffic light data into lane features
        for i, dynamic_state in enumerate(scenario.dynamic_map_states):
            for lane_state in dynamic_state.lane_states:
                if lane_state.lane in self.lanecenters:
                    # if lane_state.state != 0:  # not unknwon
                    self.lanecenters[lane_state.lane].record_tls[i] = WaymonicTLS(lane_state.state)
                self.traffic_lights.add(Pt(lane_state.stop_point.x, lane_state.stop_point.y))

        # embed road edge and road line data into lane features
        for feature in self.lanecenters.values():
            for boundary in feature.lane.left_boundaries + feature.lane.right_boundaries:
                boundary_type = roadedge_roadline_features[boundary.feature_id].WhichOneof("feature_data")
                if boundary_type == "road_line":
                    polyline = roadedge_roadline_features[boundary.feature_id].road_line.polyline[:]
                elif boundary_type == "road_edge":
                    polyline = roadedge_roadline_features[boundary.feature_id].road_edge.polyline[:]
                boundary.polyline = [Pt(pt.x, pt.y, pt.z) for pt in polyline]

    def _clean_features_information(self) -> None:
        """
        Cleans and processes lane center features information.
        This method performs the following steps:
        1. Deletes features that are too short and are at the fringe of the map.
        2. Cleans up entry and exit lane information for each feature.
        3. Cleans neighborship information for each feature.
        4. Ensures neighborhood information is consistent and in pairs.
        5. Adds bifurcation and merging information for each feature.
        6. Ensures diverge and merge information is consistent and in pairs.
        The method modifies the `lanecenters` attribute of the class, which is a dictionary
        mapping feature IDs to `LaneCenter` objects.
        """

        # I. delete features that are too short && are at fringe of the map
        def _is_fringe_short_feature(feature: LaneCenter):
            return ((not feature.lane.entry_lanes) or (not feature.lane.exit_lanes)) and (
                polyline_length(feature.lane.polyline) < self._LANE_SHORT_THRESHOLD
            )

        features_to_delete = [
            f_id
            for f_id, feature in self.lanecenters.items()
            if _is_fringe_short_feature(feature) or len(feature.lane.polyline) <= 1
        ]
        for f_id in features_to_delete:
            del self.lanecenters[f_id]

        # II. clean up entry/exit lanes
        for feature in self.lanecenters.values():
            self._clean_entryexit_info(feature)

        # III. clean neighborship information
        for feature in self.lanecenters.values():
            self._clean_neighbor_info(feature)

        # an additional step to ensure neighborhood information are in pairs
        # actually the cleaning code above has nothing wrong, but Waymo dataset does have bug on neighborhood data
        for f_id, feature in self.lanecenters.items():
            feature.lane.left_neighbors = [
                nb
                for nb in feature.lane.left_neighbors
                if f_id in [nnb.feature_id for nnb in self.lanecenters[nb.feature_id].lane.right_neighbors]
            ]
            feature.lane.right_neighbors = [
                nb
                for nb in feature.lane.right_neighbors
                if f_id in [nnb.feature_id for nnb in self.lanecenters[nb.feature_id].lane.left_neighbors]
            ]

        # IV. add bifurcation and merging info
        for feature in self.lanecenters.values():
            # For the feature's each pair of entry (exit) lanes, record them as a pair of merged (bifurcated) lanes.
            for i in feature.lane.entry_lanes:
                for j in feature.lane.entry_lanes:
                    if i != j:
                        self.lanecenters[i].lane.merge_lanes.add(j)
            for i in feature.lane.exit_lanes:
                for j in feature.lane.exit_lanes:
                    if i != j:
                        self.lanecenters[i].lane.diverge_lanes.add(j)

        # an additional step to ensure diverge/merge information are in pairs
        for f_id, feature in self.lanecenters.items():
            feature.lane.diverge_lanes = set(
                id for id in feature.lane.diverge_lanes if f_id in self.lanecenters[id].lane.diverge_lanes
            )
            feature.lane.merge_lanes = set(
                id for id in feature.lane.merge_lanes if f_id in self.lanecenters[id].lane.merge_lanes
            )

    def _clean_entryexit_info(self, feature: LaneCenter) -> None:
        """
        Clean entry and exit lanes data for a given feature.
        This method performs the following operations:
        1. Removes entry/exit lanes that do not exist in the current lanecenters.
        2. Removes entry/exit lanes whose endpoints are too far away from the feature.
        Args:
            feature (LaneCenter): The lane center feature whose entry and exit lanes need to be cleaned.
        Returns:
            None
        """

        # Only keep feature ids that are recorded in this conversion
        feature.lane.entry_lanes = [id for id in feature.lane.entry_lanes if id in self.lanecenters.keys()]
        feature.lane.exit_lanes = [id for id in feature.lane.exit_lanes if id in self.lanecenters.keys()]

        # Remove entry/exit lanes that end points are too far away from this feature
        feature.lane.entry_lanes = [
            id
            for id in feature.lane.entry_lanes
            if distance_between_points(feature.lane.polyline[0], self.lanecenters[id].lane.polyline[-1])
            < self._POINT_CLOSE_THRESHOLD
        ]
        feature.lane.exit_lanes = [
            id
            for id in feature.lane.exit_lanes
            if distance_between_points(feature.lane.polyline[-1], self.lanecenters[id].lane.polyline[0])
            < self._POINT_CLOSE_THRESHOLD
        ]

    def _clean_neighbor_info(self, feature: LaneCenter) -> None:
        """
        Clean neighborship data for a given feature.
        This method processes the neighbor information of a LaneCenter feature to determine the type of each neighbor and update the feature's neighbor lists accordingly.
        Args:
            feature (LaneCenter): The LaneCenter feature whose neighbor information is to be cleaned.
        The method performs the following steps:
        1. Iterates through the left and right neighbors of the feature.
        2. Determines the type of each neighbor using the _neighbor_type method.
        3. Keeps the neighbor in the list if it is a "real" neighbor or a parallel bifurcated/merged neighbor.
        4. Moves bifurcated or merged neighbors to the feature's diverge_lanes or merge_lanes sets.
        5. Updates the feature's left_neighbors and right_neighbors lists with the cleaned neighbor information.
        """

        def _return_new_neighbor_list(neighbor_list: list[Neighbor]) -> list[Neighbor]:
            new_neighbor_list: list[Neighbor] = []
            for neighbor in neighbor_list:
                if neighbor.feature_id not in self.lanecenters.keys():
                    continue
                nbr_type = self._neighbor_type(feature, self.lanecenters[neighbor.feature_id], neighbor)
                if nbr_type in ["real", "bifurcated-parallel", "merged-parallel"]:
                    new_neighbor_list.append(neighbor)
                if nbr_type in ["bifurcated", "bifurcated-parallel"]:
                    feature.lane.diverge_lanes.add(neighbor.feature_id)
                if nbr_type in ["merged", "merged-parallel"]:
                    feature.lane.merge_lanes.add(neighbor.feature_id)

            return new_neighbor_list

        feature.lane.left_neighbors = _return_new_neighbor_list(feature.lane.left_neighbors)
        feature.lane.right_neighbors = _return_new_neighbor_list(feature.lane.right_neighbors)

    def _neighbor_type(self, feature1: LaneCenter, feature2: LaneCenter, neighbor: Neighbor) -> str:
        """
        Determine the type of neighbor relationship between two lane features.

        This method evaluates whether two lane features (feature1 and feature2) are considered
        "real neighbors" based on their geometric properties and relative positions. The possible
        return values are:
        - "real": The features are considered real neighbors.
        - "bifurcated": The features are bifurcated neighbors.
        - "merged": The features are merged neighbors.
        - "bifurcated-parallel": The features are bifurcated and parallel.
        - "merged-parallel": The features are merged and parallel.
        - "other": The features do not fit into any of the above categories.

        Args:
            feature1 (LaneCenter): The first lane feature.
            feature2 (LaneCenter): The second lane feature.
            neighbor (Neighbor): The neighbor relationship information between the two features.

        Returns:
            str: The type of neighbor relationship.
        """

        pt11 = feature1.lane.polyline[0]
        pt12 = feature1.lane.polyline[-1]
        pt21 = feature2.lane.polyline[0]
        pt22 = feature2.lane.polyline[-1]
        line1 = [[pt11.x, pt11.y], [pt12.x, pt12.y]]
        line2 = [[pt21.x, pt21.y], [pt22.x, pt22.y]]
        parallel: bool = two_lines_parallel(line1, line2, self._LINE_PARALLEL_THRESHOLD)

        start2start_dis: float = distance_between_points(feature1.lane.polyline[0], feature2.lane.polyline[0])
        end2end_dis: float = distance_between_points(feature1.lane.polyline[-1], feature2.lane.polyline[-1])

        def distance_level(dis: float):
            if dis < 1:
                return "low"
            elif dis < 5:
                return "mid"
            else:
                return "high"

        distance_levels = [distance_level(start2start_dis), distance_level(end2end_dis)]
        if parallel:
            if "low" in distance_levels:
                if distance_levels[0] == "low":
                    return "bifurcated-parallel"
                else:
                    return "merged-parallel"

            elif (
                (neighbor.self_end_index - neighbor.self_start_index) / len(feature1.lane.polyline)
                > self._OVERLAP_PROPORTION_THRESHOLD
            ) or (
                (neighbor.neighbor_end_index - neighbor.neighbor_start_index) / len(feature2.lane.polyline)
                > self._OVERLAP_PROPORTION_THRESHOLD
            ):
                return "real"
            else:
                return "other"

        else:
            if distance_levels[0] in ["low", "mid"]:
                return "bifurcated"
            elif distance_levels[1] in ["low", "mid"]:
                return "merged"
            else:
                return "other"

    def _clean_boundary_info(self, feature: LaneCenter) -> None:
        """
        Cleans up the boundary information of a given LaneCenter feature.
        This method processes the left and right boundaries of the lane associated
        with the provided feature. It uses the `_reconstruct_boundary` method to
        determine if a boundary should be kept or discarded based on certain criteria.
        Args:
            feature (LaneCenter): The lane center feature whose boundaries need to be cleaned.
        Notes:
            - The criteria for cleaning boundaries is determined by the `_reconstruct_boundary` method.
            - Only boundaries with more than one point in their polyline are retained.
        """

        def _clean_boundary_list(boundary_list: list[Boundary], side: str) -> list[Boundary]:
            new_boundaries: list[Boundary] = []
            for boundary in boundary_list:
                reconstruct_result = self._reconstruct_boundary(boundary, feature, side)
                if reconstruct_result:
                    for boundary in reconstruct_result:
                        if len(boundary.polyline) > 1:
                            new_boundaries.append(boundary)
            return new_boundaries

        feature.lane.left_boundaries = _clean_boundary_list(feature.lane.left_boundaries, "left")
        feature.lane.right_boundaries = _clean_boundary_list(feature.lane.right_boundaries, "right")

    def _reconstruct_boundary(
        self, boundary: Boundary, feature: LaneCenter, side: str
    ) -> Union[list[Boundary], None]:
        """
        Reconstructs the boundary data for easier computation based on the given boundary and its associated feature.
        Args:
            boundary (Boundary): The boundary object to be reconstructed.
            feature (LaneCenter): The associated feature containing lane information.
            side (str): Indicates the side of the boundary, either 'left' or 'right'.

        Returns:
            Union[list[Boundary], None]: Returns a list of new boundary objects if the boundary is valid, otherwise returns None.

        The function performs the following steps:
        1. Adjusts the lane start and end indices based on a threshold.
        2. Finds the nearest points on the boundary polyline corresponding to the adjusted lane start and end indices.
        3. Extracts the boundary polyline segment based on the boundary type and side.
        4. Constructs new boundary objects and returns them.

        Notes:
            - If the boundary is considered invalid (e.g., start and end indices are too close), the function returns None.
            - The function handles both road edges and road lines differently based on the boundary type.
        """

        lane_start_index = boundary.lane_start_index
        lane_end_index = boundary.lane_end_index

        if lane_end_index - lane_start_index <= self._INDEX_SNAP_THRESHOLD:
            return None

        # I. adjust lane start index and end index
        if lane_start_index < self._INDEX_SNAP_THRESHOLD:
            lane_start_index = 0
        if lane_end_index > len(feature.lane.polyline) - self._INDEX_SNAP_THRESHOLD:
            lane_end_index = len(feature.lane.polyline) - 1

        # II. find boundary start index and end index
        boundary_start_idx = find_polyline_nearest_point(
            boundary.polyline, feature.lane.polyline[lane_start_index]
        )
        boundary_end_idx = find_polyline_nearest_point(
            boundary.polyline, feature.lane.polyline[lane_end_index]
        )
        if boundary_start_idx < self._INDEX_SNAP_THRESHOLD:
            boundary_start_idx = 0
        if boundary_end_idx > len(boundary.polyline) - self._INDEX_SNAP_THRESHOLD:
            boundary_end_idx = len(boundary.polyline) - 1
        if boundary_start_idx == boundary_end_idx:
            return None

        # III. extract boundary polyline segment
        # if it is a road edge
        if boundary.type == 0:
            if side == "left":
                # abnormal
                if boundary_start_idx < boundary_end_idx:

                    polyline1 = boundary.polyline[: boundary_start_idx + 1]
                    polyline2 = boundary.polyline[boundary_end_idx:]

                    max_closeset_idx = find_polyline_nearest_point(
                        feature.lane.polyline, boundary.polyline[-1]
                    )
                    zero_closest_idx = find_polyline_nearest_point(
                        feature.lane.polyline, boundary.polyline[0]
                    )

                    boundary1 = Boundary(
                        zero_closest_idx, lane_start_index, boundary.type, boundary.feature_id, polyline1
                    )
                    boundary2 = Boundary(
                        lane_end_index, max_closeset_idx, boundary.type, boundary.feature_id, polyline2
                    )

                    return [boundary1, boundary2]
                # normal
                else:
                    polyline = boundary.polyline[boundary_end_idx : boundary_start_idx + 1][::-1]
                    new_boundary = Boundary(
                        lane_start_index, lane_end_index, boundary.type, boundary.feature_id, polyline
                    )
                    return [new_boundary]

            else:
                # normal
                if boundary_start_idx < boundary_end_idx:
                    polyline = boundary.polyline[boundary_start_idx : boundary_end_idx + 1]
                    new_boundary = Boundary(
                        lane_start_index, lane_end_index, boundary.type, boundary.feature_id, polyline
                    )
                    return [new_boundary]
                # abnormal
                else:
                    polyline1 = boundary.polyline[boundary_start_idx:]
                    polyline2 = boundary.polyline[: boundary_end_idx + 1]

                    max_closeset_idx = find_polyline_nearest_point(
                        feature.lane.polyline, boundary.polyline[-1]
                    )
                    zero_closest_idx = find_polyline_nearest_point(
                        feature.lane.polyline, boundary.polyline[0]
                    )

                    boundary1 = Boundary(
                        zero_closest_idx, lane_end_index, boundary.type, boundary.feature_id, polyline1
                    )
                    boundary2 = Boundary(
                        max_closeset_idx, lane_end_index, boundary.type, boundary.feature_id, polyline2
                    )

                    return [boundary1, boundary2]
        # if it is road line
        else:
            if boundary_start_idx < boundary_end_idx:
                polyline = boundary.polyline[boundary_start_idx : boundary_end_idx + 1]

            elif boundary_end_idx < boundary_start_idx:
                polyline = boundary.polyline[boundary_end_idx : boundary_start_idx + 1][::-1]
            new_boundary = Boundary(
                lane_start_index, lane_end_index, boundary.type, boundary.feature_id, polyline
            )
            return [new_boundary]

    def _find_special_intersections(self, type: str) -> list[list[int]]:
        """
        Identifies and returns groups of lane center IDs that form special intersections based on the specified type.
        Args:
            type (str): The type of intersection to find. Can be either "signalized_intersection" or "stop_intersection".
        Returns:
            list[list[int]]: A list of lists, where each inner list contains lane center IDs that form a special intersection.
        The function performs the following steps:
        1. Defines a helper function `_is_connection_group` to check if a group of lane centers forms a connection group.
        2. Initializes a Union-Find data structure to group lane centers based on their connections.
        3. Iterates through lane centers and their neighbors to form initial groups.
        4. Further refines the groups by considering external lane centers with both exit and entry lanes.
        5. Filters the groups based on the specified intersection type using predefined criteria.
        Note:
            - The function relies on several instance attributes such as `self.lanecenters`, `self._POINT_CLOSE_THRESHOLD`,
              `self._signalized_intersection_criteria`, and `self._stop_intersection_criteria`.
            - The `UnionFind` class and `real_neighbor_type` function are assumed to be defined elsewhere in the codebase.
        """

        def _is_connection_group(elements: list[int]) -> bool:
            if any((element not in self.lanecenters.keys()) for element in elements):
                return False
            if len(elements) <= 1:
                return False
            for id in elements:
                if self.lanecenters[id].lane.diverge_lanes or self.lanecenters[id].lane.merge_lanes:
                    return True
            return False

        # round 1
        uf = UnionFind(max(self.lanecenters.keys()) + 1)

        for lc_id, lanecenter in self.lanecenters.items():
            for neighbor in lanecenter.lane.left_neighbors + lanecenter.lane.right_neighbors:
                if (
                    real_neighbor_type(
                        lanecenter.lane.polyline,
                        self.lanecenters[neighbor.feature_id].lane.polyline,
                        POINT_CLOSE_THRESHOLD=self._POINT_CLOSE_THRESHOLD,
                    )
                    == "complete"
                ):
                    uf.union(lanecenter.id, neighbor.feature_id)

            for lc_id in lanecenter.lane.diverge_lanes:
                uf.union(lanecenter.id, lc_id)
            for lc_id in lanecenter.lane.merge_lanes:
                uf.union(lanecenter.id, lc_id)

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if _is_connection_group(elements)]

        # round2
        internal_lcs = [id for group in connection_groups for id in group]
        external_lcs = set(self.lanecenters.keys()).difference(internal_lcs)
        for id in external_lcs:
            if self.lanecenters[id].lane.exit_lanes and self.lanecenters[id].lane.entry_lanes:
                exit_id = self.lanecenters[id].lane.exit_lanes[0]
                entry_id = self.lanecenters[id].lane.entry_lanes[0]
                if uf.find(exit_id) == uf.find(entry_id):
                    uf.union(id, exit_id)
                    uf.union(id, entry_id)

        uf_groups = uf.form_groups()
        connection_groups = [elements for elements in uf_groups if _is_connection_group(elements)]

        # filter out special intersections
        if type == "signalized_intersection":
            filter_criteria = self._signalized_intersection_criteria
        else:
            filter_criteria = self._stop_intersection_criteria
        filtered_groups = [elements for elements in connection_groups if filter_criteria(elements)]
        return filtered_groups

    def _signalized_intersection_criteria(self, lc_group: list[int]):
        """
        Determines if a lane change group meets the criteria for a signalized intersection.

        Args:
            lc_group (list[int]): A list of lane center IDs representing the lane change group.

        Returns:
            bool: True if the lane change group meets all criteria for a signalized intersection, False otherwise.

        Criteria:
            1. The lane change group must contain at least 4 lane centers.
            2. At least one lane center in the group must have a traffic light state that is not ABSENT.
            3. No lane center in the group should have multiple entry lanes where none of the entry lanes are part of the lane change group.
        """
        criteria1 = len(lc_group) >= 4
        criteria2 = any(
            tl_state != WaymonicTLS.ABSENT for id in lc_group for tl_state in self.lanecenters[id].record_tls
        )
        criteria3 = not any(
            len(self.lanecenters[id].lane.entry_lanes) > 1
            and not any(entry_lane_id in lc_group for entry_lane_id in self.lanecenters[id].lane.entry_lanes)
            for id in lc_group
        )
        return criteria1 and criteria2 and criteria3

    def _stop_intersection_criteria(self, lc_group: list[int]):
        """
        Determines if a lane center group meets the criteria for a stop intersection.

        Args:
            lc_group (list[int]): A list of lane center IDs.

        Returns:
            bool: True if the lane center group meets all the stop intersection criteria, False otherwise.

        Criteria:
            1. The length of the lane center group must be at least 3.
            2. At least one lane center in the group must require a stop.
            3. None of the lane centers in the group should have more than one entry lane.
        """
        criteria1 = len(lc_group) >= 3
        criteria2 = any(self.lanecenters[id].needs_stop for id in lc_group)
        criteria3 = not any(len(self.lanecenters[id].lane.entry_lanes) > 1 for id in lc_group)
        return criteria1 and criteria2 and criteria3
