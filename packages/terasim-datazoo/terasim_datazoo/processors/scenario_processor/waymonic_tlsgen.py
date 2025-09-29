from typing import Union
from pathlib import Path
import os
import json
from waymo_open_dataset.protos import scenario_pb2, map_pb2

from terasim_datazoo.core.waymo import *
from .traffic_lights import *
from terasim_datazoo.utils import distance_between_points

class WaymonicTLSGenerator:
    def __init__(self, scenario, lanecenters: dict[int, LaneCenter], signalized_intersections) -> None:
        self.scenario = scenario
        self.lanecenters: dict[int, LaneCenter] = lanecenters
        self.signalized_intersections: list[list[int]] = signalized_intersections
        self.T: int = len(scenario.timestamps_seconds)

    def generate_waymonic_tls(
        self,
        veh_states_file: Union[Path, None] = None,
        start_step: int = 0,
        end_step: Union[int,None] = None,
        include_bike_lane: bool = False,
        return_data: str = "intersections",
        generator_config: dict = {},
    ):
        """
        Generates Waymonic traffic light programs for signalized intersections.

        Args:
            veh_states_file (Union[Path, None], optional): Path to the vehicle states file. If None, vehicle states will be assigned to lanes dynamically. Defaults to None.
            start_step (int, optional): The starting step for traffic light generation. Defaults to 0.
            end_step (Union[int, None], optional): The ending step for traffic light generation. If None, it will generate until the end. Defaults to None.
            include_bike_lane (bool, optional): If True, includes bike lane traffic light data. Defaults to False.
            return_data (str, optional): Specifies the type of data to return. Options are "intersections", "dynamic_states", "by_head", "by_time", or a tuple of all. Defaults to "intersections".
            generator_config (dict, optional): Configuration dictionary for the traffic light sequence generator. Defaults to {}.

        Returns:
            Union[list, dict, tuple]: Depending on the `return_data` parameter, returns one of the following:
            - list: List of intersections.
            - list: List of dynamic map states.
            - dict: Dictionary of traffic light data by head.
            - list: List of traffic light data by time.
            - tuple: Tuple containing intersections, dynamic map states, traffic light data by time, and traffic light data by head.
        """
        print("Generating waymonic traffic light programs...")

        intersections: list[list[list[ApproachingLane]]] = []
        tls_data_by_head: dict[int, dict] = {}
        tls_data_by_time = [[] for _ in range(self.T)]
        dynamic_map_states = [scenario_pb2.DynamicMapState() for _ in range(self.T)]

        # I. generate a lanecenter matrix & assign vehicles to lanes
        if len(self.signalized_intersections):
            # only do this if there is any signalized intersections. since this step is time consuming
            if not veh_states_file or not os.path.exists(Path(veh_states_file)):
                lane_center_matrix, lc_id_to_row, row_to_lc_id = self._form_waymonic_lanecenter_matrix()
                veh_assignment = assign_veh_states_to_lane(
                    self.scenario.tracks, lane_center_matrix, row_to_lc_id
                )
                if veh_states_file:
                    save_veh_states_assignment(veh_assignment, veh_states_file)
            else:
                veh_assignment = load_veh_states_assignment(veh_states_file)

        # II. generate program for each intersection
        for intersection_ids in self.signalized_intersections:
            # 2.1 form a general intersection
            intersection = self._form_general_intersection_waymonic(intersection_ids, veh_assignment)
            if len(intersection) not in [3, 4]:
                continue

            # 2.2. generate program
            tls_generator = TLSGenerator(self.T, **generator_config)
            tls_sequence: list[list[dict[tuple, TLS]]] = tls_generator.gen_tls_period(
                intersection, start_step=start_step, end_step=end_step
            )

            # 2.3 from the generated tls_sequence, convert to various data type, including:
            # [intersection itself] / [dictionary data] / [dynamic state proto]
            self._reinterpret_tls_data(
                intersection, tls_sequence, tls_data_by_time, tls_data_by_head, dynamic_map_states
            )
            intersections.append(intersection)  # [intersection itself]

        if include_bike_lane:
            self._fix_bike_lane_tl_data(tls_data_by_head, dynamic_map_states)

        # III. return information with the specified type
        if return_data == "intersections":
            return intersections
        elif return_data == "dynamic_states":
            return dynamic_map_states
        elif return_data == "by_head":
            return tls_data_by_head
        elif return_data == "by_time":
            return tls_data_by_time
        else:
            return (intersections, dynamic_map_states, tls_data_by_time, tls_data_by_head)

    def _form_waymonic_lanecenter_matrix(self):
        """
        Form an ndarray where each row stores the shape of a feature lane in self.features.
        
        The resulting array has the dimensions (# of features, max(length of features), 3).
        Each row corresponds to a feature lane, and each feature lane is represented by a sequence of 3D points (x, y, z).

        Returns:
            tuple: A tuple containing:
                - lane_center_matrix (np.ndarray): The ndarray representing the lane centers.
                - lc_id_to_idx (dict): A dictionary mapping feature IDs to their corresponding row indices in the array.
                - idx_to_lc_id (dict): A dictionary mapping row indices in the array to their corresponding feature IDs.
        """
       
        lane_center_matrix: np.ndarray = np.full(
            shape=(
                len(self.lanecenters),
                np.max([len(lanecenter.lane.polyline) for lanecenter in self.lanecenters.values()]),
                3,
            ),
            fill_value=np.inf,
            dtype=np.float64,
        )

        lc_id_to_idx = {f_id: i for i, f_id in enumerate(self.lanecenters.keys())}
        idx_to_lc_id = {i: f_id for i, f_id in enumerate(self.lanecenters.keys())}
        for id, lanecenter in self.lanecenters.items():
            lane_center_matrix[lc_id_to_idx[id], : len(lanecenter.lane.polyline)] = np.array(
                [[pt.x, pt.y, pt.z] for pt in lanecenter.lane.polyline]
            )

        return lane_center_matrix, lc_id_to_idx, idx_to_lc_id

    def _form_general_intersection_waymonic(
        self, injunction_ids: list[int], veh_assignment: dict[int, list[dict[int, VehicleState]]]
    ) -> list[list[ApproachingLane]]:
        """
        Forms a general intersection structure for waymonic traffic light generation.

        Args:
            injunction_ids (list[int]): A list of IDs representing the lanecenters involved in the intersection.
            veh_assignment (dict[int, list[dict[int, VehicleState]]]): A dictionary mapping lanecenter IDs to lists of vehicle states.

        Returns:
            list[list[ApproachingLane]]: A nested list where each sublist represents an approaching lane and its corresponding in-junction lanes.
        """

        # 1. the ids of all lanecenters incoming to this intersection
        incoming_ids: set[int] = {
            entry_lane
            for lc_id in injunction_ids
            for entry_lane in self.lanecenters[lc_id].lane.entry_lanes
            if entry_lane not in injunction_ids
        }
        # 2. turn them into a general intersection structure
        approaching_lanes: list[ApproachingLane] = []
        for lc_id in incoming_ids:
            approaching = ApproachingLane(
                shape=self.lanecenters[lc_id].lane.polyline, record_vehs=veh_assignment[str(lc_id)], id=lc_id
            )
            for in_junction_lc_id in self.lanecenters[lc_id].lane.exit_lanes:
                injunction_lane = InJunctionLane(
                    shape=self.lanecenters[in_junction_lc_id].lane.polyline,
                    record_tls=self.lanecenters[in_junction_lc_id].record_tls,
                    record_vehs=veh_assignment[str(in_junction_lc_id)],
                    id=in_junction_lc_id,
                )
                approaching.injunction_lanes.append(injunction_lane)
            approaching_lanes.append(approaching)

        intersection = group_lanes_into_ways(approaching_lanes)
        return intersection

    def _reinterpret_tls_data(
        self,
        intersection: list[list[ApproachingLane]],  # input | output
        tls_sequence: list[list[dict[tuple, TLS]]],  # input
        tls_data_by_time: list[list[dict]],  # output
        tls_data_by_tlhead: dict[int, dict],  # output
        dynamic_map_states: list,  # output
    ) -> None:
        """
        Reinterprets the traffic light sequence data into various formats.

        Args:
            intersection (list[list[ApproachingLane]]): The intersection structure.
            tls_sequence (list[list[dict[tuple, TLS]]]): The traffic light sequence data.
            tls_data_by_time (list[list[dict]]): The traffic light data organized by time.
            tls_data_by_tlhead (dict[int, dict]): The traffic light data organized by traffic light head.
            dynamic_map_states (list): The dynamic map states.

        Returns:
            None
        """

        # for every time step and the corresponding tls state
        for t, tls_state in enumerate(tls_sequence):
            # whether there is unprotected left turn in this step?
            unprotected_left = has_unprotected_left_turns(tls_state)

            for i, approach in enumerate(intersection):
                for lane in approach:
                    direction_set = set(conn.direction for conn in lane.injunction_lanes)
                    phase = next(phase for phase in tls_state[i].keys() if list(direction_set)[0] in phase)
                    state = tls_state[i][phase]
                    arrow_ever = any(
                        st in [WaymonicTLS.ARROW_GO, WaymonicTLS.ARROW_CAUTION, WaymonicTLS.ARROW_STOP]
                        for conn in lane.injunction_lanes
                        for st in conn.record_tls_waymonic
                    )

                    waymonic_state: WaymonicTLS = self._reinterpret_into_waymonic_state(
                        direction_set, state, arrow_ever, unprotected_left
                    )

                    for conn in lane.injunction_lanes:
                        # [intersection itself]
                        conn.new_tls[t] = state
                        #  [tls data by time]
                        tl_head_location = [conn.shape[0].x, conn.shape[0].y, conn.shape[0].z]
                        tl_head_dir = points_to_vector(conn.shape[0], conn.shape[min(3, len(conn.shape) - 1)])
                        tls_data_by_time[t].append(
                            {
                                "feature_id": conn.id,
                                "state": waymonic_state.value,
                                "location": tl_head_location,
                                "tl_head_dir": [tl_head_dir[0], tl_head_dir[1], 0],
                            }
                        )
                        # [dynamic state proto]
                        trafficsignallanestate = map_pb2.TrafficSignalLaneState(
                            lane=int(conn.id), state=int(waymonic_state.value)
                        )
                        trafficsignallanestate.stop_point.CopyFrom(
                            map_pb2.MapPoint(x=conn.shape[0].x, y=conn.shape[0].y, z=conn.shape[0].z)
                        )
                        dynamic_map_states[t].lane_states.append(trafficsignallanestate)

        # [tls data by head]
        for i, approach in enumerate(intersection):
            for lane in approach:
                for conn in lane.injunction_lanes:
                    tls_data_by_tlhead[len(tls_data_by_tlhead)] = {
                        "location": [conn.shape[0].x, conn.shape[0].y, conn.shape[0].z],
                        "tl_head_dir": points_to_vector(
                            conn.shape[0], conn.shape[min(3, len(conn.shape) - 1)]
                        ),
                        "states": [state.value for state in conn.new_tls],
                    }

    @staticmethod
    def _reinterpret_into_waymonic_state(
        direction_set: tuple[Direction], state: TLS, arrow_ever: bool, unprotected_left: bool
    ) -> WaymonicTLS:
        """
        Reinterprets the given traffic light state into a Waymonic traffic light state based on the direction set.

        Args:
            direction_set (tuple[Direction]): A tuple containing the set of directions.
            state (TLS): The current traffic light state.
            arrow_ever (bool): Indicates if there is ever an arrow signal.
            unprotected_left (bool): Indicates if the left turn is unprotected.

        Returns:
            WaymonicTLS: The corresponding Waymonic traffic light state.

        Raises:
            AssertionError: If the direction set does not match any expected pattern.
        """

        def _general_lut(state):
            table = {TLS.RED: WaymonicTLS.STOP, TLS.YELLOW: WaymonicTLS.CAUTION, TLS.GREEN: WaymonicTLS.GO}
            return table[state]

        if Direction.S in direction_set or direction_set == {Direction.R}:
            waymonic_state = _general_lut(state)
        elif direction_set == {Direction.L}:
            waymonic_state = {
                TLS.RED: WaymonicTLS.ARROW_STOP if arrow_ever else WaymonicTLS.STOP,
                TLS.YELLOW: WaymonicTLS.ARROW_CAUTION if arrow_ever else WaymonicTLS.CAUTION,
                TLS.GREEN: (WaymonicTLS.ARROW_GO if arrow_ever and unprotected_left else WaymonicTLS.GO),
            }[state]
        elif direction_set == {Direction.L, Direction.R}:
            waymonic_state = {
                TLS.RED: WaymonicTLS.ARROW_STOP if arrow_ever else WaymonicTLS.STOP,
                TLS.YELLOW: WaymonicTLS.ARROW_CAUTION if arrow_ever else WaymonicTLS.CAUTION,
                TLS.GREEN: WaymonicTLS.ARROW_GO if arrow_ever else WaymonicTLS.GO,
            }[state]
        else:
            assert False

        return waymonic_state

    def _fix_bike_lane_tl_data(self, tls_data_by_head, dynamic_map_states) -> None:

        missing_head_lane = []
        missing_head_pos = []
        for i, dynamic_state in enumerate(self.scenario.dynamic_map_states):
            for lane_state in dynamic_state.lane_states:
                if lane_state.lane in missing_head_lane:
                    continue
                for feature in self.scenario.map_features:
                    feature_data_type = feature.WhichOneof("feature_data")
                    if (
                        feature_data_type == "lane"
                        and LaneType(feature.lane.type) == LaneType.BIKELANE
                        and feature.id == lane_state.lane
                    ):
                        missing_head_lane.append(lane_state.lane)
                        missing_head_pos.append(
                            [lane_state.stop_point.x, lane_state.stop_point.y, lane_state.stop_point.z]
                        )

        for k in range(len(missing_head_lane)):
            closest_tlhead_id = None
            min_dis = np.inf
            missing_pos = missing_head_pos[k]

            for id, information in tls_data_by_head.items():
                this_dis = distance_between_points(
                    Pt(missing_pos[0], missing_pos[1], missing_pos[2]),
                    Pt(information["location"][0], information["location"][1], information["location"][2]),
                )
                if this_dis < min_dis:
                    closest_tlhead_id = id
                    min_dis = this_dis
            if closest_tlhead_id != None:
                for t in range(self.T):
                    trafficsignallanestate = map_pb2.TrafficSignalLaneState(
                        lane=int(missing_head_lane[k]), state=tls_data_by_head[closest_tlhead_id]["states"][t]
                    )
                    trafficsignallanestate.stop_point.CopyFrom(
                        map_pb2.MapPoint(x=missing_pos[0], y=missing_pos[1], z=missing_pos[2])
                    )
                    dynamic_map_states[t].lane_states.append(trafficsignallanestate)
