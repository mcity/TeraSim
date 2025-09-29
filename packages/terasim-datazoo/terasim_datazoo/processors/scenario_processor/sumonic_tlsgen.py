from typing import Union
from pathlib import Path
import os

from terasim_datazoo.core.sumo import *
from .traffic_lights import *


class SumonicTLSGenerator:
    def __init__(
        self, scenario, features: dict[int, LaneCenter], nodes: dict[int, Node], edges: dict[int, Edge]
    ) -> None:
        self.scenario = scenario
        self.features = features
        self.nodes = nodes
        self.edges = edges

        self.T: int = len(scenario.timestamps_seconds)

    def generate_sumonic_tls(
        self,
        veh_states_file: Union[Path, None] = None,
        start_step: int = 0,
        end_step: Union[int, None] = None,
        end_strategy: str = "extend",
        generator_config: dict = {},
    ):
        """
        Generates sumonic traffic light programs for the intersections in the scenario.
        Args:
            veh_states_file (Union[Path, None], optional): Path to the file containing vehicle states. If None, vehicle states will be assigned and optionally saved to this file. Defaults to None.
            start_step (int, optional): The starting step for generating the traffic light sequence. Defaults to 0.
            end_step (Union[int, None], optional): The ending step for generating the traffic light sequence. If None, the sequence will be generated until the end of the simulation. Defaults to None.
            end_strategy (str, optional): Strategy to use when the end step is reached. Options are "extend" or other strategies defined in the implementation. Defaults to "extend".
            generator_config (dict, optional): Configuration dictionary for the TLSGenerator. Defaults to an empty dictionary.
        Returns:
            list: A list of intersections with generated traffic light programs.
        """
        
        print("Generating sumonic traffic light programs...")

        intersections: list[list[list[ApproachingLane]]] = []

        # I. generate a lanecenter matrix & assign vehicles to lanes
        if any(node.type == self._TL_TYPE for node in self.nodes.values()):
            # only do this if there is any signalized intersections. since this step is time consuming
            if not veh_states_file or not os.path.exists(Path(veh_states_file)):
                lane_center_matrix, laneid_to_row, row_to_laneid = self._form_sumonic_lane_matrix()
                veh_assignment = assign_veh_states_to_lane(
                    self.scenario.tracks, lane_center_matrix, row_to_laneid, start_step=0, end_step=self.T
                )
                if veh_states_file:
                    save_veh_states_assignment(veh_assignment, veh_states_file)
            else:
                veh_assignment = load_veh_states_assignment(veh_states_file)

        # generate program for each intersection
        for node in self.nodes.values():
            if node.type != self._TL_TYPE:
                continue

            # 1. turn the intersection into a general intersection structure
            intersection = self._form_general_intersection_sumonic(node, veh_assignment)
            if len(intersection) not in [3, 4]:
                continue

            # 2. generate program
            tls_generator = TLSGenerator(self.T, **generator_config)
            tls_sequence: list[list[dict[tuple, TLS]]] = tls_generator.gen_tls_period(
                intersection, start_step=start_step, end_step=end_step
            )

            # 3. reinterpret
            self._embed_tls_data(intersection, tls_sequence, node, end_strategy=end_strategy)
            intersections.append(intersection)

        return intersections

    def _form_sumonic_lane_matrix(self) -> tuple[np.ndarray, dict[str, int], dict[int, str]]:
        """
        Forms a matrix representing the lanes and connections in the SUMO network.

        This method constructs a 3D NumPy array where each row corresponds to a lane or connection,
        and each column represents a point in the lane or connection's shape. The third dimension
        holds the coordinates of these points. Additionally, it creates mappings between lane/connection
        IDs and their corresponding indices in the matrix.

        Returns:
            tuple: A tuple containing:
                - np.ndarray: A 3D array with shape (total_rows, max_col, 3) filled with lane and connection points.
                - dict[str, int]: A dictionary mapping lane/connection IDs to their row indices in the matrix.
                - dict[int, str]: A dictionary mapping row indices in the matrix to their corresponding lane/connection IDs.
        """

        lane_id_to_idx: dict[str, int] = {}
        idx_to_lane_id: dict[int, str] = {}

        total_rows = 0
        max_col = 0
        for edge in self.edges.values():
            for lane in edge.lanes:
                if lane.type == LaneTypeSUMO.NORMAL:
                    total_rows += 1
                    max_col = max(max_col, len(lane.shape))
        for node in self.nodes.values():
            for conn in node.connections:
                if conn.shape:
                    total_rows += 1
                    max_col = max(max_col, len(conn.shape))

        lane_center_matrix: np.ndarray = np.full(
            shape=(total_rows, max_col, 3),
            fill_value=np.inf,
            dtype=np.float64,
        )

        idx = 0
        for edge in self.edges.values():
            for i, lane in enumerate(edge.lanes):
                if lane.type == LaneTypeSUMO.NORMAL:
                    lane_center_matrix[idx, : len(lane.shape)] = np.array([pt.to_list() for pt in lane.shape])
                    lane_id_to_idx[self.id_by_lane(edge.id, i)] = idx
                    idx_to_lane_id[idx] = self.id_by_lane(edge.id, i)
                    idx += 1
        for node in self.nodes.values():
            for conn in node.connections:
                if conn.shape:
                    lane_center_matrix[idx, : len(conn.shape)] = np.array(
                        [pt.to_list() for pt in conn.shape]
                    )
                    lane_id_to_idx[self.id_by_conn(conn)] = idx
                    idx_to_lane_id[idx] = self.id_by_conn(conn)
                    idx += 1

        return lane_center_matrix, lane_id_to_idx, idx_to_lane_id

    def _form_general_intersection_sumonic(
        self, node: Node, veh_assignment: dict[int, list[dict[int, VehicleState]]]
    ) -> list[list[ApproachingLane]]:
        """
        Forms a general intersection representation for SUMO (Simulation of Urban MObility) using the provided node and vehicle assignment data.

        Args:
            node (Node): The node representing the intersection in the SUMO network.
            veh_assignment (dict[int, list[dict[int, VehicleState]]]): A dictionary mapping lane IDs to lists of vehicle states.

        Returns:
            list[list[ApproachingLane]]: A list of lists, where each inner list contains ApproachingLane objects representing the lanes approaching the intersection.
        """

        approaching_lanes: list[ApproachingLane] = []
        for edge_id in node.incoming_SUMO_edges:
            for i, lane in enumerate(self.edges[edge_id].lanes):
                if lane.type != LaneTypeSUMO.NORMAL:
                    continue
                approaching = ApproachingLane(
                    shape=lane.shape, record_vehs=veh_assignment[self.id_by_lane(edge_id, i)]
                )
                for conn in node.connections:
                    if conn.from_edge == edge_id and conn.from_lane == i:
                        injunction_lane = InJunctionLane(
                            shape=conn.shape,
                            record_tls=conn.tl_state_record,
                            record_vehs=veh_assignment[self.id_by_conn(conn)],
                            id=f"{conn.from_edge}_{conn.from_lane}_{conn.to_edge}_{conn.to_lane}",
                        )
                        approaching.injunction_lanes.append(injunction_lane)
                approaching_lanes.append(approaching)

        intersection = group_lanes_into_ways(approaching_lanes)
        return intersection

    @staticmethod
    def id_by_lane(edge_id: int, i) -> str:
        return f"lane-{edge_id}-{i}"

    @staticmethod
    def id_by_conn(connection: Connection) -> str:
        return f"{connection.from_edge}-{connection.from_lane}-{connection.to_edge}-{connection.to_lane}"

    def _embed_tls_data(
        self,
        intersection: list[list[ApproachingLane]],
        tls_sequence: list[list[dict[tuple, TLS]]],
        node: Node,
        end_strategy: str = "extend",
    ):
        """
        Embeds traffic light system (TLS) data into a node's traffic light program.
        Args:
            intersection (list[list[ApproachingLane]]): A list of lists containing approaching lanes for each intersection.
            tls_sequence (list[list[dict[tuple, TLS]]]): A list of lists containing TLS states for each intersection.
            node (Node): The node object representing the intersection.
            end_strategy (str, optional): Strategy for handling the end of the TLS sequence. Defaults to "extend".
        Returns:
            None
        """

        states_list: list[tuple[list[dict[tuple, TLS]], int]] = []
        prev_state = None
        for tls_state in tls_sequence:
            if prev_state == None or tls_state != prev_state:
                states_list.append([tls_state, 1])
            else:
                states_list[-1][1] += 1
            prev_state = tls_state

        for i, (tls_state, duration) in enumerate(states_list):
            sumotlphase = SumoTLPhase()
            sumotlphase.duration = duration / 10
            if i == len(states_list) - 1 and end_strategy == "extend":
                sumotlphase.duration = 2000

            unprotected_left = has_unprotected_left_turns(tls_state)
            for j, approach in enumerate(intersection):
                for lane in approach:
                    for conn in lane.injunction_lanes:
                        phase = next(item for item in tls_state[j].keys() if conn.direction in item)
                        state = tls_state[j][phase]
                        sumonic_state = self._reinterpret_into_sumonic_state(
                            state, conn.direction, unprotected_left, node.type
                        )
                        sumotlphase.states.append(sumonic_state)
                        sumotlphase.connections.append(tuple(map(int, conn.id.split("_"))))
            
            # add red light for sidewalks
            sumotlphase.states.extend([SumonicTLS.RED] * (len(node.crossings)+100-len(sumotlphase.states)))

            node.tl_program.append(sumotlphase)


    @staticmethod
    def _reinterpret_into_sumonic_state(
        state: TLS, direction: Direction, unprotected_left: bool, node_type: str
    ) -> SumonicTLS:
        """
        Reinterprets a given traffic light state into a Sumonic traffic light state.
        Args:
            state (TLS): The current traffic light state.
            direction (Direction): The direction of the traffic.
            unprotected_left (bool): Indicates if the left turn is unprotected.
            node_type (str): The type of the traffic light node.
        Returns:
            SumonicTLS: The reinterpreted Sumonic traffic light state.
        """
        
        def _general_lut(state):
            table = {TLS.UNKNOWN: SumonicTLS.OFF_MINOR, TLS.RED: SumonicTLS.RED, TLS.YELLOW: SumonicTLS.YELLOW, TLS.GREEN: SumonicTLS.GREEN}
            return table[state]

        if state == TLS.GREEN and direction == Direction.L and unprotected_left:
            sumonic_state = SumonicTLS.GREEN_MINOR
        elif state == TLS.RED and direction == Direction.R and node_type == "traffic_light_right_on_red":
            sumonic_state = SumonicTLS.GREEN_RIGHT
        else:
            sumonic_state = _general_lut(state)
        return sumonic_state
