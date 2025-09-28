import numpy as np
import copy
from typing import Union

from terasim_datazoo.utils import UnionFind, Direction, TLS
from terasim_datazoo.core.intersection import ApproachingLane


class TLSGenerator:
    """
    TLSGenerator is a class responsible for generating traffic light signals for general intersections based on various parameters and states.

    Attributes:
        T (int): Total time steps for the simulation.
        V_GREEN (float): Speed threshold for green light.
        V_RED (float): Speed threshold for red light.
        A_GREEN (float): Acceleration threshold for green light.
        A_RED (float): Acceleration threshold for red light.
        DELTA_T (int): Time interval for state generation.
        THETA (float): Confidence threshold for state estimation.
        W_BIG (float): Weight for high confidence.
        W_SMALL (float): Weight for low confidence.
        SMOOTHING_WIDTH (int): Width for smoothing the traffic light sequence.
        YELLOW_DURATION (int): Duration for yellow light.

    Methods:
        __init__(self, T: int, delta_t: int = 10, smoothing_width: int = 30, yellow_duration: int = 20) -> None:
            Initializes the TLSGenerator with given parameters.

        gen_tls_period(self, intersection: list[list[ApproachingLane]], start_step: int = 0, end_step: Union[int, None] = None) -> list[list[dict[tuple, TLS]]]:
            Generates traffic light signals for a given period.

        gen_tls_one_moment(self, intersection: list[list[ApproachingLane]], curr_step: int = 25, prev_state: Union[list[dict[tuple, TLS]], None] = None) -> list[dict[tuple, TLS]]:
            Generates traffic light signals for a single moment.
    """

    def __init__(self, T: int, delta_t: int = 10, smoothing_width: int = 30, yellow_duration: int = 20) -> None:

        self.T = T
        self.V_GREEN = 3
        self.V_RED = 1
        self.A_GREEN = 0.5
        self.A_RED = -1
        self.DELTA_T = delta_t
        self.THETA = 0.8
        self.W_BIG = 100
        self.W_SMALL = 0.1
        self.SMOOTHING_WIDTH = smoothing_width
        self.YELLOW_DURATION = yellow_duration

    def gen_tls_period(
        self,
        intersection: list[list[ApproachingLane]],
        start_step: int = 0,
        end_step: Union[int, None] = None,
    ) -> list[list[dict[tuple, TLS]]]:
        """
        Generates traffic light states for a given intersection over a specified period.

        Args:
            intersection (list[list[ApproachingLane]]): A list of lists representing the approaching lanes at the intersection.
            start_step (int, optional): The starting time step for generating traffic light states. Defaults to 0.
            end_step (Union[int, None], optional): The ending time step for generating traffic light states. If None, defaults to the total simulation time.

        Returns:
            list[list[dict[tuple, TLS]]]: A list of lists containing dictionaries that map tuples to TLS (Traffic Light State) objects for each time step.
        """

        if len(intersection) not in [2, 3, 4]:  # not a valid intersection
            return []
        if end_step == None:
            end_step = self.T
        tl_state_buff: list[list[dict[tuple, TLS]]] = [None for _ in range(self.T)]

        prev_state: list[dict[tuple, TLS]] = None
        gen_start_step = max(start_step, self.DELTA_T)
        gen_end_step = min(end_step, self.T - self.DELTA_T)
        for time_step in range(gen_start_step, gen_end_step):
            tls_state = self.gen_tls_one_moment(intersection, time_step, prev_state)
            tl_state_buff[time_step] = tls_state
            prev_state = tls_state

        for time_step in range(start_step, gen_start_step):
            tl_state_buff[time_step] = tl_state_buff[gen_start_step]
        for time_step in range(gen_end_step, end_step):
            tl_state_buff[time_step] = tl_state_buff[gen_end_step - 1]

        tl_state_buff[start_step:end_step] = self._smooth_sequence(tl_state_buff[start_step:end_step])
        tl_state_buff[start_step:end_step] = self._add_yellow_light(tl_state_buff[start_step:end_step])

        return tl_state_buff

    def gen_tls_one_moment(
        self,
        intersection: list[list[ApproachingLane]],
        curr_step: int = 25,
        prev_state: Union[list[dict[tuple, TLS]], None] = None,
    ) -> list[dict[tuple, TLS]]:
        
        """
        Generates the traffic light state for a single moment at an intersection.

        Args:
            intersection (list[list[ApproachingLane]]): A list of lists representing the approaching lanes at the intersection.
            curr_step (int, optional): The current time step. Defaults to 25.
            prev_state (Union[list[dict[tuple, TLS]], None], optional): The previous state of the traffic lights. Defaults to None.

        Returns:
            list[dict[tuple, TLS]]: The final state of the traffic lights for the given moment.

        1. Generate a container template for the traffic light state.
        2. Derive the raw state of the traffic lights based on the current step.
        3. Estimate the state of the traffic lights and calculate confidence.
        4. Impute the state by combining raw and estimated states with confidence.
        5. If the intersection has only two lanes, return the imputed state.
        6. Get feasible states for the traffic lights.
        7. Score candidate states based on feasibility and imputed state.
        8. If the previous state is among candidate states, use it as the final state.
        9. Otherwise, fill the right turn signal in the top candidate state and use it as the final state.
        """

        self.container_template = self._gen_tl_state_container(intersection)
        # step 1: raw
        raw_state = self._derive_raw_state(intersection, curr_step)
        # step 2: est
        est_state, confidence = self._derive_estimated_state(intersection, curr_step)
        # step 3: imp
        imp_state, weight = self._derive_imputed_state(raw_state, est_state, confidence)
        if len(intersection) == 2:
            return imp_state
        # step 4: feasible
        feas_states = self._get_feasible_states()
        # step 5: final
        candidate_states = self._score_candidate_states(feas_states, imp_state, weight)

        if prev_state in candidate_states:
            final_state = copy.deepcopy(prev_state)
        else:
            final_state = self._fill_right_turn_signal(copy.deepcopy(candidate_states[0]))

        return final_state

    def _gen_tl_state_container(self, intersection: list[list[ApproachingLane]]) -> list[dict[tuple]]:
        """
        Generates a state container for traffic light states at an intersection.
        Args:
            intersection (list[list[ApproachingLane]]): A list of lists where each sublist represents 
                                approaching lanes at an intersection.
        Returns:
            list[dict[tuple]]: A list of dictionaries where each dictionary corresponds to an approach 
                       at the intersection. Each dictionary maps a tuple of directions (phases) 
                       to None. The phases represent possible traffic light states for the 
                       corresponding approach.
        """
        
        state_container: list[dict] = [None for _ in intersection]
        for i, approach in enumerate(intersection):
            movements = set(conn.direction for lane in approach for conn in lane.injunction_lanes)

            uf = UnionFind(3)
            for lane in approach:
                for conn_i in lane.injunction_lanes:
                    for conn_j in lane.injunction_lanes:
                        uf.union(conn_i.direction.value, conn_j.direction.value)
            phases = [[Direction(i) for i in group] for group in uf.form_groups()]
            phases = [
                tuple(dirs) for dirs in phases if all(dir in movements for dir in dirs)
            ]  # e.g. directions = [(L,), (S,R)]
            state_container[i] = {phase: None for phase in phases}

        return state_container

    def _derive_raw_state(
        self, intersection: list[list[ApproachingLane]], curr_step: int
    ) -> list[dict[tuple, TLS]]:
        """
        Derives the raw state of traffic lights at an intersection for a given simulation step.
        Args:
            intersection (list[list[ApproachingLane]]): A nested list representing the lanes approaching the intersection.
            curr_step (int): The current simulation step.
        Returns:
            list[dict[tuple, TLS]]: A list of dictionaries where each dictionary maps a tuple (representing a phase) to a TLS state.
        Raises:
            AssertionError: If a traffic light state (TLS) record is None for any step within the last 10 steps.
        """

        raw_state: list[dict[tuple, TLS]] = copy.deepcopy(self.container_template)

        for i, approach in enumerate(intersection):
            for lane in approach:
                for conn in lane.injunction_lanes:
                    phase = next(item for item in raw_state[i].keys() if conn.direction in item)
                    for step in range(curr_step, max(0, curr_step - 10) - 1, -1):
                        assert conn.record_tls[step] != None
                        if conn.record_tls[step] not in [TLS.ABSENT, TLS.UNKNOWN]:
                            if raw_state[i][phase] == None:
                                raw_state[i][phase] = TLS(conn.record_tls[step])
                                if conn.record_tls[step] == TLS.YELLOW:
                                    raw_state[i][phase] = TLS.GREEN

        return raw_state

    def _derive_estimated_state(
        self, intersection: list[list[ApproachingLane]], curr_step: int
    ) -> tuple[list[dict[tuple, TLS]], list[dict[tuple, float]]]:
        """
        Derives the estimated state of traffic lights at an intersection for a given simulation step.

        Args:
            intersection (list[list[ApproachingLane]]): A nested list representing the lanes approaching the intersection.
            curr_step (int): The current simulation step.

        Returns:
            tuple[list[dict[tuple, TLS]], list[dict[tuple, float]]]: A tuple containing two lists:
            - The first list contains dictionaries that map tuples (representing phases) to estimated TLS states.
            - The second list contains dictionaries that map tuples (representing phases) to confidence values.
        """

        est_state: list[dict[tuple, TLS]] = copy.deepcopy(self.container_template)
        confidence: list[dict[tuple, float]] = copy.deepcopy(self.container_template)

        for i, approach in enumerate(intersection):
            for phase in est_state[i].keys():
                if phase == (Direction.R,):
                    continue

                mean_acc, mean_spd, sum_f, sum_g, must_green_indicator = self._get_traj_metrics_at_phase(
                    approach, phase, curr_step
                )

                if must_green_indicator:
                    est_state[i][phase] = TLS.GREEN
                    confidence[i][phase] = self.W_BIG
                    continue

                if sum_g >= self.THETA:
                    if mean_spd >= self.V_GREEN:
                        est_state[i][phase] = TLS.GREEN
                        confidence[i][phase] = sum_g
                    elif mean_spd <= self.V_RED:
                        est_state[i][phase] = TLS.RED
                        confidence[i][phase] = sum_g
                if sum_f >= self.THETA:
                    if mean_acc >= self.A_GREEN:
                        est_state[i][phase] = TLS.GREEN
                        confidence[i][phase] = sum_f
                    elif mean_acc <= self.A_RED:
                        est_state[i][phase] = TLS.RED
                        confidence[i][phase] = sum_f

        return est_state, confidence

    def _derive_imputed_state(
        self,
        raw_state: list[dict[tuple, TLS]],
        est_state: list[dict[tuple, TLS]],
        confidence: list[dict[tuple, float]],
    ) -> tuple[list[dict[tuple, TLS]], list[dict[tuple, float]]]:
        """
        Derives the imputed state and corresponding weights for traffic light phases based on raw and estimated states.
        This method uses a heuristic approach to determine the imputed state and weight for each traffic light phase.
        The heuristic prioritizes phases that are confirmed by both raw and estimated states, followed by those confirmed
        only by the estimated state, then those confirmed only by the raw state, and finally those with no confirmation.
        Args:
            raw_state (list[dict[tuple, TLS]]): The raw state of traffic light phases.
            est_state (list[dict[tuple, TLS]]): The estimated state of traffic light phases.
            confidence (list[dict[tuple, float]]): The confidence levels associated with the estimated states.
        Returns:
            tuple[list[dict[tuple, TLS]], list[dict[tuple, float]]]: A tuple containing:
                - imp_state: The imputed state of traffic light phases.
                - weight: The weights associated with the imputed states.
        """

        # heuristics: (raw+est double confirmed) >> (est confirmed only (regardless of raw)) >> (raw confirmed only) >> (none confirmed)
        # magnitude: 100 >> 1 >> 0.1 >> 0
        imp_state: list[dict[tuple, TLS]] = copy.deepcopy(self.container_template)
        weight: list[dict[tuple, float]] = copy.deepcopy(self.container_template)

        for i in range(len(raw_state)):
            for phase in raw_state[i].keys():
                raw_none = raw_state[i][phase] == None
                est_none = est_state[i][phase] == None
                if raw_none and est_none:
                    weight[i][phase] = 0
                elif raw_none and not est_none:
                    imp_state[i][phase] = est_state[i][phase]
                    weight[i][phase] = confidence[i][phase]
                elif not raw_none and est_none:
                    imp_state[i][phase] = raw_state[i][phase]
                    weight[i][phase] = self.W_SMALL
                else:
                    if raw_state[i][phase] == est_state[i][phase]:
                        imp_state[i][phase] = est_state[i][phase]
                        weight[i][phase] = self.W_BIG
                    else:
                        if confidence[i][phase] >= self.THETA:
                            imp_state[i][phase] = est_state[i][phase]
                            weight[i][phase] = confidence[i][phase]
                        else:
                            imp_state[i][phase] = raw_state[i][phase]
                            weight[i][phase] = 0

        return imp_state, weight

    def _get_feasible_states(self) -> list[list[dict[tuple, TLS]]]:
        """
        Generate a list of feasible traffic light states based on the container template.
        This method generates candidate traffic light states for intersections with either 
        3 or 4 incoming directions. The states are generated based on the following rules:
        - For 4 incoming directions:
            - Each direction gets a green light while others are red.
            - Opposite directions get green lights simultaneously.
            - If left and straight directions are not combined, generate states where:
                - Left turns are green and straight/right turns are red.
                - Left turns are red and straight/right turns are green.
        - For 3 incoming directions:
            - Each direction gets a green light while others are red.
            - If straight and left turns are not combined, generate states where:
                - Both straight directions are green and the third direction is red.
                - Both left turns are green and the third direction is red.
            - Generate a state where the main road is fully green.
        Returns:
            list[list[dict[tuple, TLS]]]: A list of candidate traffic light states.
        """


        candidate_states: list[list[dict[tuple, TLS]]] = []

        if len(self.container_template) == 4:
            # incoming
            for i in range(4):
                candidate_state = copy.deepcopy(self.container_template)
                for j in range(4):
                    for phase in candidate_state[j].keys():
                        candidate_state[j][phase] = TLS.GREEN if j == i else TLS.RED
                candidate_states.append(candidate_state)

            # opposites
            for green_group in [[0, 2], [1, 3]]:
                # L green S green
                candidate_state = copy.deepcopy(self.container_template)
                for j in range(4):
                    for phase in candidate_state[j].keys():
                        candidate_state[j][phase] = TLS.GREEN if j in green_group else TLS.RED
                candidate_states.append(candidate_state)

                # L & S either one is green
                if not any(
                    any(
                        Direction.L in phase and Direction.S in phase
                        for phase in self.container_template[i].keys()
                    )
                    for i in green_group
                ):
                    # Left turn green, straight red (precondition: straight and left turn are not combined)
                    candidate_state = copy.deepcopy(self.container_template)
                    for j in range(4):
                        for phase in candidate_state[j].keys():
                            if j in green_group:
                                if Direction.L in phase:
                                    candidate_state[j][phase] = TLS.GREEN
                                elif Direction.S in phase or Direction.R in phase:
                                    candidate_state[j][phase] = TLS.RED
                            else:
                                candidate_state[j][phase] = TLS.RED
                    candidate_states.append(candidate_state)

                    # Left turn red, straight green (precondition: straight and left turn are not combined)
                    candidate_state = copy.deepcopy(self.container_template)
                    for j in range(4):
                        for phase in candidate_state[j].keys():
                            if j in green_group:
                                if Direction.L in phase:
                                    candidate_state[j][phase] = TLS.RED
                                elif Direction.S in phase or Direction.R in phase:
                                    candidate_state[j][phase] = TLS.GREEN
                            else:
                                candidate_state[j][phase] = TLS.RED
                    candidate_states.append(candidate_state)

        elif len(self.container_template) == 3:
            # incoming
            for i in range(3):
                candidate_state = copy.deepcopy(self.container_template)
                for j in range(3):
                    for phase in candidate_state[j].keys():
                        candidate_state[j][phase] = TLS.GREEN if j == i else TLS.RED
                candidate_states.append(candidate_state)

            # opposite (precondition: straight and left turn are not combined)
            if not any(
                any(
                    Direction.S in phase and Direction.L in phase
                    for phase in self.container_template[i].keys()
                )
                for i in [0, 1]
            ):
                # case 1: both straight directions are green
                candidate_state = copy.deepcopy(self.container_template)
                for phase in candidate_state[2].keys():
                    candidate_state[2][phase] = TLS.RED
                for j in [0, 1]:
                    for phase in candidate_state[j].keys():
                        if Direction.L in phase:
                            candidate_state[j][phase] = TLS.RED
                        else:
                            candidate_state[j][phase] = TLS.GREEN
                candidate_states.append(candidate_state)

                # case 2: both left turns are green
                candidate_state = copy.deepcopy(self.container_template)
                for phase in candidate_state[2].keys():
                    candidate_state[2][phase] = TLS.RED
                for j in [0, 1]:
                    for phase in candidate_state[j].keys():
                        if Direction.L in phase:
                            candidate_state[j][phase] = TLS.GREEN
                        else:
                            candidate_state[j][phase] = TLS.RED
                candidate_states.append(candidate_state)

            # opposite: all green
            candidate_state = copy.deepcopy(self.container_template)
            for j in range(3):
                for phase in candidate_state[2].keys():
                    candidate_state[2][phase] = TLS.RED if j == 2 else TLS.GREEN
            candidate_states.append(candidate_state)
        else:
            assert False

        return candidate_states

    def _score_candidate_states(
        self,
        feas_states: list[list[dict[tuple, TLS]]],
        imp_state: list[dict[tuple, TLS]],
        weight: list[dict[tuple, float]],
    ):
        """
        Scores and filters candidate states based on match and conflict scores.
        This function evaluates a list of feasible states against an important state
        and a weight matrix. It calculates match and conflict scores for each feasible
        state, filters the states to find the ones with the highest match score, and
        then further filters to find the ones with the lowest conflict score.
        Args:
            feas_states (list[list[dict[tuple, TLS]]]): A list of feasible states, 
                where each feasible state is a list of dictionaries mapping tuples to TLS objects.
            imp_state (list[dict[tuple, TLS]]): The important state, represented as a list of 
                dictionaries mapping tuples to TLS objects.
            weight (list[dict[tuple, float]]): A weight matrix, represented as a list of 
                dictionaries mapping tuples to float values.
        Returns:
            list[list[dict[tuple, TLS]]]: A list of candidate states that have the highest match 
            score and the lowest conflict score.
        """

        def s_match(feas_state: list[dict[tuple, TLS]]) -> float:
            return sum(
                weight[i][phase]
                for i in range(len(imp_state))
                for phase in imp_state[i].keys()
                if imp_state[i][phase] != None and imp_state[i][phase] == feas_state[i][phase]
            )

        def s_conflict(feas_state: list[dict[tuple, TLS]]) -> float:
            return sum(
                weight[i][phase]
                for i in range(len(imp_state))
                for phase in imp_state[i].keys()
                if imp_state[i][phase] != None and imp_state[i][phase] != feas_state[i][phase]
            )

        # random.shuffle(candidate_states)
        scores = [
            (i, s_match(feas_state), s_conflict(feas_state)) for i, feas_state in enumerate(feas_states)
        ]
        highest_match = max(scores, key=lambda item: item[1])[1]
        filtered_scores = [item for item in scores if item[1] == highest_match]
        lowest_conflict = min(filtered_scores, key=lambda item: item[2])[2]
        filtered_scores = [item for item in filtered_scores if item[2] == lowest_conflict]
        candidate_states = [feas_states[i] for (i, _, _) in filtered_scores]

        return candidate_states

    def _fill_right_turn_signal(self, state: list[dict[tuple, TLS]]) -> list[dict[tuple, TLS]]:
        """
        Fills the right turn signal in the traffic light state.
        This method updates the traffic light state by assigning a signal to the right turn direction
        based on the existing signals for straight or left turn directions.
        Args:
            state (list[dict[tuple, TLS]]): A list of dictionaries representing the traffic light state.
                Each dictionary maps a tuple of directions to a TLS (Traffic Light Signal) object.
        Returns:
            list[dict[tuple, TLS]]: The updated traffic light state with right turn signals filled in.
        """

        for i in range(len(state)):
            if (Direction.R,) in state[i].keys():  # and state_info[i][(R,)] == None:
                straight_phase = next((phase for phase in state[i].keys() if Direction.S in phase), None)
                if straight_phase:
                    state[i][(Direction.R,)] = state[i][straight_phase]
                else:
                    left_phase = next((phase for phase in state[i].keys() if Direction.L in phase), None)
                    if left_phase:
                        state[i][(Direction.R,)] = state[i][left_phase]
        return state

    def _get_traj_metrics_at_phase(
        self, approach: list[ApproachingLane], phase: tuple[int], curr_step: int
    ) -> tuple[float, bool]:
        """
        Calculate trajectory metrics for vehicles at a given traffic light phase.
        Args:
            approach (list[ApproachingLane]): List of approaching lanes.
            phase (tuple[int]): Current traffic light phase.
            curr_step (int): Current simulation step.
        Returns:
            tuple[float, bool]: A tuple containing:
                - mean_acc (float): Weighted mean acceleration of vehicles.
                - mean_spd (float): Weighted mean speed of vehicles.
                - sum_f (float): Logarithm of the sum of weighted acceleration values.
                - sum_g (float): Logarithm of the sum of weighted speed values.
                - is_green_light (bool): Indicator if the light is green based on vehicle movements.
        """

        approaching_lanes = [
            lane for lane in approach if any((conn.direction in phase) for conn in lane.injunction_lanes)
        ]

        traj_records_per_veh: dict[int, list[tuple]] = {}

        def _append(veh_id, pos_idx, speed, acceleration):
            if veh_id not in traj_records_per_veh:
                traj_records_per_veh[veh_id] = []
            traj_records_per_veh[veh_id].append((pos_idx, speed, acceleration))

        for tt in range(curr_step - self.DELTA_T, curr_step + self.DELTA_T):
            for lane in approaching_lanes:
                # vehicles on the approaching lane
                for veh_id, veh_record in lane.record_vehs[tt].items():
                    pos_idx = len(lane.shape) - veh_record.lane_pos_idx - 1
                    _append(veh_id, pos_idx, veh_record.speed, veh_record.acceleration)

                # (does this lane have right turn connection?)
                have_right_turn_conn = any(conn.direction == Direction.R for conn in lane.injunction_lanes)
                # vehicles on the injunction lanes
                for conn in lane.injunction_lanes:
                    for veh_id, veh_state_record in conn.record_vehs[tt].items():
                        pos_idx = -veh_state_record.lane_pos_idx
                        _append(veh_id, pos_idx, veh_state_record.speed, veh_state_record.acceleration)

                        # !! extra hard coded condition
                        # if any vehicle has passed the stopping line a bit and is moving,
                        # this must be a green light
                        # if this incoming lane does not have right turn connection
                        if not have_right_turn_conn and abs(tt - curr_step) <= 2:
                            if (
                                veh_state_record.lane_pos_idx >= 0
                                and veh_state_record.lane_pos_idx < 10
                                and veh_state_record.speed > 0
                            ):
                                return 0, 0, 0, 0, True

        def metrics_per_vehicle(traj_records: list[tuple]) -> tuple[float]:

            pos_idxs, speeds, accelerations = zip(*traj_records)
            f_values = [self.f(d, a) for d, a in zip(pos_idxs, accelerations)]
            g_values = [self.g(d, v) for d, v in zip(pos_idxs, speeds)]

            mean_acc = np.average(accelerations, weights=f_values) if np.sum(f_values) else 0
            max_f = np.max(f_values)
            mean_spd = np.average(speeds, weights=g_values) if np.sum(g_values) else 0
            max_g = np.max(g_values)

            return mean_acc, mean_spd, max_f, max_g

        traj_metrics_per_veh: dict[int, list[tuple[float]]] = {
            veh_id: metrics_per_vehicle(records) for veh_id, records in traj_records_per_veh.items()
        }
        if not traj_metrics_per_veh:
            return 0, 0, 0, 0, False

        accelerations, speeds, f_values, g_values = zip(*list(traj_metrics_per_veh.values()))
        f_values_filtered = [f_values[i] for i in range(len(f_values)) if f_values[i]]
        accelerations_filtered = [accelerations[i] for i in range(len(accelerations)) if f_values[i]]
        g_values_filtered = [g_values[i] for i in range(len(g_values)) if g_values[i]]
        speeds_filtered = [speeds[i] for i in range(len(speeds)) if g_values[i]]
        assert len(accelerations_filtered) == len(f_values_filtered)
        assert len(speeds_filtered) == len(g_values_filtered)

        sum_f = np.log(1 + np.sum(f_values_filtered))
        mean_acc = np.average(accelerations_filtered, weights=f_values_filtered) if sum_f else 0
        sum_g = np.log(1 + np.sum(g_values_filtered))
        mean_spd = np.average(speeds_filtered, weights=g_values_filtered) if sum_g else 0

        return mean_acc, mean_spd, sum_f, sum_g, False

    @staticmethod
    def f(index, a) -> float:
        D1 = 15
        NEG_LIMIT = -8

        d = index * 0.5
        if d < NEG_LIMIT or (a < 0 and d < 0):
            return 0
        elif d <= D1:
            return 1
        elif d >= 2 * D1:
            return 0
        else:
            return ((d - 2 * D1) ** 2) / (D1 * D1)

    @staticmethod
    def g(index, v) -> float:
        V0 = 6
        D0 = 6
        D1 = 15
        NEG_LIMIT = -12

        d = index * 0.5

        def g0(v):
            if v <= 2 * V0:
                a = (D1 - D0) / (V0**2)
                return a * (v - V0) ** 2 + D0
            else:
                K = 1
                return min(K * (v - 2 * V0) + D1, 30)

        D2 = g0(v)
        if d > 2 * D2 or d < NEG_LIMIT:
            return 0
        elif -NEG_LIMIT <= d <= D2:
            return 1
        else:
            return ((d - 2 * D2) ** 2) / (D2**2)

    def _smooth_sequence(self, tl_state_buff: list[dict[tuple, TLS]]):
        """
        Smooths the sequence of traffic light states by identifying and correcting abnormal time intervals.

        Args:
            tl_state_buff (list[dict[tuple, TLS]]): A buffer containing the traffic light states.

        Returns:
            list[dict[tuple, TLS]]: The smoothed traffic light state buffer.
        """

        # according to independent phases, find out absnormal time
        intervals: set[tuple[int]] = set()

        for i in range(len(self.container_template)):
            for phase in self.container_template[i].keys():
                intervals.update(self._find_short_intervals(tl_state_buff, i, phase))

        intervals = sorted(intervals, key=lambda interval: interval[0])

        for start, end in intervals:
            # assert tl_state_buff[start-1] == tl_state_buff[end+1]
            for j in range(start, end + 1):
                tl_state_buff[j] = tl_state_buff[start - 1]

        return tl_state_buff

    def _find_short_intervals(self, tl_state_buff, way_i, phase) -> list[tuple[int]]:
        """
        Identify short intervals of traffic light states within a buffer.

        This method scans through a buffer of traffic light states (`tl_state_buff`) and identifies
        short intervals where the traffic light state changes from GREEN to RED or RED to GREEN
        for a specific way and phase. The intervals are considered short if their length is less
        than `self.SMOOTHING_WIDTH`.

        Args:
            tl_state_buff (list): A buffer containing traffic light states.
            way_i (int): The index of the way in the traffic light state buffer.
            phase (int): The phase of the traffic light to be checked.

        Returns:
            list[tuple[int]]: A list of tuples, each containing the start and end indices of the
            identified short intervals.
        """

        n = len(tl_state_buff)
        intervals: list[tuple[int]] = []

        t = 0
        while t < n:
            if tl_state_buff[t][way_i][phase] == TLS.GREEN:
                j = t + 1
                while j < n and tl_state_buff[j][way_i][phase] == TLS.RED:
                    j += 1
                if (
                    j < n
                    and j - t - 1 > 0
                    and j - t - 1 < self.SMOOTHING_WIDTH
                    and tl_state_buff[j][way_i][phase] == TLS.GREEN
                ):
                    intervals.append((t + 1, j - 1))
                t = j
            elif tl_state_buff[t][way_i][phase] == TLS.RED:
                j = t + 1
                while j < n and tl_state_buff[j][way_i][phase] == TLS.GREEN:
                    j += 1
                if (
                    j < n
                    and j - t - 1 > 0
                    and j - t - 1 < self.SMOOTHING_WIDTH
                    and tl_state_buff[j][way_i][phase] == TLS.RED
                ):
                    intervals.append((t + 1, j - 1))
                t = j
            else:
                t += 1

        return intervals

    def _add_yellow_light(self, tl_state_buff: list[dict[tuple, TLS]]) -> list[dict[tuple, TLS]]:
        """
        Adds yellow light phases to the traffic light state buffer.

        This method iterates through the traffic light state buffer and identifies transitions from GREEN to RED.
        It then inserts YELLOW phases for a specified duration before each RED phase.

        Args:
            tl_state_buff (list[dict[tuple, TLS]]): A list of dictionaries representing the traffic light states.
                Each dictionary maps a tuple (representing a specific traffic light phase) to a TLS state.

        Returns:
            list[dict[tuple, TLS]]: The modified traffic light state buffer with YELLOW phases added.
        """

        for i in range(len(self.container_template)):
            for phase in self.container_template[i].keys():
                indices = []
                for t in range(1, len(tl_state_buff)):
                    if tl_state_buff[t][i][phase] == TLS.RED and tl_state_buff[t - 1][i][phase] == TLS.GREEN:
                        indices.append(t)
                for t in indices:
                    for j in range(t - self.YELLOW_DURATION, t):
                        if 0 <= j < len(tl_state_buff):
                            tl_state_buff[j][i][phase] = TLS.YELLOW

        return tl_state_buff
