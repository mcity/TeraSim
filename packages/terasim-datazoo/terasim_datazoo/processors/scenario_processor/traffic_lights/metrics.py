from terasim_datazoo.utils import Direction, TLS
from terasim_datazoo.core.intersection import ApproachingLane, InJunctionLane


def has_red_light_running(
    intersections: list[list[list[ApproachingLane]]], data_type: str = "raw", only_raw_position=False
) -> bool:
    """
    Check if any intersection has a red light running vehicle.
    Parameters:
        intersections (list[list[list[ApproachingLane]]]): A nested list representing the intersections and their lanes.
        data_type (str, optional): The type of data to consider. Defaults to "raw".
        only_raw_position (bool, optional): Flag indicating whether to consider only the raw position. Defaults to False.
    Returns:
        bool: True if any intersection has a red light running, False otherwise.
    """
    
    return any(
        has_red_light_running_per_intersection(
            intersection, data_type=data_type, only_raw_position=only_raw_position
        )
        for intersection in intersections
    )


def has_red_light_running_per_intersection(
    intersection: list[list[ApproachingLane]], data_type: str = "raw", only_raw_position=False
) -> bool:
    """
    Check if there is a red light running violation at a single intersection.

    Args:
        intersection (list[list[ApproachingLane]]): A nested list representing the intersection structure.
        data_type (str, optional): The type of data to consider. Defaults to "raw".
        only_raw_position (bool, optional): Flag to indicate if only raw positions should be considered. Defaults to False.

    Returns:
        bool: True if there is a red light running violation, False otherwise.
    """

    STOP_POINT = 8

    for approach in intersection:
        for lane in approach:
            # we ignore lanes that have right turn connection
            if any(conn.direction == Direction.R for conn in lane.injunction_lanes):
                continue

            for conn in lane.injunction_lanes:
                for t in range(1, 91):
                    tls_list = conn.record_tls if data_type == "raw" else conn.new_tls
                    if only_raw_position:
                        position_condition = tls_list[t] not in [TLS.ABSENT, TLS.UNKNOWN]
                    else:
                        position_condition = True

                    if position_condition:
                        if tls_list[t] == tls_list[t - 1] == TLS.RED:
                            for veh_id, veh_state in conn.record_vehs[t].items():
                                if veh_state.lane_pos_idx > STOP_POINT:
                                    if veh_id in conn.record_vehs[t - 1]:
                                        if conn.record_vehs[t - 1][veh_id].lane_pos_idx <= STOP_POINT:
                                            return True

    return False


def tlhead_count(intersections: list[list[list[ApproachingLane]]], data_type: str = "raw") -> int:
    """
    Calculate the total count of traffic lights' heads in the given intersections.
    Parameters:
    - intersections: A nested list representing the intersections, where each intersection is a nested list of approaching lanes.
    - data_type: A string indicating the type of data to be used for counting. Default is "raw".
    Returns:
    - The total count of traffic lights' heads in the intersections.
    """

    return sum(tlhead_count_per_intersection(intersection, data_type) for intersection in intersections)


def tlhead_count_per_intersection(intersection: list[list[ApproachingLane]], data_type: str = "raw") -> int:
    """
    Calculate the total number of traffic light heads per intersection.
    Parameters:
    - intersection (list[list[ApproachingLane]]): A nested list representing the intersection structure.
    - data_type (str): The type of data to consider. Default is "raw".
    Returns:
    - int: The total number of traffic light heads.
    """


    total_tlheads = 0
    for approach in intersection:
        for lane in approach:
            for conn in lane.injunction_lanes:
                tls_list = conn.record_tls if data_type == "raw" else conn.new_tls
                total_tlheads += sum((state not in [TLS.ABSENT, TLS.UNKNOWN]) for state in tls_list)
    return total_tlheads
