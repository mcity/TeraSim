from dataclasses import dataclass
import math
import sumolib
from typing import List, Optional

from terasim.overlay import traci

from .base import AgentInfo


@dataclass
class VehicleInfoForPredict(AgentInfo):
    """Vehicle information for trajectory prediction."""

    acceleration: float
    route: List[str]
    route_index: int
    edge_id: str
    lane_id: str
    lane_index: int
    lane_position: float
    length: float
    route_id_list: Optional[List[str]] = None
    route_length_list: Optional[List[float]] = None
    upcoming_lane_id_list: Optional[List[str]] = None

    def __getitem__(self, item):
        return self.__dict__[item]


def get_next_lane_edge(net, lane_id):
    """Get the next lane and edge IDs for a given lane.
    
    Args:
        net (sumolib.net.Net): SUMO network object.
        lane_id (str): Lane ID.

    Returns:
        tuple: Next lane ID and edge ID.
    """
    origin_lane = net.getLane(lane_id)
    outgoing_lanes = [conn.getToLane() for conn in origin_lane.getOutgoing()]
    outgoing_edges = [lane.getEdge() for lane in outgoing_lanes]
    return outgoing_lanes[0].getID(), outgoing_edges[0].getID()


def get_lane_angle(lane_id: str, mode: str = "start") -> float:
    """Get the angle of a lane at a specific position.
    
    Args:
        lane_id (str): Lane ID.
        mode (str): Position mode (start or end).

    Returns:
        float: Lane angle.
    """
    if mode == "start":
        relative_position = 0
    elif mode == "end":
        relative_position = traci.lane.getLength(lane_id) - 0.1
    else:
        raise ValueError("mode must be either start or end")
    lane_angle = traci.lane.getAngle(lane_id, relative_position)
    return lane_angle


def get_lanechange_longitudinal_speed(
    veh_id: str,
    current_speed: float,
    lane_width: Optional[float] = None,
    lanechange_duration: float = 1.0,
) -> float:
    """Calculate the longitudinal speed during a lane change maneuver.

    Args:
        veh_id (str): Vehicle ID.
        current_speed (float): Current speed of the vehicle.
        lane_width (float, optional): Lane width. Defaults to None.
        lanechange_duration (float, optional): Duration of the lane change maneuver. Defaults to 1.0.

    Returns:
        float: Longitudinal speed during lane change.
    """
    if lane_width is None:
        lane_width = traci.lane.getWidth(traci.vehicle.getLaneID(veh_id))
    lateral_speed = lane_width / lanechange_duration
    return math.sqrt(max(current_speed**2 - lateral_speed**2, 0))


def get_upcoming_lane_id_list(veh_id: str) -> List[str]:
    """Get a list of upcoming lane IDs for a vehicle.
    
    Args:
        veh_id (str): Vehicle ID.

    Returns:
        List[str]: List of upcoming lane IDs.
    """
    veh_next_links = traci.vehicle.getNextLinks(veh_id)
    current_lane_id = traci.vehicle.getLaneID(veh_id)
    lane_links = traci.lane.getLinks(current_lane_id)
    upcoming_lane_id_list = [current_lane_id]

    if isinstance(lane_links, list) and len(lane_links) > 0:
        for lane_link in lane_links:
            lane_id = lane_link[0]
            via_lane_id = lane_link[4]
            if via_lane_id != "":
                upcoming_lane_id_list.append(via_lane_id)
            upcoming_lane_id_list.append(lane_id)

    if len(veh_next_links) == 0:
        return upcoming_lane_id_list

    for link in veh_next_links:
        lane_id = link[0]
        via_lane_id = link[4]
        upcoming_lane_id_list.append(via_lane_id)
        upcoming_lane_id_list.append(lane_id)
    return upcoming_lane_id_list


def get_vehicle_info(veh_id: str, obs_dict: dict, sumo_net) -> VehicleInfoForPredict:
    """Generate vehicle information for future trajectory prediction.
    
    Args:
        veh_id (str): Vehicle ID.
        obs_dict (dict): Observation dictionary.
        sumo_net (sumolib.net.Net): SUMO network object.

    Returns:
        VehicleInfoForPredict: Vehicle information.
    """
    veh_info = VehicleInfoForPredict(
        id=veh_id,
        acceleration=traci.vehicle.getAcceleration(veh_id),
        route=traci.vehicle.getRoute(veh_id),
        route_index=traci.vehicle.getRouteIndex(veh_id),
        edge_id=traci.vehicle.getRoadID(veh_id),
        lane_id=traci.vehicle.getLaneID(veh_id),
        lane_index=traci.vehicle.getLaneIndex(veh_id),
        position=traci.vehicle.getPosition(veh_id),
        velocity=traci.vehicle.getSpeed(veh_id),
        heading=traci.vehicle.getAngle(veh_id),
        lane_position=traci.vehicle.getLanePosition(veh_id),
        length=traci.vehicle.getLength(veh_id),
    )

    route_with_internal = sumolib.route.addInternal(sumo_net, veh_info.route)
    veh_info.route_id_list = [route._id for route in route_with_internal]
    veh_info.route_length_list = [
        traci.lane.getLength(edge_id + "_0") for edge_id in veh_info.route_id_list
    ]
    veh_info.upcoming_lane_id_list = get_upcoming_lane_id_list(veh_id)
    return veh_info


def is_car_following(follow_id: str, leader_id: str, angle_difference_threshold: float = 5.0) -> bool:
    """Check if one vehicle is following another vehicle.
    
    Args:
        follow_id (str): Following vehicle ID.
        leader_id (str): Leading vehicle ID.
        angle_difference_threshold (float, optional): Angle difference threshold for lane change detection. Defaults to 5.0.

    Returns:
        bool: True if the follow_id is following the leader_id, False otherwise.
    """
    current_lane_id = traci.vehicle.getLaneID(follow_id)
    current_edge_id = traci.vehicle.getRoadID(follow_id)
    leader_lane_id = traci.vehicle.getLaneID(leader_id)
    leader_edge_id = traci.vehicle.getRoadID(leader_id)
    current_angle = traci.vehicle.getAngle(follow_id)
    leader_angle = traci.vehicle.getAngle(leader_id)

    # Check if vehicles are on the same link
    if current_lane_id == leader_lane_id:
        return True
    elif current_edge_id == leader_edge_id:
        return False
    elif abs((current_angle - leader_angle + 180) % 360 - 180) <= angle_difference_threshold:
        return True

    # Check future links
    follower_future_link_infos = traci.vehicle.getNextLinks(follow_id)
    if len(follower_future_link_infos) == 0:
        return False

    follower_future_lane_id = follower_future_link_infos[0][0]
    follower_future_junction_lane_id = follower_future_link_infos[0][4]

    if (
        leader_lane_id in follower_future_lane_id
        or leader_lane_id in follower_future_junction_lane_id
    ):
        return True

    leader_future_link_infos = traci.vehicle.getNextLinks(leader_id)
    if len(leader_future_link_infos) == 0:
        return False

    leader_future_lane_id = leader_future_link_infos[0][0]
    leader_junction_lane_id = leader_future_link_infos[0][4]

    # Check if vehicles share any future links
    if (
        len(
            set(
                [
                    follower_future_lane_id,
                    follower_future_junction_lane_id,
                    leader_future_lane_id,
                    leader_junction_lane_id,
                ]
            )
        )
        < 4
    ):
        return True

    return False

def is_lane_changing(veh_id, obs_dict, angle_threshold=5):
    """Check if a vehicle is lane changing.

    Args:
        veh_id (str): Vehicle ID.
        obs_dict (dict): Observation dictionary.
        angle_threshold (int, optional): Angle threshold for lane change detection. Defaults to 5.
    
    Returns:
        bool: True if the vehicle is lane changing, False otherwise.
    """
    original_angle = obs_dict["ego"]["heading"]
    lane_id = obs_dict["ego"]["lane_id"]
    lane_position = traci.vehicle.getLanePosition(veh_id)
    lane_angle = traci.lane.getAngle(
        laneID=lane_id,
        relativePosition=max(
            lane_position - 0.5 * traci.vehicle.getLength(veh_id), 0
        ),
    )
    angle_diff = (lane_angle - original_angle + 180) % 360 - 180
    return abs(angle_diff) >= angle_threshold