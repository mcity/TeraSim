from dataclasses import dataclass
import numpy as np
from typing import List, Optional

from terasim.overlay import profile, traci

from ..base import CommandType


@dataclass
class VulnerableRoadUserInfoForPredict:
    id: str
    acceleration: float
    edge_id: str
    lane_id: str
    position: List[float]
    velocity: float
    heading: float
    length: float
    route_id_list: Optional[List[str]] = None
    route_length_list: Optional[List[float]] = None

    def __getitem__(self, item):
        return self.__dict__[item]


@profile
def get_vulnerbale_road_user_info(vru_id, obs_dict, sumo_net) -> VulnerableRoadUserInfoForPredict:
    """Generate vulnerable road user information for future trajectory prediction.

    Args:
        vru_id (str): VRU ID.
        obs_dict (dict): observation dictionary.
        sumo_net (sumolib.net.Net): SUMO network object

    Returns:
        VulnerableRoadUserInfoForPredict: Vulnerable road user information.
    """
    ego_obs = obs_dict["ego"]
    vru_info = VulnerableRoadUserInfoForPredict(
        id=vru_id,
        acceleration=0,
        edge_id=traci.person.getRoadID(vru_id),
        lane_id=traci.person.getLaneID(vru_id),
        position=traci.person.getPosition(vru_id),
        velocity=traci.person.getSpeed(vru_id),
        heading=traci.person.getAngle(vru_id),
        length=traci.person.getLength(vru_id),
    )
    vru_info.route_id_list = [traci.person.getRoadID(vru_id)]
    # veh_info.route_length_list = [route._length for route in route_with_internal]
    vru_info.route_length_list = [
        traci.lane.getLength(edge_id + "_0") for edge_id in vru_info.route_id_list
    ]
    return vru_info
