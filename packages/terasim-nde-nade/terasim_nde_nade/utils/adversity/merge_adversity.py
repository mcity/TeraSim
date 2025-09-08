import addict

from terasim.overlay import traci

from .obs_processing import get_cf_acceleration, get_ff_acceleration

from ..agents import is_car_following
from ..base import CommandType, NDECommand

from ...params import RAMP_EDGE_FEATURE


TIME_HEADWAY_THRESHOLD = 2.0


def derive_merge_adversarial_command_speeding(
    obs_dict, highlight_flag=False, highlight_color=[0, 255, 0, 255]
) -> addict.Dict:
    """Derive the adversarial command for potential merging vehicle based on the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to False.
        highlight_color (list, optional): Color to highlight the vehicle. Defaults to [0, 255, 0, 255].

    Returns:
        addict.Dict: Adversarial command.
    """
    adversarial_command_dict = addict.Dict()

    leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)
    safe_acceleration = get_ff_acceleration(obs_dict)

    right_leaders = traci.vehicle.getRightLeaders(obs_dict["ego"]["veh_id"])

    is_car_following_flag = False
    if leader_info is not None: # there is a leading vehicle, not neglect it
        leader_velocity = traci.vehicle.getSpeed(leader_info[0])
        # if the vehicle and the leading vehicle are both stopped, disable adversarial
        if (
            obs_dict["ego"]["velocity"] < 0.5
            and leader_velocity < 0.5
        ):
            return adversarial_command_dict
        
        # if the vehicle is car following, update the safe acceleration to be the car following acceleration
        is_car_following_flag = is_car_following(
            obs_dict["ego"]["veh_id"], leader_info[0], angle_difference_threshold=0.0
        )
        if is_car_following_flag:
            safe_acceleration = get_cf_acceleration(obs_dict, leader_info)
    
    # compare the safe acceleration with the cooperative acceleration considering the left leaders (merging vehicles)
    cooperative_acceleration = safe_acceleration
    right_leader_info = None
    if len(right_leaders) > 0:
        for right_leader in right_leaders:
            # SUMO sometimes regards the vehicle in front of the ego vehicle as the left leader
            is_car_following_flag_right_leader = is_car_following(
                obs_dict["ego"]["veh_id"], right_leader[0], angle_difference_threshold=0.0
            )
            if is_car_following_flag_right_leader:
                is_car_following_flag = True
                tmp_safe_acceleration = get_cf_acceleration(obs_dict, right_leader)
                if tmp_safe_acceleration < safe_acceleration:
                    safe_acceleration = tmp_safe_acceleration
                    leader_info = right_leader
            else:
                tmp_cooperative_acceleration = get_cf_acceleration(obs_dict, right_leader)
                if tmp_cooperative_acceleration < cooperative_acceleration:
                    cooperative_acceleration = tmp_cooperative_acceleration
                    right_leader_info = right_leader

    # if the safe following acceleration is significantly larger than the cooperative accelerations considering the merging vehicles
    if safe_acceleration - cooperative_acceleration > 1.5:
        adversarial_command = NDECommand(
            command_type=CommandType.ACCELERATION,
            duration=2.0,
            acceleration=safe_acceleration,
        )
        adversarial_command.info.update(
            {
                "is_car_following_flag": is_car_following_flag,
                "leader_info": leader_info,
                "safe_acceleration": safe_acceleration,
                "cooperative_acceleration": cooperative_acceleration,
                "right_leader_info": right_leader_info,
                "current_acceleration": obs_dict["ego"]["acceleration"],
                "mode": "adversarial",
                "adversarial_mode": "MergeSpeedUp",
            }
        )
        adversarial_command_dict.update(addict.Dict({"MergeSpeedUp": adversarial_command}))
        if highlight_flag:
            traci.vehicle.setColor(
                obs_dict["ego"]["veh_id"], highlight_color
            )  # highlight the vehicle with red

    return adversarial_command_dict


def derive_merge_adversarial_command_lanechange(
    obs_dict, highlight_flag=False, highlight_color=[0, 255, 0, 255]
) -> addict.Dict:
    """Derive the adversarial command for potential merging vehicle based on the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to False.
        highlight_color (list, optional): Color to highlight the vehicle. Defaults to [0, 255, 0, 255].

    Returns:
        addict.Dict: Adversarial command.
    """
    adversarial_command_dict = addict.Dict()
    
    ego_lane_index = obs_dict["ego"]["lane_index"]
    target_lane_id = obs_dict["ego"]["edge_id"]+f"_{ego_lane_index-1}" # the target lane is the right lane
    right_leaders = traci.vehicle.getRightLeaders(obs_dict["ego"]["veh_id"])
    right_followers = traci.vehicle.getRightFollowers(obs_dict["ego"]["veh_id"])
    
    safe_flag = True
    for right_leader in right_leaders:
        right_leader_lane_id = traci.vehicle.getLaneID(right_leader[0])
        right_leader_speed = traci.vehicle.getSpeed(right_leader[0])
        # calculate time headway
        if right_leader_speed < obs_dict["ego"]["velocity"]:
            time_headway = right_leader[1] / (obs_dict["ego"]["velocity"] - right_leader_speed)
            if right_leader_lane_id == target_lane_id and time_headway <= TIME_HEADWAY_THRESHOLD:
                safe_flag = False
                break
    for right_follower in right_followers:
        right_follower_lane_id = traci.vehicle.getLaneID(right_follower[0])
        right_follower_speed = traci.vehicle.getSpeed(right_follower[0])
        # calculate time headway
        if obs_dict["ego"]["velocity"] < right_follower_speed:
            time_headway = right_follower[1] / (right_follower_speed - obs_dict["ego"]["velocity"])
            if right_follower_lane_id == target_lane_id and time_headway <= TIME_HEADWAY_THRESHOLD:
                safe_flag = False
                break

    if safe_flag:
        adversarial_command_dict["MergeLaneChange"] = NDECommand(
            command_type=CommandType.RIGHT, duration=1.0
        )
        adversarial_command_dict["MergeLaneChange"].info.update(
            {"mode": "adversarial", "adversarial_mode": "MergeLaneChange"}
        )
        if highlight_flag:
            traci.vehicle.setColor(
                obs_dict["ego"]["veh_id"], highlight_color
            )  # highlight the vehicle with green
    return adversarial_command_dict


def exist_merging_vehicle(obs_dict, sumo_net=None) -> bool:
    """Determine if there is a merging vehicle in the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.

    Returns:
        bool: Flag to indicate if there is a merging vehicle.
    """
    # Assumption: the merging vehicle is in the rightmost lane
    merging_lane_id = None
    ego_edge_id = obs_dict["ego"]["edge_id"]
    if RAMP_EDGE_FEATURE in ego_edge_id:
        
        for lane_index in range(traci.edge.getLaneNumber(ego_edge_id)):
            lane_id = f"{ego_edge_id}_{lane_index}"
            disallowed_types = traci.lane.getDisallowed(lane_id)
            if "passenger" in disallowed_types or "truck" in disallowed_types:
                continue
            else:
                merging_lane_id = lane_id
                break
        if merging_lane_id is None:
            raise ValueError(f"All lanes of the edge {ego_edge_id} does not allow passenger/truck vehicles!")
        merging_vehicle_ids = list(traci.lane.getLastStepVehicleIDs(merging_lane_id))
        ego_id = obs_dict["ego"]["veh_id"]
        if len(merging_vehicle_ids) > 0 and ego_id not in merging_vehicle_ids:
            return True, merging_lane_id
        # consider the outgoing lane of the current edge
        if sumo_net is not None:
            outgoing_node_id = sumo_net.getEdge(ego_edge_id).getToNode().getID()
            if RAMP_EDGE_FEATURE in outgoing_node_id:
                outgoing_merging_lane_id = None
                for lane_id in sumo_net.getEdge(ego_edge_id).getToNode().getInternal():
                    disallowed_types = traci.lane.getDisallowed(lane_id)
                    if "passenger" in disallowed_types or "truck" in disallowed_types:
                        continue
                    else:
                        outgoing_merging_lane_id = lane_id
                        break
                if outgoing_merging_lane_id is not None:
                    merging_vehicle_ids_outpoing = list(traci.lane.getLastStepVehicleIDs(outgoing_merging_lane_id))
                    if len(merging_vehicle_ids_outpoing) > 0 and ego_id not in merging_vehicle_ids_outpoing:
                        return True, merging_lane_id
    return False, merging_lane_id