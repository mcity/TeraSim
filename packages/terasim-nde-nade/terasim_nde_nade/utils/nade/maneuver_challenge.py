from addict import Dict
from loguru import logger

from terasim.params import AgentType
from terasim.overlay import traci, profile

from .tools import avoidable_maneuver_challenge_hook
from ..collision import check_trajectory_intersection


predict_steps = 10 # * duration seconds
vehicle_vehicle_maneuver_challenge_step = 10 # * duration seconds
vru_to_vehicle_maneuver_challenge_step = 10 # * duration seconds
vehicle_vru_maneuver_challenge_step = 10 # * durationseconds
predict_time_resolution = 0.5 # seconds


def is_link_intersect(veh1_obs, veh2_obs):
    """Check if the next link of the two vehicles are intersected.

    Args:
        veh1_obs (dict): Observation of the first vehicle.
        veh2_obs (dict): Observation of the second vehicle.

    Returns:
        bool: Flag to indicate if the next link of the two vehicles are intersected.
    """
    veh_1_edge_id = veh1_obs["ego"]["edge_id"]
    veh_2_edge_id = veh2_obs["ego"]["edge_id"]
    if veh_1_edge_id == veh_2_edge_id:
        return True

    veh1_next_lane_id_set = set(veh1_obs["ego"]["upcoming_lanes"])
    veh2_next_lane_id_set = set(veh2_obs["ego"]["upcoming_lanes"])

    if veh1_next_lane_id_set.intersection(veh2_next_lane_id_set):
        return True

    veh1_foe_lane_id_set = set(veh1_obs["ego"]["upcoming_foe_lane_id_list"])
    veh2_foe_lane_id_set = set(veh2_obs["ego"]["upcoming_foe_lane_id_list"])

    if veh1_foe_lane_id_set.intersection(
        veh2_next_lane_id_set
    ) or veh2_foe_lane_id_set.intersection(veh1_next_lane_id_set):
        return True
    return False

def clip_trajectory_to_maneuver_challenge_step(trajectory, maneuver_challenge_step):
    """Clip the trajectory to the maneuver challenge step.
    """
    return trajectory[:maneuver_challenge_step+1]

def get_maneuver_challenge(
    adversarial_agent_id,
    adversarial_agent_future,
    adversarial_agent_type,
    all_normal_agent_future,
    normal_agent_type,
    env_observation,
    agent_command_information,
    record_in_ctx=False,
    highlight_flag=True,
    buffer=0,
    centered_agent_set=set(),
    maneuver_challenge_step=vehicle_vehicle_maneuver_challenge_step,
):
    """Get the challenge for the adversarial maneuver.

    Args:
        adversarial_agent_id (str): ID of the adversarial agent.
        adversarial_agent_future (list): Future trajectory of the adversarial agent.
        adversarial_agent_type (str): Type of the adversarial agent, i.e., "vehicle" or "vulnerable_road_user".
        all_normal_agent_future (dict): Future trajectory of the normal command for all agents.
        normal_agent_type (str): Type of the agents conducting the normal maneuver, i.e., "vehicle" or "vulnerable_road_user".
        env_observation (dict): Environment observation.
        agent_command_information (dict): Command information of the adversarial agent.
        record_in_ctx (bool, optional): Flag to indicate if the information should be recorded in the context. Defaults to False.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to True.
        buffer (int, optional): Buffer for the collision check. Defaults to 0.
        centered_agent_set (set, optional): Set of centered agent IDs. Defaults to set().

    Returns:
        dict: Maneuver challenge information.
    """
    if centered_agent_set != set() and adversarial_agent_id not in centered_agent_set:
        filtered_normal_agent_future = {
            agent_id: all_normal_agent_future[agent_id]
            for agent_id in all_normal_agent_future
            if agent_id in centered_agent_set
        }
    else:
        filtered_normal_agent_future = all_normal_agent_future

    
    # see if the one adversarial future will intersect with other normal futures
    final_collision_flag = False
    if adversarial_agent_future is not None and filtered_normal_agent_future is not None:
        clipped_adversarial_agent_future = clip_trajectory_to_maneuver_challenge_step(adversarial_agent_future, maneuver_challenge_step)
        clipped_filtered_normal_agent_future = Dict()
        for agent_id in filtered_normal_agent_future:
            clipped_filtered_normal_agent_future[agent_id] = clip_trajectory_to_maneuver_challenge_step(filtered_normal_agent_future[agent_id], maneuver_challenge_step)

        for agent_id in clipped_filtered_normal_agent_future:
            if agent_id == adversarial_agent_id:
                continue
            if clipped_filtered_normal_agent_future[agent_id] is None:
                print(
                    f"agent_id: {agent_id}, all_normal_agent_future[agent_id]: {all_normal_agent_future[agent_id]}"
                )
            if adversarial_agent_type == AgentType.VEHICLE:
                # for vehicles, we need to check if the next link of the two vehicles are intersected, which can reduce the computational complexity
                link_intersection_flag = is_link_intersect(
                    env_observation[adversarial_agent_type][adversarial_agent_id],
                    env_observation[normal_agent_type][agent_id],
                )
                if not link_intersection_flag:
                    continue  # if the next link of the two vehicles are not intersected, then the two vehicles will not collide
            elif adversarial_agent_type == AgentType.VULNERABLE_ROAD_USER:
                # for VRU, avoid collision with vehicles moving slowly
                if env_observation[normal_agent_type][agent_id]["ego"]["velocity"] < 1.0:
                    continue
            
            collision_flag = check_trajectory_intersection(
                clipped_adversarial_agent_future,
                clipped_filtered_normal_agent_future[agent_id],
                env_observation[adversarial_agent_type][adversarial_agent_id]["ego"][
                    "length"
                ],
                env_observation[normal_agent_type][agent_id]["ego"]["length"],
                env_observation[adversarial_agent_type][adversarial_agent_id]["ego"][
                    "width"
                ],
                env_observation[normal_agent_type][agent_id]["ego"]["width"],
                adversarial_agent_type.value,
                normal_agent_type.value,
                buffer,
            )
            final_collision_flag = final_collision_flag or collision_flag
            if collision_flag and record_in_ctx:
                if "conflict_vehicle_list" not in agent_command_information:
                    agent_command_information["conflict_vehicle_list"] = []
                agent_command_information["conflict_vehicle_list"].append(agent_id)
                # logger.trace(
                #     f"veh_id: {adversarial_veh_id} will collide with veh_id: {veh_id}"
                # )
        return {
            "normal": 0,
            "adversarial": (1 if final_collision_flag else 0),
        }  # the first element is the number of vehicles that will be affected by the adversarial vehicle
    else:
        return {"normal": 0}
    
@profile
def get_environment_maneuver_challenge(env_future_trajectory, env_observation, env_command_information, centered_agent_set=set()):
    """Get the maneuver challenge for each agent when it is in the adversarial mode while other vehicles are in the normal mode.
    Note: We only consider the challenge for the following cases:
    1. vehicle in the adversarial mode and the vehicle in the normal mode.
    2. vru in the adversarial mode and the vehicle in the normal mode.

    Args:
        env_future_trajectory (dict): Future trajectory of the agents.
        env_observation (dict): Environment observation.
        env_command_information (dict): Command information of the agents.
        centered_agent_set (set, optional): Set of centered agent IDs. Defaults to set().

    Returns:
        dict: Environment maneuver challenge information.
        dict: Updated environment command information.
    """
    normal_future_trajectory_veh = Dict(
        {
            veh_id: env_future_trajectory[AgentType.VEHICLE][veh_id].get("normal", None)
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )
    adversarial_future_trajectory_veh = Dict(
        {
            veh_id: env_future_trajectory[AgentType.VEHICLE][veh_id].get(
                "adversarial", None
            )
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )
    adversarial_future_trajectory_vru = Dict(
        {
            vru_id: env_future_trajectory[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                "adversarial", None
            )
            for vru_id in env_future_trajectory[AgentType.VULNERABLE_ROAD_USER]
        }
    )

    # get the maneuver challenge for each vehicle, check if the adversarial future will collide with other vehicles' normal future
    maneuver_challenge_veh = Dict(
        {
            veh_id: get_maneuver_challenge(
                veh_id,
                adversarial_future_trajectory_veh[veh_id],
                AgentType.VEHICLE,
                normal_future_trajectory_veh,
                AgentType.VEHICLE,
                env_observation,
                env_command_information[AgentType.VEHICLE][veh_id],
                record_in_ctx=True,
                centered_agent_set=centered_agent_set,
                maneuver_challenge_step=vehicle_vehicle_maneuver_challenge_step,
            )
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )
    maneuver_challenge_vru = Dict(
        {
            vru_id: get_maneuver_challenge(
                vru_id,
                adversarial_future_trajectory_vru[vru_id],
                AgentType.VULNERABLE_ROAD_USER,
                normal_future_trajectory_veh,
                AgentType.VEHICLE,
                env_observation,
                env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id],
                record_in_ctx=True,
                centered_agent_set=centered_agent_set,
                maneuver_challenge_step=vru_to_vehicle_maneuver_challenge_step,
            )
            for vru_id in env_future_trajectory[AgentType.VULNERABLE_ROAD_USER]
        }
    )

    for veh_id in env_command_information[AgentType.VEHICLE]:
        env_command_information[AgentType.VEHICLE][veh_id]["maneuver_challenge"] = (
            maneuver_challenge_veh[veh_id]
            if veh_id in maneuver_challenge_veh
            else {"normal": 0}
        )

    for vru_id in env_command_information[AgentType.VULNERABLE_ROAD_USER]:
        env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id]["maneuver_challenge"] = (
            maneuver_challenge_vru[vru_id]
            if vru_id in maneuver_challenge_vru
            else {"normal": 0}
        )

    maneuver_challenge_veh_shrinked = Dict(
        {
            veh_id: maneuver_challenge_veh[veh_id]
            for veh_id in maneuver_challenge_veh
            if maneuver_challenge_veh[veh_id].get("adversarial")
        }
    )
    for veh_id in maneuver_challenge_veh_shrinked:
        avoidable_maneuver_challenge_hook(veh_id)
    conflict_vehicle_info = Dict(
        {
            AgentType.VEHICLE: {
                veh_id: env_command_information[AgentType.VEHICLE][veh_id].get(
                    "conflict_vehicle_list"
                )
                for veh_id in env_command_information[AgentType.VEHICLE]
                if env_command_information[AgentType.VEHICLE][veh_id].get("conflict_vehicle_list")
            },
            AgentType.VULNERABLE_ROAD_USER: {
                vru_id: env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                    "conflict_vehicle_list"
                )
                for vru_id in env_command_information[AgentType.VULNERABLE_ROAD_USER]
                if env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
                    "conflict_vehicle_list"
                )
            },
        }
    )
    logger.trace(
        f"maneuver_challenge: {maneuver_challenge_veh_shrinked}, conflict_vehicle_info: {conflict_vehicle_info}"
    )
    env_maneuver_challenge = {
        AgentType.VEHICLE: maneuver_challenge_veh,
        AgentType.VULNERABLE_ROAD_USER: maneuver_challenge_vru,
    }
    return env_maneuver_challenge, env_command_information

