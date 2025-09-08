from addict import Dict
from loguru import logger
import numpy as np
import os

from terasim.overlay import traci
from terasim.params import AgentType
import terasim.utils as utils

from .maneuver_challenge import get_maneuver_challenge
from .tools import get_nde_cmd_from_cmd_info, unavoidable_maneuver_challenge_hook
from ..base import CommandType, NDECommand
from ..trajectory import predict_future_trajectory_vehicle

predict_steps = 10 # * duration seconds
predict_time_resolution = 0.5 # seconds
def get_vehicle_accept_collision_command():
    """Get the accept collision command for the vehicle.

    Returns:
        NDECommand: Accept collision command for the vehicle
    """
    accept_command = NDECommand(
        command_type=CommandType.ACCELERATION,
        acceleration=0,
        prob=0,
        duration=2,
    )
    accept_command.info = {"mode": "accept_collision"}
    return accept_command

def get_vehicle_avoidance_command(
    adversarial_vehicle_id,
    victim_vehicle_id,
    emergency_brake_deceleration
):
    """Get the avoidance command for the vehicle.

    Args:
        adversarial_vehicle_id (str): ID of the adversarial vehicle.
        victim_vehicle_id (str): ID of the victim vehicle.
        emergency_brake_deceleration (float): Emergency brake deceleration of the vehicle.

    Returns:
        NDECommand: Avoidance command for the vehicle.
    """
    avoidance_command = NDECommand(
        command_type=CommandType.ACCELERATION,
        acceleration=-emergency_brake_deceleration,
        prob=0,
        duration=3,
    )
    avoidance_command.info.update(
        {
            "mode": "avoid_collision",
            "adversarial_vehicle_id": adversarial_vehicle_id,
            "victim_vehicle_id": victim_vehicle_id,
        }
    )
    return avoidance_command

def get_adversity_pair(env_command_information, potential=False):
    """Get the adversarial pair information.
    
    Args:
        env_command_information (dict): Command information of the environment.
        potential (bool): If True, return the potential adversarial pair information, otherwise return the adversity information for the agent actually doing adversarial actions.

    Returns:
        adversity_pair_dict (dict): Adversarial pair information.    
    """
    veh_command_information = {
        veh_id: info
        for veh_id, info in env_command_information[AgentType.VEHICLE].items()
        if "conflict_vehicle_list" in info
    }
    vru_command_information = {
        vru_id: info
        for vru_id, info in env_command_information[AgentType.VULNERABLE_ROAD_USER].items()
        if "conflict_vehicle_list" in info
    }

    if potential:
        adversity_pair_dict = {
            AgentType.VEHICLE: {
                veh_id: ctx["conflict_vehicle_list"]
                for veh_id, ctx in veh_command_information.items()
            },
            AgentType.VULNERABLE_ROAD_USER: {
                vru_id: ctx["conflict_vehicle_list"]
                for vru_id, ctx in vru_command_information.items()
            },
        }
    else:
        adversity_pair_dict = {
            AgentType.VEHICLE: {
                veh_id: ctx["conflict_vehicle_list"]
                for veh_id, ctx in veh_command_information.items()
                if "mode" in ctx and ctx["mode"] == "adversarial"
            },
            AgentType.VULNERABLE_ROAD_USER: {
                vru_id: ctx["conflict_vehicle_list"]
                for vru_id, ctx in vru_command_information.items()
                if "mode" in ctx and ctx["mode"] == "adversarial"
            },
        }
    return adversity_pair_dict

def remove_collision_avoidance_command_using_avoidability(
    env_observation, env_future_trajectory, env_command_information
):
    """Remove the collision avoidance command for the vehicles that encounters the unavoidable collision.

    Args:
        env_observation (dict): Observation of the environment.
        env_future_trajectory (dict): Future trajectory of the environment.
        env_command_information (dict): Command information of the environment.

    Returns:
        env_command_information (dict): Updated command information of the environment.
    """
    potential_adversity_pair = get_adversity_pair(
        env_command_information, potential=True
    )
    for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
        for (
            adversarial_agent_id,
            victim_vehicle_list,
        ) in potential_adversity_pair[agent_type].items():
            if (
                env_command_information[agent_type][adversarial_agent_id].get("avoidable", True)
                is False
            ):
                for victim_vehicle_id in victim_vehicle_list:
                    # remove the collision avoidance command
                    env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                        "ndd_command_distribution"
                    ]["avoid_collision"] = None
                    env_future_trajectory[AgentType.VEHICLE][victim_vehicle_id].pop(
                        "avoid_collision", None
                    )
                    logger.trace(
                        f"veh_id: {victim_vehicle_id} is not avoidable from {adversarial_agent_id}, remove the collision avoidance command"
                    )
    return env_command_information

def add_avoid_accept_collision_command(
    env_future_trajecotory, env_manuever_challenge, env_observation, env_command_information, sumo_net
):
    """Add the avoidance and accept collision command for the vehicles that are victims in the adversarial mode.

    Args:
        env_future_trajecotory (dict): Future trajectory of the environment.
        env_manuever_challenge (dict): Maneuver challenge of the environment.
        env_observation (dict): Observation of the environment.
        env_command_information (dict): Command information of the environment.
        sumo_net (object): Sumo network object.

    Returns:
        dict: Updated future trajectory of the environment.
        dict: Updated command information of the environment.
    """
    potential_adversity_pair_dict = get_adversity_pair(
        env_command_information, potential=True
    )
    # add avoidance command for the victim vehicles
    for (
        adversarial_vehicle_id,
        victim_vehicle_list,
    ) in potential_adversity_pair_dict[AgentType.VEHICLE].items():

        for victim_vehicle_id in victim_vehicle_list:
            avoidance_command = get_vehicle_avoidance_command(
                adversarial_vehicle_id,
                victim_vehicle_id,
                traci.vehicle.getEmergencyDecel(victim_vehicle_id)
            )
            env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                "ndd_command_distribution"
            ]["avoid_collision"] = avoidance_command
            (
                env_future_trajecotory[AgentType.VEHICLE][victim_vehicle_id][
                    "avoid_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                victim_vehicle_id,
                env_observation[AgentType.VEHICLE][victim_vehicle_id],
                avoidance_command,
                sumo_net,
                time_horizon_step=predict_steps,
                time_resolution=predict_time_resolution,
                interpolate_resolution=predict_time_resolution,
                current_time=None,
                veh_info=None,
            )

            accept_command = get_vehicle_accept_collision_command()
            env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                "ndd_command_distribution"
            ]["accept_collision"] = accept_command
            (
                env_future_trajecotory[AgentType.VEHICLE][victim_vehicle_id][
                    "accept_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                victim_vehicle_id,
                env_observation[AgentType.VEHICLE][victim_vehicle_id],
                accept_command,
                sumo_net,
                time_horizon_step=predict_steps,
                time_resolution=predict_time_resolution,
                interpolate_resolution=predict_time_resolution,
                current_time=None,
                veh_info=None,
            )

            logger.trace(
                f"add avoidance command for vehicle: {victim_vehicle_id}, with info {info}"
            )
        logger.trace(
            f"add avoidance command for vehicle: {victim_vehicle_list} from vehicle: {adversarial_vehicle_id}"
        )
    # add avoidance command for the victim vulnerable road users
    # TODO: if the victim vehicle already has the avoidance command, do not add the avoidance command again
    for (
        adversarial_vru_id,
        victim_vehicle_list,
    ) in potential_adversity_pair_dict[AgentType.VULNERABLE_ROAD_USER].items():
        for victim_vehicle_id in victim_vehicle_list:
            if (
                env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                    "ndd_command_distribution"
                ].get("avoid_collision", None)
                is not None
            ):
                continue
            avoidance_command = get_vehicle_avoidance_command(
                adversarial_vru_id,
                victim_vehicle_id,
                traci.vehicle.getEmergencyDecel(victim_vehicle_id)
            )
            env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                "ndd_command_distribution"
            ]["avoid_collision"] = avoidance_command
            (
                env_future_trajecotory[AgentType.VEHICLE][victim_vehicle_id][
                    "avoid_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                victim_vehicle_id,
                env_observation[AgentType.VEHICLE][victim_vehicle_id],
                avoidance_command,
                sumo_net,
                time_horizon_step=predict_steps,
                time_resolution=predict_time_resolution,
                interpolate_resolution=predict_time_resolution,
                current_time=None,
                veh_info=None,
            )

            accept_command = get_vehicle_accept_collision_command()
            env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                "ndd_command_distribution"
            ]["accept_collision"] = accept_command
            (
                env_future_trajecotory[AgentType.VEHICLE][victim_vehicle_id][
                    "accept_collision"
                ],
                info,
            ) = predict_future_trajectory_vehicle(
                victim_vehicle_id,
                env_observation[AgentType.VEHICLE][victim_vehicle_id],
                accept_command,
                sumo_net,
                time_horizon_step=predict_steps,
                time_resolution=predict_time_resolution,
                interpolate_resolution=predict_time_resolution,
                current_time=None,
                veh_info=None,
            )

            logger.trace(
                f"add avoidance command for vehicle: {victim_vehicle_id}, with info {info}"
            )
        logger.trace(
            f"add avoidance command for vehicle: {victim_vehicle_list} from vru: {adversarial_vru_id}"
        )
    return env_future_trajecotory, env_command_information

def get_environment_avoidability(
    env_maneuver_challenge, env_future_trajectory, env_observation, env_command_information, centered_agent_set=set()
):
    """Get the avoidability of the vehicles in the environment.
    
    Args:
        env_maneuver_challenge (dict): Maneuver challenge of the environment.
        env_future_trajectory (dict): Future trajectory of the environment.
        env_observation (dict): Observation of the environment.
        env_command_information (dict): Command information of the environment.
        centered_agent_set (set, optional): Set of centered agent IDs.
    
    Returns:
        dict: Maneuver challenge for the avoidance command of the victim vehicles.
        dict: Updated command information of the environment (including avoidability information).
    """
    adversarial_future_trajectory_dict = Dict(
        {
            AgentType.VEHICLE: {
                veh_id: env_future_trajectory[AgentType.VEHICLE][veh_id].get(
                    "adversarial", None
                )
                for veh_id in env_future_trajectory[AgentType.VEHICLE]
            },
            AgentType.VULNERABLE_ROAD_USER: {
                vru_id: env_future_trajectory[AgentType.VULNERABLE_ROAD_USER][
                    vru_id
                ].get("adversarial", None)
                for vru_id in env_future_trajectory[AgentType.VULNERABLE_ROAD_USER]
            },
        }
    )
    avoidance_future_trajectory_dict = Dict(
        {
            veh_id: env_future_trajectory[AgentType.VEHICLE][veh_id].get(
                "avoid_collision", None
            )
            for veh_id in env_future_trajectory[AgentType.VEHICLE]
        }
    )

    # initialize the avoidability of each vehicle
    for veh_id in env_maneuver_challenge[AgentType.VEHICLE]:
        env_command_information[AgentType.VEHICLE][veh_id]["avoidable"] = True

    # get the maneuver challenge for the adversarial vehicle future and the avoidance vehicle future
    maneuver_challenge_avoidance_dicts = Dict(
        {
            AgentType.VEHICLE: {},
            AgentType.VULNERABLE_ROAD_USER: {},
        }
    )
    for veh_id in env_maneuver_challenge[AgentType.VEHICLE]:
        if env_maneuver_challenge[AgentType.VEHICLE][veh_id].get("adversarial"):
            conflict_vehicle_list = env_command_information[AgentType.VEHICLE][veh_id].get(
                "conflict_vehicle_list", []
            )
            conflict_vehicle_future_dict = Dict(
                {
                    veh_id: avoidance_future_trajectory_dict[veh_id]
                    for veh_id in conflict_vehicle_list
                }
            )
            maneuver_challenge_avoidance_dicts[AgentType.VEHICLE][
                veh_id
            ] = get_maneuver_challenge(
                veh_id,
                adversarial_future_trajectory_dict[AgentType.VEHICLE][veh_id],
                AgentType.VEHICLE,
                conflict_vehicle_future_dict,
                AgentType.VEHICLE,
                env_observation,
                env_command_information[AgentType.VEHICLE][veh_id],
                record_in_ctx=False,
                buffer=0.5,  # buffer for the collision avoidance, 1m
                centered_agent_set=centered_agent_set,
            )
            if maneuver_challenge_avoidance_dicts[AgentType.VEHICLE][veh_id].get(
                "adversarial"
            ):
                env_command_information[AgentType.VEHICLE][veh_id]["avoidable"] = False
                logger.debug(
                    f"timestep: {utils.get_time()}, veh_id: {veh_id} is not avoidable"
                )
            else:
                logger.debug(
                    f"timestep: {utils.get_time()}, veh_id: {veh_id} is avoidable"
                )
            logger.trace(
                f"adversarial vehicle observation {env_observation[AgentType.VEHICLE][veh_id]}, conflict vehicle observation {Dict({veh_id: env_observation[AgentType.VEHICLE][veh_id] for veh_id in conflict_vehicle_list})}"
            )
            logger.trace(
                f"adversarial future trajectory dict for {veh_id}: {adversarial_future_trajectory_dict[AgentType.VEHICLE][veh_id]}, and conflict future trajectory dict for {conflict_vehicle_list}: {conflict_vehicle_future_dict}"
            )
    for vru_id in env_maneuver_challenge[AgentType.VULNERABLE_ROAD_USER]:
        if env_maneuver_challenge[AgentType.VULNERABLE_ROAD_USER][vru_id].get(
            "adversarial"
        ):
            conflict_vehicle_list = env_command_information[AgentType.VULNERABLE_ROAD_USER][
                vru_id
            ].get("conflict_vehicle_list", [])
            conflict_vehicle_future_dict = Dict(
                {
                    veh_id: avoidance_future_trajectory_dict[veh_id]
                    for veh_id in conflict_vehicle_list
                }
            )
            maneuver_challenge_avoidance_dicts[AgentType.VULNERABLE_ROAD_USER][
                vru_id
            ] = get_maneuver_challenge(
                vru_id,
                adversarial_future_trajectory_dict[AgentType.VULNERABLE_ROAD_USER][
                    vru_id
                ],
                AgentType.VULNERABLE_ROAD_USER,
                conflict_vehicle_future_dict,
                AgentType.VEHICLE,
                env_observation,
                env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id],
                record_in_ctx=False,
                buffer=0.5,  # buffer for the collision avoidance, 1m
                centered_agent_set=centered_agent_set,
            )
            if maneuver_challenge_avoidance_dicts[AgentType.VULNERABLE_ROAD_USER][
                vru_id
            ].get("adversarial"):
                env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id][
                    "avoidable"
                ] = False
                logger.debug(
                    f"timestep: {utils.get_time()}, vru_id: {vru_id} is not avoidable"
                )
            else:
                logger.debug(
                    f"timestep: {utils.get_time()}, vru_id: {vru_id} is avoidable"
                )
            logger.trace(
                f"adversarial vru observation {env_observation[AgentType.VULNERABLE_ROAD_USER][vru_id]}, conflict vehicle observation {Dict({veh_id: env_observation[AgentType.VEHICLE][veh_id] for veh_id in conflict_vehicle_list})}"
            )
            logger.trace(
                f"adversarial future trajectory dict for {vru_id}: {adversarial_future_trajectory_dict[AgentType.VULNERABLE_ROAD_USER][vru_id]}, and conflict future trajectory dict for {conflict_vehicle_list}: {conflict_vehicle_future_dict}"
            )

    return maneuver_challenge_avoidance_dicts, env_command_information

def modify_nde_cmd_veh_using_avoidability(
    unavoidable_collision_prob_factor, env_maneuver_challenge, env_command_information
):
    """Modify the probability of the NDE command for the vehicles that encounters the unavoidable collisions.

    Args:
        unavoidable_collision_prob_factor (float): Unavoidable collision probability factor.
        env_maneuver_challenge (dict): Maneuver challenge of the environment.
        env_command_information (dict): Command information of the environment.

    Returns:
        dict: Updated NDE command for the vehicles.
        dict: Updated command information of the environment.
    """
    nde_control_commands_veh = get_nde_cmd_from_cmd_info(
        env_command_information, AgentType.VEHICLE
    )

    for veh_id in env_maneuver_challenge[AgentType.VEHICLE]:
        # if the vehicle adversarial control command do has the potential to collide with other vehicles
        if env_maneuver_challenge[AgentType.VEHICLE][veh_id].get("adversarial"):
            # mark all rearend collision as unavoidable
            if env_command_information[AgentType.VEHICLE][veh_id].get("avoidable", True) is False:
                # collision unavoidable
                nde_control_commands_veh[veh_id]["adversarial"].prob = (
                    nde_control_commands_veh[veh_id]["adversarial"].prob * unavoidable_collision_prob_factor
                )
                nde_control_commands_veh[veh_id]["normal"].prob = (
                    1 - nde_control_commands_veh[veh_id]["adversarial"].prob
                )
                logger.trace(
                    f"{veh_id} is marked as unavoidable collision and the prob is reduced to {nde_control_commands_veh[veh_id]['adversarial'].prob}"
                )
                unavoidable_maneuver_challenge_hook(veh_id)
    return nde_control_commands_veh, env_command_information

def record_adversarial_related_information(adversarial_pair_dict, env_command_information, record):
    """Record the adversarial related information.

    Args:
        adversarial_pair_dict (dict): Adversarial pair information.
        env_command_information (dict): Command information of the environment.
        record (object): Record object.

    Returns:
        dict: Updated command information of the environment.
        object: Updated record object.
    """
    adversity_flag = True
    adversarial_agent_type = None
    if len(adversarial_pair_dict[AgentType.VEHICLE]):
        adversarial_agent_type = AgentType.VEHICLE
    elif len(adversarial_pair_dict[AgentType.VULNERABLE_ROAD_USER]):
        adversarial_agent_type = AgentType.VULNERABLE_ROAD_USER
    else:
        adversity_flag = False
    if adversity_flag:
        record.event_info[
            utils.get_time()
        ].adversarial_pair_dict = adversarial_pair_dict[adversarial_agent_type]
        record.event_info[utils.get_time()].adversarial_agent_id = list(
            adversarial_pair_dict[adversarial_agent_type].keys()
        )[0]
        adversarial_command_dict = {
            agent_id: env_command_information[adversarial_agent_type][
                agent_id
            ].ndd_command_distribution.adversarial
            for agent_id in adversarial_pair_dict[adversarial_agent_type]
        }
        victim_vehicle_id_set = set()
        for victim_vehicle_list in adversarial_pair_dict[adversarial_agent_type].values():
            victim_vehicle_id_set.update(victim_vehicle_list)

        victim_command_dict = {
            veh_id: env_command_information[AgentType.VEHICLE][veh_id].ndd_command_distribution
            for veh_id in victim_vehicle_id_set
        }

        record.event_info[utils.get_time()].adversarial_info_dict = {
            agent_id: adversarial_command.info
            for agent_id, adversarial_command in adversarial_command_dict.items()
        }

        adversarial_command_dict = {
            agent_id: str(adversarial_command)
            for agent_id, adversarial_command in adversarial_command_dict.items()
        }

        victim_command_dict = {
            veh_id: str(victim_command)
            for veh_id, victim_command in victim_command_dict.items()
        }

        record.event_info[
            utils.get_time()
        ].adversarial_command = adversarial_command_dict
        record.event_info[
            utils.get_time()
        ].victim_command = victim_command_dict
    return env_command_information, record

def apply_collision_avoidance(
    env_future_trajectory,
    env_command_information,
    nade_control_commands,
    record,
    excluded_agent_set=set(),
):
    """Apply collision avoidance for the victim vehicles.

    Args:
        env_future_trajectory (dict): Future trajectory of the environment.
        env_command_information (dict): Command information of the environment.
        nade_control_commands (dict): NADE control commands.
        record (object): Record object.
        excluded_agent_set (set): Set of excluded agent IDs.

    Returns:
        dict: Updated NADE control commands.
        dict: Updated command information of the environment.
        float: Weight of the testing episode.
        object: Updated record object.
    """

    adversarial_pair_dict = get_adversity_pair(env_command_information)
    avoid_collision_IS_prob = float(os.getenv("AVOID_COLLISION_IS_PROB", 0.2))
    avoid_collision_ndd_prob = 0.99
    weight = 1.0
    env_command_information, record = record_adversarial_related_information(
        adversarial_pair_dict, env_command_information, record
    )
    # no vehicle victim
    if len(adversarial_pair_dict[AgentType.VEHICLE]) == 0:
        return nade_control_commands, env_command_information, weight, record

    # victim vehicle set is all the vehicles that are victim by the adversarial vehicle, combine all vehicles in the adversarial_pair_dict values
    victim_vehicle_set = set()
    for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
        for victim_vehicle_list in adversarial_pair_dict[agent_type].values():
            victim_vehicle_set.update(victim_vehicle_list)
    
    for agent_id in excluded_agent_set:
        if agent_id in victim_vehicle_set:
            record.event_info[utils.get_time()].update(
                {
                    "victim_vehicle_id": agent_id,
                    "mode": "accept_collision",
                    "additional_info": "AV_neglected",
                }
            )
            return nade_control_commands, env_command_information, 1.0, record

    avoidance_command_list = [
        env_command_information[AgentType.VEHICLE][veh_id]["ndd_command_distribution"].get(
            "avoid_collision", None
        )
        for veh_id in victim_vehicle_set
    ]
    # no collision avoidance can be applied (predicted not avoidable)
    if all(
        avoidance_command is None for avoidance_command in avoidance_command_list
    ):
        logger.critical(
            f"all avoidance command is None, no collision avoidance command will be selected and NADE for collision avoidance will be disabled, victim_vehicle_set: {victim_vehicle_set}"
        )
        for victim_vehicle_id in victim_vehicle_set:
            env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                "mode"
            ] = "accept_collision"
            record.event_info[utils.get_time()].update(
                {
                    "victim_vehicle_id": victim_vehicle_id,
                    "mode": "accept_collision",
                    "additional_info": "all_avoidance_none",
                }
            )
            nade_control_commands[AgentType.VEHICLE][
                victim_vehicle_id
            ] = env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                "ndd_command_distribution"
            ].get(
                "accept_collision", None
            )
        return nade_control_commands, env_command_information, weight, record

    timestamp = utils.get_time()
    IS_prob = np.random.uniform(0, 1)

    # avoid collision
    if IS_prob < avoid_collision_IS_prob:
        for (
            adversarial_vehicle_id,
            victim_vehicle_list,
        ) in adversarial_pair_dict[AgentType.VEHICLE].items():
            if adversarial_vehicle_id in excluded_agent_set:
                logger.critical(
                    f"adversarial_vehicle_id: {adversarial_vehicle_id}, victim_vehicle_list: {victim_vehicle_list}"
                )
            logger.info(
                f"{timestamp}, victim_vehicle_list: {victim_vehicle_list} avoiding collision from {adversarial_vehicle_id}, avoidability: {env_command_information[AgentType.VEHICLE][adversarial_vehicle_id].get('avoidable', True)}"
            )
            for victim_vehicle_id in victim_vehicle_list:
                avoid_collision_command = env_command_information[AgentType.VEHICLE][
                    victim_vehicle_id
                ]["ndd_command_distribution"].get("avoid_collision", None)
                # if avoidable, then collision command should be available, if not avoidable, then collision command should be None
                assert (
                    (avoid_collision_command is not None)
                    and (
                        env_command_information[AgentType.VEHICLE][adversarial_vehicle_id][
                            "avoidable"
                        ]
                    )
                ) or (
                    (avoid_collision_command is None)
                    and (
                        not env_command_information[AgentType.VEHICLE][adversarial_vehicle_id][
                            "avoidable"
                        ]
                    )
                )
                if avoid_collision_command:
                    nade_control_commands[AgentType.VEHICLE][
                        victim_vehicle_id
                    ] = avoid_collision_command
                    env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                        "mode"
                    ] = "avoid_collision"
                    record.event_info[utils.get_time()].update(
                        {
                            "victim_vehicle_id": victim_vehicle_id,
                            "mode": "avoid_collision",
                        }
                    )
                else:
                    logger.critical(
                        f"victim_vehicle_id: {victim_vehicle_id} does not have avoidance command from {adversarial_vehicle_id}, avoidability: {env_command_information[AgentType.VEHICLE][adversarial_vehicle_id].get('avoidable', True)}"
                    )
        weight *= avoid_collision_ndd_prob / avoid_collision_IS_prob
    # accept collision
    else:
        for (
            adversarial_vehicle_id,
            victim_vehicle_list,
        ) in adversarial_pair_dict[AgentType.VEHICLE].items():
            logger.info(
                f"{timestamp}, victim_vehicle_list: {victim_vehicle_list} accept collision from {adversarial_vehicle_id}, avoidability: {env_command_information[AgentType.VEHICLE][adversarial_vehicle_id].get('avoidable', True)}"
            )
            for victim_vehicle_id in victim_vehicle_list:
                env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                    "mode"
                ] = "accept_collision"
                record.event_info[utils.get_time()].update(
                    {
                        "victim_vehicle_id": victim_vehicle_id,
                        "mode": "accept_collision",
                    }
                )
                nade_control_commands[AgentType.VEHICLE][
                    victim_vehicle_id
                ] = env_command_information[AgentType.VEHICLE][victim_vehicle_id][
                    "ndd_command_distribution"
                ].get(
                    "accept_collision", None
                )
        weight *= (1 - avoid_collision_ndd_prob) / (1 - avoid_collision_IS_prob)

    record.event_info[utils.get_time()].victim_command = {
        str(nade_control_commands[victim_vehicle_id])
        for victim_vehicle_id in victim_vehicle_set
    }

    return nade_control_commands, env_command_information, weight, record

