from addict import Dict
from loguru import logger

from terasim.overlay import traci
from terasim.params import AgentType

from ..base import CommandType


def avoidable_maneuver_challenge_hook(veh_id):
    """Highlight the victim vehicle that can avoid the collision with the adversarial agent.
    
    Args:
        veh_id (str): ID of the vehicle.
    """
    traci.vehicle.highlight(veh_id, (255, 0, 0, 120), duration=traci.simulation.getDeltaT())

def unavoidable_maneuver_challenge_hook(veh_id):
    """Highlight the victim vehicle that cannot avoid the collision with the adversarial agent.

    Args:
        veh_id (str): ID of the vehicle.
    """
    traci.vehicle.highlight(veh_id, (128, 128, 128, 255), duration=traci.simulation.getDeltaT())

def adversarial_hook(veh_id):
    """Highlight the adversarial agent.

    Args:
        veh_id (str): ID of the vehicle.
    """
    traci.vehicle.highlight(veh_id, (255, 0, 0, 255), duration=2)

def get_nde_cmd_from_cmd_info(env_command_information, agent_type):
    """Get the NDE command distribution of the specific type of agents from the command information.

    Args:
        env_command_information (dict): Environment command information.
        agent_type (AgentType): Type of the agents.

    Returns:
        dict: NDE command distribution.
    """
    nde_control_commands_agenttype = Dict(
        {
            agent_id: env_command_information[agent_type][agent_id]["ndd_command_distribution"]
            for agent_id in env_command_information[agent_type]
        }
    )
    return nde_control_commands_agenttype

def update_nde_cmd_to_vehicle_cmd_info(env_command_information, nde_control_commands_veh):
    """Update the environment command information with the NDE command distribution of the vehicles.

    Args:
        env_command_information (dict): Environment command information.
        nde_control_commands_veh (dict): NDE command distribution of the vehicles.

    Returns:
        dict: Updated environment command information.
    """
    for veh_id in nde_control_commands_veh:
        env_command_information[AgentType.VEHICLE][veh_id][
            "ndd_command_distribution"
        ] = nde_control_commands_veh[veh_id]
    return env_command_information

def get_environment_criticality(env_maneuver_challenge, env_command_information):
    """Extract the criticality information of the vehicles based on the maneuver challenge and NDE command distribution.

    Args:
        env_maneuver_challenge (dict): Environment maneuver challenge.
        env_command_information (dict): Environment command information.

    Returns:
        dict: Environment criticality.
        dict: Updated environment command information.
    """
    nde_control_commands_veh = get_nde_cmd_from_cmd_info(
        env_command_information, AgentType.VEHICLE
    )
    env_criticality = {}
    for veh_id in env_maneuver_challenge[AgentType.VEHICLE]:
        ndd_control_command_dict = nde_control_commands_veh[veh_id]
        maneuver_challenge_dict = env_maneuver_challenge[AgentType.VEHICLE][
            veh_id
        ]
        env_criticality[veh_id] = {
            modality: ndd_control_command_dict[modality].prob * maneuver_challenge_dict[modality]
            for modality in maneuver_challenge_dict
            if modality != "info"
        }
    for veh_id in env_command_information[AgentType.VEHICLE]:
        env_command_information[AgentType.VEHICLE][veh_id]["criticality"] = (
            env_criticality[veh_id]
            if veh_id in env_criticality
            else {"normal": 0}
        )
    return env_criticality, env_command_information

def update_control_cmds_from_predicted_trajectory(
    nade_control_commands, env_future_trajectory, excluded_agent_set=set()
):
    """Update the env_command_information with the predicted future trajectories.
    Change the command type from acceleration to trajectory if the predicted collision type is rearend.

    Args:
        nade_control_commands (dict): NDE command distribution.
        env_future_trajectory (dict): Predicted future trajectories.
        excluded_agent_set (set, optional): Set of excluded agents. Defaults to set().

    Returns:
        dict: Updated NDE command distribution.
    """
    for agent_type in nade_control_commands:
        for agent_id in nade_control_commands[agent_type]:
            if agent_id in excluded_agent_set:
                continue
            if (
                nade_control_commands[agent_type][agent_id].info.get("mode")
                == "avoid_collision"
                or nade_control_commands[agent_type][agent_id].info.get("mode")
                == "adversarial"
                or nade_control_commands[agent_type][agent_id].info.get("mode")
                == "accept_collision"
            ):
                if (
                    nade_control_commands[agent_type][agent_id].command_type
                    == CommandType.ACCELERATION
                ):
                    nade_control_commands[agent_type][
                        agent_id
                    ].command_type = CommandType.TRAJECTORY
                    nade_control_commands[agent_type][
                        agent_id
                    ].future_trajectory = env_future_trajectory[agent_type][agent_id][
                        nade_control_commands[agent_type][agent_id].info.get("mode")
                    ][1:]
                    time_resolution = nade_control_commands[agent_type][agent_id].future_trajectory[1,-1] - nade_control_commands[agent_type][agent_id].future_trajectory[0,-1]
                    nade_control_commands[agent_type][
                        agent_id
                    ].time_resolution = time_resolution
                    logger.info(
                        f"agent_id: {agent_id} is updated to trajectory command with mode: {nade_control_commands[agent_type][agent_id].info.get('mode')}, trajectory: {nade_control_commands[agent_type][agent_id].future_trajectory}"
                    )
    return nade_control_commands