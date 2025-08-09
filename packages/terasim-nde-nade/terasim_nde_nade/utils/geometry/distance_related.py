import numpy as np


def calclulate_distance_from_centered_agent(
    env_observation, centered_agent_id, centered_agent_type
):
    """Calculate the distance from the centered agent to all agents, including itself.

    Args:
        env_observation (dict): Environment observation.
        centered_agent_id (str): ID of the centered agent.
        centered_agent_type (AgentType): Type of the centered agent.

    Returns:
        dict: Distance from the centered agent to all agents, including itself.
    """
    assert centered_agent_type in env_observation.keys()
    assert centered_agent_id in env_observation[centered_agent_type].keys()
    
    centered_agent_pose = np.array(env_observation[centered_agent_type][centered_agent_id]['ego']['position'])
    all_agent_id = []
    all_agent_pose = []
    for agent_type in env_observation.keys():
        for agent_id in env_observation[agent_type].keys():
            all_agent_id.append(agent_id)
            all_agent_pose.append(env_observation[agent_type][agent_id]['ego']['position'])
    all_agent_pose = np.array(all_agent_pose)
    distance = np.linalg.norm(all_agent_pose - centered_agent_pose, axis=1)
    results = dict(zip(all_agent_id, distance))
    return results
