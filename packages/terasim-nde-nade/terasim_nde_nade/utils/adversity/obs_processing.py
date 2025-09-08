from terasim.overlay import traci


def get_ff_acceleration(obs_dict):
    """Get the free flow acceleration of the ego vehicle.

    Args:
        obs_dict (dict): Observation of the ego agent.

    Returns:
        float: Free flow acceleration of the ego vehicle.
    """
    ff_speed = min(
        traci.vehicle.getFollowSpeed(
            obs_dict["ego"]["veh_id"],
            obs_dict["ego"]["velocity"],
            3000,
            obs_dict["ego"]["velocity"],
            7.06,
        ),
        traci.vehicle.getAllowedSpeed(obs_dict["ego"]["veh_id"]),
    )
    return (ff_speed - obs_dict["ego"]["velocity"]) / traci.simulation.getDeltaT()


def get_cf_acceleration(obs_dict, leader_info):
    """Get the car following acceleration of the ego vehicle.

    Args:
        obs_dict (dict): Observation of the ego agent.
        leader_info (tuple): Information of the leader vehicle.

    Returns:
        float: Car following acceleration of the ego vehicle.
    """
    leader_id, leader_distance = leader_info
    cf_speed_with_leading_vehicle = traci.vehicle.getFollowSpeed(
        obs_dict["ego"]["veh_id"],
        obs_dict["ego"]["velocity"],
        leader_distance,
        traci.vehicle.getSpeed(leader_id),
        7.06,
    )
    cf_acceleration = (
        cf_speed_with_leading_vehicle - obs_dict["ego"]["velocity"]
    ) / traci.simulation.getDeltaT()
    return cf_acceleration
