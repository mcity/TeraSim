import addict

from terasim.overlay import traci

from ..base import CommandType, NDECommand


def derive_lane_change_adversarial_command(
    obs_dict, highlight_flag=False, highlight_color=[0, 255, 0, 255]
) -> addict.Dict:
    """Derive the adversarial lane change command based on the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to False.
        highlight_color (list, optional): Color to highlight the vehicle. Defaults to [0, 255, 0, 255].

    Returns:
        addict.Dict: Adversarial command.
    """
    adversarial_command_dict = addict.Dict()
    left_lc_state = traci.vehicle.getLaneChangeStatePretty(
        obs_dict["ego"]["veh_id"], 1
    )[1]
    right_lc_state = traci.vehicle.getLaneChangeStatePretty(
        obs_dict["ego"]["veh_id"], -1
    )[1]
    left_lc_state_original = traci.vehicle.getLaneChangeState(
        obs_dict["ego"]["veh_id"], 1
    )
    right_lc_state_original = traci.vehicle.getLaneChangeState(
        obs_dict["ego"]["veh_id"], -1
    )

    if (
        "blocked by left follower" in left_lc_state
        and "blocked by left leader" not in left_lc_state
    ):  # blocked only by left follower
        left_follower = traci.vehicle.getLeftFollowers(
            obs_dict["ego"]["veh_id"]
        )  # get the left follower

        if len(left_follower):  # the left follower is close to the ego vehicle
            follower_mingap = traci.vehicle.getMinGap(left_follower[0][0])
            if left_follower[0][1] + follower_mingap > -2:
                adversarial_command_dict["LeftFoll"] = NDECommand(
                    command_type=CommandType.LEFT, duration=1.0
                )
                adversarial_command_dict["LeftFoll"].info.update(
                    {"mode": "adversarial", "adversarial_mode": "LeftFoll"}
                )
                if highlight_flag:
                    traci.vehicle.setColor(
                        obs_dict["ego"]["veh_id"], highlight_color
                    )  # highlight the vehicle with green
    if (
        "blocked by right follower" in right_lc_state
        and "blocked by right leader" not in right_lc_state
    ):  # blocked only by right follower
        right_follower = traci.vehicle.getRightFollowers(
            obs_dict["ego"]["veh_id"]
        )  # get the right follower
        if len(right_follower):
            follower_mingap = traci.vehicle.getMinGap(right_follower[0][0])
            # the right follower is close to the ego vehicle
            if right_follower[0][1] + follower_mingap > -2:
                adversarial_command_dict["RightFoll"] = NDECommand(
                    command_type=CommandType.RIGHT, duration=1.0
                )
                adversarial_command_dict["RightFoll"].info.update(
                    {"mode": "adversarial", "adversarial_mode": "RightFoll"}
                )
                if highlight_flag:
                    traci.vehicle.setColor(
                        obs_dict["ego"]["veh_id"], highlight_color
                    )  # highlight the vehicle with green
    return adversarial_command_dict
