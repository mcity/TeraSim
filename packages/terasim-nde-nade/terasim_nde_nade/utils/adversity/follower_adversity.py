import addict

from terasim.overlay import traci

from ..agents import is_car_following
from ..base import CommandType, NDECommand


def derive_follower_adversarial_command(
    obs_dict, highlight_flag=False, highlight_color=[255, 0, 0, 255]
) -> addict.Dict:
    """Derive the adversarial follower negligence command based on the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to False.
        highlight_color (list, optional): Color to highlight the vehicle. Defaults to [255, 0, 0, 255].

    Returns:
        addict.Dict: Adversarial command.
    """
    follower_info = traci.vehicle.getFollower(obs_dict["ego"]["veh_id"], dist=40)
    current_acceleration = obs_dict["ego"]["acceleration"]

    # get adversarial command candidates
    adversarial_command_dict = addict.Dict()

    if follower_info is not None and follower_info[0] != "":  # there is a following vehicle, add follower neglgience type
        if follower_info[1] > 40:
            return adversarial_command_dict
        # if the vehicle and the following vehicle are both stopped, disable adversarial
        if (
            obs_dict["ego"]["velocity"] < 0.5
            and traci.vehicle.getSpeed(follower_info[0]) < 0.5
        ):
            return adversarial_command_dict

        # if the vehicle is car following
        is_car_following_flag = is_car_following(
            follower_info[0], obs_dict["ego"]["veh_id"]
        )

        # if ego vehicle is decelerating and the follower is still accelerating
        follower_acceleration = traci.vehicle.getAcceleration(follower_info[0])
        if current_acceleration <= 0 and follower_acceleration >= 0:
            emergency_decel = traci.vehicle.getEmergencyDecel(obs_dict["ego"]["veh_id"]) * -1
            adversarial_command = NDECommand(
                command_type=CommandType.ACCELERATION,
                duration=2.0,
                acceleration=emergency_decel,
            )
            adversarial_command.info.update(
                {
                    "is_car_following_flag": is_car_following_flag,
                    "follower_info": follower_info,
                    "emergency_deceleration": emergency_decel,
                    "current_acceleration": current_acceleration,
                    "mode": "adversarial",
                    "adversarial_mode": "Follow",
                }
            )
            adversarial_command_dict.update(addict.Dict({"Follow": adversarial_command}))
            if highlight_flag:
                traci.vehicle.setColor(
                    obs_dict["ego"]["veh_id"], highlight_color
                )  # highlight the vehicle with red
        return adversarial_command_dict
    else:
        return adversarial_command_dict
