import addict

from terasim.overlay import traci

from .obs_processing import get_cf_acceleration, get_ff_acceleration

from ..agents import is_car_following
from ..base import CommandType, NDECommand


def derive_leader_adversarial_command(
    obs_dict, highlight_flag=False, highlight_color=[255, 0, 0, 255]
) -> addict.Dict:
    """Derive the adversarial leader negligence command based on the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to False.
        highlight_color (list, optional): Color to highlight the vehicle. Defaults to [255, 0, 0, 255].

    Returns:
        addict.Dict: Adversarial command.
    """
    leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)
    current_acceleration = obs_dict["ego"]["acceleration"]
    ff_acceleration = get_ff_acceleration(obs_dict)

    # get adversarial command candidates
    adversarial_command_dict = addict.Dict()
    cf_acceleration = current_acceleration

    if leader_info is not None:  # there is a leading vehicle, add lead neglgience type
        cf_acceleration = get_cf_acceleration(obs_dict, leader_info)
        # if the vehicle and the leading vehicle are both stopped, disable adversarial
        if (
            obs_dict["ego"]["velocity"] < 0.5
            and traci.vehicle.getSpeed(leader_info[0]) < 0.5
        ):
            return adversarial_command_dict

        # if the vehicle is car following
        is_car_following_flag = is_car_following(
            obs_dict["ego"]["veh_id"], leader_info[0]
        )
        if is_car_following_flag:
            leader_velocity = traci.vehicle.getSpeed(leader_info[0])
            # ego vehicle is stopping or the velocity difference between the ego vehicle and the leader is small
            if (
                obs_dict["ego"]["velocity"] < 0.5
                or abs(obs_dict["ego"]["velocity"] - leader_velocity) < 2
            ):
                return adversarial_command_dict

        # if the free flow acceleration is significantly larger than the car following accelerations
        if ff_acceleration - cf_acceleration > 1.5:
            adversarial_command = NDECommand(
                command_type=CommandType.ACCELERATION,
                duration=2.0,
                acceleration=ff_acceleration,
            )
            adversarial_command.info.update(
                {
                    "is_car_following_flag": is_car_following_flag,
                    "leader_info": leader_info,
                    "ff_acceleration": ff_acceleration,
                    "cf_acceleration": cf_acceleration,
                    "current_acceleration": current_acceleration,
                    "mode": "adversarial",
                    "adversarial_mode": "Lead",
                }
            )
            adversarial_command_dict.update(addict.Dict({"Lead": adversarial_command}))
            if highlight_flag:
                traci.vehicle.setColor(
                    obs_dict["ego"]["veh_id"], highlight_color
                )  # highlight the vehicle with red
        return adversarial_command_dict
    else:
        return adversarial_command_dict
