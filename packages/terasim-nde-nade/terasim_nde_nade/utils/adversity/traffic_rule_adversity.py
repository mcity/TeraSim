import addict

from terasim.overlay import traci

from .obs_processing import get_cf_acceleration, get_ff_acceleration
from ..base import CommandType, NDECommand


def will_stop_at_stopline(veh_id):
    """Check if the vehicle will stop at the stopline.

    Args:
        veh_id (str): Vehicle id.

    Returns:
        bool: True if the vehicle will stop at the stopline, False otherwise.
    """
    # next_tls = traci.vehicle.getNextTLS(veh_id)
    next_links = traci.vehicle.getNextLinks(veh_id)

    if next_links and not next_links[0][1]:
        # if next link is not empty, and the vehicle does not have road priority, then the vehicle is stopping at the stopline
        # check if the vehicle is stopping at intersection stopline using next_links[0][5] which denotes the state of the link
        if next_links[0][5] in "GgRrYyw":
            # traci set vehicle brown color
            # traci.vehicle.setColor(veh_id, (139, 69, 19, 255))
            return True, "intersection"
        elif next_links[0][5] in "smM":
            # traci set vehicle purple color
            # traci.vehicle.setColor(veh_id, (128, 0, 128, 255))
            return True, "roundabout"
        else:
            return False, None
    else:
        return False, None


def derive_traffic_rule_adversarial_command(
    obs_dict, highlight_flag=False, highlight_color=[0, 0, 255, 255]
) -> addict.Dict:
    """Derive the adversarial traffic rule command based on the observation.

    Args:
        obs_dict (dict): Observation of the ego agent.
        highlight_flag (bool, optional): Flag to indicate if the vehicle should be highlighted. Defaults to False.
        highlight_color (list, optional): Color to highlight the vehicle. Defaults to [0, 0, 255, 255].
    
    Returns:
        addict.Dict: Adversarial command.
    """
    leader_info = traci.vehicle.getLeader(obs_dict["ego"]["veh_id"], 40)
    current_acceleration = obs_dict["ego"]["acceleration"]
    ff_acceleration = get_ff_acceleration(obs_dict)

    # get adversarial command candidates
    adversarial_command_dict = addict.Dict()
    cf_acceleration = current_acceleration
    if leader_info is not None:
        cf_acceleration = get_cf_acceleration(obs_dict, leader_info)

    if (
        leader_info is None or cf_acceleration - current_acceleration > 1.5
    ):  # the vehicle is constained by the traffic rules
        stopping, stop_location = will_stop_at_stopline(obs_dict["ego"]["veh_id"])
        if not stopping:
            return adversarial_command_dict

        if ff_acceleration - current_acceleration > 1.0 or (
            current_acceleration < 0 and ff_acceleration > 0.2
        ):
            adversarial_command_dict["TrafficRule"] = NDECommand(
                command_type=CommandType.ACCELERATION,
                duration=2.0,
                acceleration=ff_acceleration,
            )
            adversarial_command_dict["TrafficRule"].info.update(
                {
                    "mode": "adversarial",
                    "adversarial_mode": "TrafficRule",
                    "stopping": stopping,
                    "stop_location": stop_location,
                }
            )

            if highlight_flag:
                traci.vehicle.setColor(
                    obs_dict["ego"]["veh_id"], highlight_color
                )  # highlight the vehicle with blue
    return adversarial_command_dict
