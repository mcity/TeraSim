import addict
import math

from terasim.overlay import traci

from ..base import CommandType, NDECommand


def derive_lane_change_abort_adversarial_command(
    obs_dict, highlight_flag=False, highlight_color=[0, 255, 0, 255]
) -> addict.Dict:
    """Derive the adversarial lane change abortion command based on the observation.

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

    duration = 1.0

    current_lane_position = obs_dict["ego"]["lane_position"]
    speed = obs_dict["ego"]["velocity"]
    lane_id = obs_dict["ego"]["lane_id"]

    within_edge_flag = current_lane_position + speed*duration < traci.lane.getLength(lane_id)
    moving_flag = speed > 1e-2
    
    if (
        "blocked by left follower" in left_lc_state
        and "blocked by left leader" not in left_lc_state
    ):  # blocked only by left follower
        left_follower = traci.vehicle.getLeftFollowers(
            obs_dict["ego"]["veh_id"]
        )  # get the left follower

        if len(left_follower):  # the left follower is close to the ego vehicle
            follower_mingap = traci.vehicle.getMinGap(left_follower[0][0])
            safe_flag = left_follower[0][1] + follower_mingap > -2

            if safe_flag and within_edge_flag and moving_flag:
                adversarial_command_dict["LeftAbort"] = generate_lanechangeabort_trajectory_command(
                    obs_dict, duration, side="left"
                )
                adversarial_command_dict["LeftAbort"].info.update(
                    {"mode": "adversarial", "adversarial_mode": "LeftAbort"}
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
            safe_flag = right_follower[0][1] + follower_mingap > -2
            if safe_flag and within_edge_flag and moving_flag:
                adversarial_command_dict["RightAbort"] = generate_lanechangeabort_trajectory_command(
                    obs_dict, duration, side="right"
                )
                adversarial_command_dict["RightAbort"].info.update(
                    {"mode": "adversarial", "adversarial_mode": "RightAbort"}
                )
                if highlight_flag:
                    traci.vehicle.setColor(
                        obs_dict["ego"]["veh_id"], highlight_color
                    )  # highlight the vehicle with green
    return adversarial_command_dict


def generate_lanechangeabort_trajectory_command(obs_dict, duration, side = "left"):
    """Generate the trajectory command for lane change abortion.

    Args:
        obs_dict (dict): Observation of the ego agent.
        duration (float): Duration of the lane change abortion.
        side (str, optional): Side of the lane change. Defaults to "left".

    Returns:
        NDECommand: Trajectory command for lane change abortion.
    """
    ego_id = obs_dict["ego"]["veh_id"]
    current_lane_position = obs_dict["ego"]["lane_position"]
    speed = obs_dict["ego"]["velocity"]
    edge_id = obs_dict["ego"]["edge_id"]
    lane_id = obs_dict["ego"]["lane_id"]

    dt = traci.simulation.getDeltaT()
    steps = int(duration / dt / 2)
    distance = speed * duration

    total_width = traci.lane.getWidth(lane_id) / 2

    dw = total_width / steps
    dl = distance / steps / 2

    dw = dw if side == "left" else -dw

    trajectory = []
    lane_center_list = []
    current_timestamp = traci.simulation.getTime()
    current_lateral_pos = traci.vehicle.getLateralLanePosition(ego_id)
    # move towards the side
    for i in range(steps):
        new_line_pos = current_lane_position + (i+1) * dl    
        new_offset = (i+1) * dw + current_lateral_pos
        new_x, new_y = convert_distance_and_offset_to_XY(
                edge_id, lane_id, new_line_pos, new_offset
        )
        new_angle = traci.lane.getAngle(lane_id, new_line_pos)

        trajectory.append(
            [
                new_x,
                new_y,
                new_angle,
                speed,  # speed
                current_timestamp + (i+1) * dt,  # time
            ]
        )
        # lane_center_list.append(
        #     convert_distance_and_offset_to_XY(
        #         edge_id, lane_id, new_line_pos, 0
        #     )
        # )

    # move back to the original lane
    for i in range(steps):
        new_line_pos = current_lane_position + (steps + i + 1) * dl
        new_offset = (steps - i - 1) * dw + current_lateral_pos
        new_x, new_y = convert_distance_and_offset_to_XY(
                edge_id, lane_id, new_line_pos, new_offset
        )
        new_angle = traci.lane.getAngle(lane_id, new_line_pos)
        trajectory.append(
            [
                new_x,
                new_y,
                new_angle,
                speed,  # speed
                current_timestamp + (steps + i + 1) * dt,  # time
            ]
        )
        # lane_center_list.append(
        #     convert_distance_and_offset_to_XY(
        #         edge_id, lane_id, new_line_pos, 0
        #     )
        # )

    return NDECommand(
            command_type=CommandType.TRAJECTORY,
            future_trajectory=trajectory,
            duration=duration,
            time_resolution=dt,
            keep_route_mode=2
        )


def convert_distance_and_offset_to_XY(edgeID, laneID, distance, offset):
    """Convert from lane coordinates to XY coordinates. Note: distance should be smaller than the lane length.

    Args:
        edgeID (str): Edge ID.
        laneID (str): Lane ID.
        distance (float): Distance along the lane.
        offset (float): Offset from the lane centerline.
    
    Returns:
        float, float: XY coordinates.
    """
    laneIndex = laneID.split('_')[-1]
    laneIndex = int(laneIndex)
    
    x_base, y_base = traci.simulation.convert2D(edgeID, distance, laneIndex)

    angle = traci.lane.getAngle(laneID, distance)
    angle_offset = angle + 90 if offset > 0 else angle - 90
    angle_offset = math.radians(angle_offset % 360)

    x = x_base + abs(offset) * math.sin(angle_offset)
    y = y_base + abs(offset) * math.cos(angle_offset)

    return x, y
