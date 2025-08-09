"""TeraSim NDE/NADE utilities package."""
from .adversity import (
    AbstractAdversity,
    AbstractStaticAdversity,
    build_adversities,
    build_static_adversities,
    AdversityManager,
    StaticAdversityManager,
    derive_follower_adversarial_command,
    derive_lane_change_abort_adversarial_command,
    derive_lane_change_adversarial_command,
    derive_leader_adversarial_command,
    derive_merge_adversarial_command_speeding,
    derive_merge_adversarial_command_lanechange,
    exist_merging_vehicle,
    derive_traffic_rule_adversarial_command,
)
from .agents import (
    is_car_following,
    is_lane_changing,
)
from .base import (
    CommandType,
    NDECommand,
)
from .collision import (
    get_collision_type_and_prob,
    get_location,
    is_head_on,
)
from .geometry import calclulate_distance_from_centered_agent
from .nade import (
    get_environment_maneuver_challenge,
    add_avoid_accept_collision_command,
    get_environment_avoidability,
    modify_nde_cmd_veh_using_avoidability,
    remove_collision_avoidance_command_using_avoidability,
    apply_collision_avoidance,
    get_environment_criticality,
    get_nde_cmd_from_cmd_info,
    update_nde_cmd_to_vehicle_cmd_info,
    update_control_cmds_from_predicted_trajectory,
    adversarial_hook
)
from .trajectory import (
    predict_environment_future_trajectory,
    interpolate_future_trajectory,
)

__all__ = [
    "AbstractAdversity",
    "AbstractStaticAdversity",
    "build_adversities",
    "build_static_adversities",
    "AdversityManager",
    "StaticAdversityManager",
    "derive_follower_adversarial_command",
    "derive_lane_change_abort_adversarial_command",
    "derive_lane_change_adversarial_command",
    "derive_leader_adversarial_command",
    "derive_merge_adversarial_command_speeding",
    "derive_merge_adversarial_command_lanechange",
    "exist_merging_vehicle",
    "derive_traffic_rule_adversarial_command",
    "is_car_following",
    "is_lane_changing",
    "CommandType",
    "NDECommand",
    "get_collision_type_and_prob",
    "get_location",
    "is_head_on",
    "calclulate_distance_from_centered_agent",
    "get_environment_maneuver_challenge",
    "add_avoid_accept_collision_command",
    "get_environment_avoidability",
    "modify_nde_cmd_veh_using_avoidability",
    "remove_collision_avoidance_command_using_avoidability",
    "apply_collision_avoidance",
    "get_environment_criticality",
    "get_nde_cmd_from_cmd_info",
    "update_nde_cmd_to_vehicle_cmd_info",
    "update_control_cmds_from_predicted_trajectory",
    "adversarial_hook",
    "predict_environment_future_trajectory",
    "interpolate_future_trajectory",
]