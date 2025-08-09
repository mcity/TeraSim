import addict

from ...params import RAMP_EDGE_FEATURE, HIGHWAY_EDGE_TYPE
from ...utils import (
    AbstractAdversity,
    derive_merge_adversarial_command_speeding,
    derive_merge_adversarial_command_lanechange,
    exist_merging_vehicle,
    get_location,
    is_lane_changing,
)


class MergeAdversity(AbstractAdversity):
    def trigger(self, obs_dict) -> bool:
        """Determine when to trigger the MergeAdversity module.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            bool: Flag to indicate if the MergeAdversity module should be triggered.
        """
        self._adversarial_command_dict = addict.Dict()
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"], highway_speed_threshold=20, obs_dict=obs_dict
        )
        is_lane_changing_flag = is_lane_changing(obs_dict["ego"]["veh_id"], obs_dict)
        exist_merging_vehicle_flag, merging_lane_id = exist_merging_vehicle(obs_dict, sumo_net=get_location.sumo_net_cache)
        # specific location and not doing lane change and exist merging vehicle
        if vehicle_location == self._location and not is_lane_changing_flag and exist_merging_vehicle_flag:
            adversarial_command_dict = {}
            merging_lane_index = int(merging_lane_id.split("_")[-1])
            # lane index == 1 means the vehicle is in the second leftmost lane, then, adversarial behavior is just speeding
            if obs_dict["ego"]["lane_index"] == merging_lane_index + 1:
                adversarial_command_dict = derive_merge_adversarial_command_speeding(obs_dict)
            # lane index == 2 means the vehicle is in the third leftmost lane, then, adversarial behavior is lane change
            elif obs_dict["ego"]["lane_index"] == merging_lane_index + 2:
                adversarial_command_dict = derive_merge_adversarial_command_lanechange(obs_dict)
            for key, command in adversarial_command_dict.items():
                adversarial_mode = command.info.get("adversarial_mode", None)
                if adversarial_mode == "MergeSpeedUp" or adversarial_mode == "MergeLaneChange":
                    self._adversarial_command_dict[key] = command
        if self._adversarial_command_dict:
            return True
        else:
            return False

    def derive_command(self, obs_dict) -> addict.Dict:
        """Derive the adversarial command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            addict.Dict: Adversarial command.
        """
        if self._probability > 0 and self.trigger(obs_dict):
            for key, command in self._adversarial_command_dict.items():
                command.prob = self._probability
                command.info.update(
                    {
                        "predicted_collision_type": self._predicted_collision_type,
                        "vehicle_location": self._location,
                    }
                )
            return self._adversarial_command_dict
        return addict.Dict()
