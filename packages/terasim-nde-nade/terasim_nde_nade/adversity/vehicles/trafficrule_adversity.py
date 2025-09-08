import addict

from ...utils import (
    AbstractAdversity,
    derive_traffic_rule_adversarial_command,
    get_location,
)


class TrafficRuleAdversity(AbstractAdversity):
    def trigger(self, obs_dict) -> bool:
        """Determine when to trigger the TrafficRuleAdversity module.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            bool: Flag to indicate if the TrafficRuleAdversity module should be triggered.
        """
        self._adversarial_command_dict = addict.Dict()
        vehicle_location = get_location(
            obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"], obs_dict=obs_dict
        )
        if vehicle_location == self._location:
            adversarial_command_dict = derive_traffic_rule_adversarial_command(obs_dict)
            for key, command in adversarial_command_dict.items():
                if command.info.get("adversarial_mode", None) == "TrafficRule":
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
