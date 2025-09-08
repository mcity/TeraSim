from addict import Dict
import random


from .adversity_builder import build_adversities, build_static_adversities

from ..base import (
    CommandType, 
    NDECommand,
)


class AdversityManager:
    def __init__(self, config):
        """Initialize the AdversityManager module.

        Args:
            config (dict): Configuration of the adversities.
        """
        self.config = config
        self.adversities = []
        if self.config is not None:
            self.adversities = build_adversities(self.config)

    def derive_command(self, obs_dict):
        """Derive the command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            NDECommand: Command to be executed by the ego agent.
            dict: Dictionary containing the normal and adversarial maneuvers.
        """
        # get adversarial command candidates
        adversarial_command_dict = Dict()
        for adversity in self.adversities:
            adversity_output = adversity.derive_command(obs_dict)
            if adversity_output:
                adversarial_command_dict.update(Dict(adversity_output))
                break

        # Create command_dict based on filtered adversarial commands
        command_dict = {}
        if adversarial_command_dict:
            adversarial_command = list(adversarial_command_dict.values())[0]
            command_dict["adversarial"] = adversarial_command
            normal_prob = 1 - adversarial_command.prob
        else:
            normal_prob = 1

        # vehicle_location = get_location(
        #     obs_dict["ego"]["veh_id"], obs_dict["ego"]["lane_id"]
        # )
        command_dict["normal"] = NDECommand(
            command_type=CommandType.DEFAULT,
            prob=normal_prob,
            # info={"vehicle_location": vehicle_location},
        )

        # Sample final command based on the probability in command_dict
        command = random.choices(
            list(command_dict.values()),
            weights=[cmd.prob for cmd in command_dict.values()],
            k=1,
        )[0]

        return command, command_dict
    

class StaticAdversityManager:
    def __init__(self, config):
        """Initialize the StaticAdversityManager module.

        Args:
            config (dict): Configuration of the adversities.
        """
        self.config = config
        self.adversities = []
        if self.config is not None:
            self.adversities = build_static_adversities(self.config)

        self.initialize_status = {}
        for adversity in self.adversities:
            self.initialize_status[adversity.adversity_id] = False
    
    def update(self, time):
        """Update the adversarial events.
        """
        for adversity in self.adversities:
            if not self.initialize_status[adversity.adversity_id] and time >= adversity.start_time:
                adversity.initialize(time=time)
                self.initialize_status[adversity.adversity_id] = True
            elif self.initialize_status[adversity.adversity_id]:
                adversity.update(time=time)
        return
        