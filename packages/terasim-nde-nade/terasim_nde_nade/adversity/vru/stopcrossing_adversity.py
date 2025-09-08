import addict
import math
import random

from terasim.overlay import traci
from terasim.params import AgentType

from ...utils import AbstractAdversity, CommandType, NDECommand


class StopCrossingAdversity(AbstractAdversity):
    def __init__(self, location, ego_type, probability, predicted_collision_type):
        """Initialize the StopCrossingAdversity module.

        Args:
            location (str): Location of the adversarial event.
            ego_type (str): Type of the ego agent.
            probability (float): Probability of the adversarial event.
            predicted_collision_type (str): Predicted collision type.
        """
        super().__init__(location, ego_type, probability, predicted_collision_type)
        self.sumo_net = None

    def trigger(self, obs_dict) -> bool:
        """Determine when to trigger the StopCrossingAdversity module.

        Args:
            obs_dict (dict): Observation of the ego agent.
        
        Returns:
            bool: Flag to indicate if the StopCrossingAdversity module should be triggered.
        """
        self._adversarial_command_dict = addict.Dict()

        ego_id = obs_dict["ego"]["vru_id"]
        edge_id = traci.person.getRoadID(ego_id)
        return self.sumo_net.getEdge(edge_id).getFunction() == "crossing"

    def derive_command(self, obs_dict) -> addict.Dict:
        """Derive the adversarial command based on the observation.
        
        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            addict.Dict: Adversarial command.
        """
        if self._probability > 0 and self.trigger(obs_dict):
            # derive the trajectory command for the vru, which should have the following characteristics:
            # 1. The vru first stop for 2 seconds
            # 2. The vru will use the max speed to cross the road
            ego_id = obs_dict["ego"]["vru_id"]
            trajectory = []
            current_pos = traci.person.getPosition(ego_id)
            current_angle = traci.person.getAngle(ego_id)
            radians = math.radians(current_angle)
            current_time = traci.simulation.getTime()
            dt = traci.simulation.getDeltaT()
            # stage 1: stop for 2 seconds
            stop_duration = max(int(random.gauss(20, 5)), 0)  # * 0.1s
            for i in range(stop_duration):
                trajectory.append(
                    [
                        current_pos[0],  # x
                        current_pos[1],  # y
                        current_angle,  # angle
                        0,  # speed
                        current_time + i * dt,  # time
                    ]
                )
            # stage 2: cross the road
            speed = traci.person.getMaxSpeed(ego_id)
            distance = speed * (traci.simulation.getDeltaT())
            total_length = traci.lane.getLength(traci.person.getRoadID(ego_id) + "_0")
            duration = round(total_length / speed, 1)
            for i in range(int(duration / dt)):
                trajectory.append(
                    [
                        current_pos[0] + i * distance * math.sin(radians),  # x
                        current_pos[1] + i * distance * math.cos(radians),  # y
                        current_angle,  # angle
                        speed,  # speed
                        current_time + (i + stop_duration) * dt,  # time
                    ]
                )
            adversarial_command = NDECommand(
                agent_type=AgentType.VULNERABLE_ROAD_USER,
                command_type=CommandType.TRAJECTORY,
                future_trajectory=trajectory,
                duration=duration,
                prob=self._probability,
                keep_route_mode=3,
            )
            adversarial_command.info.update(
                {
                    "speed": speed,
                    "angle": current_angle,
                    "mode": "adversarial",
                    "adversarial_mode": "StopCrossing",
                    "time_resolution": 0.1,
                    "predicted_collision_type": self._predicted_collision_type,
                    "location": self._location,
                }
            )
            self._adversarial_command_dict.update(
                addict.Dict({"StopCrossing": adversarial_command})
            )
            return self._adversarial_command_dict
        return addict.Dict()
