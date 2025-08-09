import addict
import math

from terasim.overlay import traci
from terasim.params import AgentType

from ...utils import AbstractAdversity, CommandType, NDECommand


class RunningRedLightAdversity(AbstractAdversity):
    def __init__(self, location, ego_type, probability, predicted_collision_type):
        """Initialize the RunningRedLightAdversity module.

        Args:
            location (str): Location of the adversarial event.
            ego_type (str): Type of the ego agent.
            probability (float): Probability of the adversarial event.
            predicted_collision_type (str): Predicted collision type.
        """
        super().__init__(location, ego_type, probability, predicted_collision_type)
        self.sumo_net = None
        self.next_crossroad_id = None

    def trigger(self, obs_dict) -> bool:
        """Determine when to trigger the RunningRedLightAdversity module.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            bool: Flag to indicate if the RunningRedLightAdversity module should be triggered.
        """
        self._adversarial_command_dict = addict.Dict()

        ego_id = obs_dict["ego"]["vru_id"]
        if traci.person.getSpeed(ego_id) > 0.2:
            return False
        # vru is not moving
        lane_id = traci.person.getLaneID(ego_id)
        tls_list = traci.trafficlight.getIDList()
        for tls_id in tls_list:
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
            if (
                lane_id in controlled_lanes
                and traci.lane.getLength(lane_id) - traci.person.getLanePosition(ego_id)
                < 3
            ):
                # vru is at a walking area controlled by a traffic light
                # vru is at the end of the lane
                next_road_id = traci.person.getNextEdge(ego_id)
                if "c" in next_road_id:
                    # vru is going to cross the road
                    all_traffic_light_state = traci.trafficlight.getRedYellowGreenState(
                        tls_id
                    )
                    controlled_links = traci.trafficlight.getControlledLinks(tls_id)
                    for index in range(len(controlled_links)):
                        if (
                            controlled_links[index]
                            and lane_id in controlled_links[index][0]
                        ):
                            if all_traffic_light_state[index] == "r":
                                # the traffic light is red
                                self.next_crossroad_id = next_road_id
                                return True
        return False

    def derive_command(self, obs_dict) -> addict.Dict:
        """Derive the adversarial command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            addict.Dict: Adversarial command.
        """
        if self._probability > 0 and self.trigger(obs_dict):
            # derive the trajectory command for the vru, which should have the following characteristics:
            # 1. The vru will cross the road in a straight line with the current angle
            # 2. The vru will use the max speed to cross the road
            ego_id = obs_dict["ego"]["vru_id"]
            speed = traci.person.getMaxSpeed(ego_id)
            distance = speed * (traci.simulation.getDeltaT())
            lane_shape = traci.lane.getShape(self.next_crossroad_id + "_0")

            # determine which end of the lane is the starting point
            ego_pose = obs_dict["ego"]["position"]
            distance_to_shape0 = (ego_pose[0]-lane_shape[0][0])**2 + (ego_pose[1]-lane_shape[0][1])**2
            distance_to_shape1 = (ego_pose[0]-lane_shape[-1][0])**2 + (ego_pose[1]-lane_shape[-1][1])**2
            if distance_to_shape0 < distance_to_shape1:
                # the starting point is the first point of the lane
                future_angle = math.degrees(
                    math.atan2(
                        lane_shape[-1][0] - lane_shape[0][0],
                        lane_shape[-1][1] - lane_shape[0][1],
                    )
                )
            else:
                # the starting point is the last point of the lane
                future_angle = math.degrees(
                    math.atan2(
                        lane_shape[0][0] - lane_shape[-1][0],
                        lane_shape[0][1] - lane_shape[-1][1],
                    )
                )

            radians = math.radians(future_angle)
            current_time = traci.simulation.getTime()
            dt = traci.simulation.getDeltaT()
            current_pos = traci.person.getPosition(ego_id)
            total_length = traci.lane.getLength(self.next_crossroad_id + "_0")
            duration = round(total_length / speed, 1)
            trajectory = []
            new_pos = (
                current_pos[0] + distance * math.sin(radians),
                current_pos[1] + distance * math.cos(radians),
            )
            for i in range(int(duration / dt)):
                trajectory.append(
                    [
                        new_pos[0] + i * distance * math.sin(radians),  # x
                        new_pos[1] + i * distance * math.cos(radians),  # y
                        future_angle,  # angle
                        speed,  # speed
                        current_time + i * dt,  # time
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
                    "angle": future_angle,
                    "mode": "adversarial",
                    "adversarial_mode": "RunningRedLight",
                    "time_resolution": 0.1,
                    "predicted_collision_type": self._predicted_collision_type,
                    "location": self._location,
                }
            )
            self._adversarial_command_dict.update(
                addict.Dict({"RunningRedLight": adversarial_command})
            )
            return self._adversarial_command_dict
        return addict.Dict()
