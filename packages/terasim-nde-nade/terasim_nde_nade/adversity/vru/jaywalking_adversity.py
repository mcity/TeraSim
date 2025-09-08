import addict
import math
import numpy as np

from terasim.overlay import traci
from terasim.params import AgentType

from ...utils import AbstractAdversity, CommandType, NDECommand


def is_pedestrian_moving_forward(p_id, sumonet) -> bool:
    """Check if the pedestrian is moving forward.

    Args:
        p_id (str): Pedestrian ID.
        sumonet (SumoNet): SumoNet object.

    Returns:
        bool: Flag to indicate if the pedestrian is moving forward.
    """
    edge_id = traci.person.getRoadID(p_id)
    edge_shape = sumonet.getEdge(edge_id).getShape()
    edge_start = edge_shape[0]
    edge_end = edge_shape[-1]
    if edge_end[1] - edge_start[1] == 0:
        edge_angle = 90 if edge_end[0] > edge_start[0] else 270
    else:
        edge_angle = (
            -math.degrees(
                math.atan((edge_end[0] - edge_start[0]) / (edge_end[1] - edge_start[1]))
            )
            + 90
        ) % 360
    person_angle = traci.person.getAngle(p_id)
    if (edge_angle - person_angle) % 360 < 180:
        return True
    else:
        return False


def is_pedestrian_crossing(p_id, sumonet) -> bool:
    """Check if the pedestrian is crossing.

    Args:
        p_id (str): Pedestrian ID.
        sumonet (SumoNet): SumoNet object.

    Returns:
        bool: Flag to indicate if the pedestrian is crossing.
    """
    edge_id = traci.person.getRoadID(p_id)
    return sumonet.getEdge(edge_id).getFunction() == "crossing"


def is_pedestrian_going_to_cross(p_id, sumonet) -> bool:
    """Check if the pedestrian is going to cross.

    Args:
        p_id (str): Pedestrian ID.
        sumonet (SumoNet): SumoNet object.

    Returns:
        bool: Flag to indicate if the pedestrian is going to cross.
    """
    next_road_id = traci.person.getNextEdge(p_id)
    if next_road_id:
        return sumonet.getEdge(next_road_id).getFunction() == "crossing"
    else:
        return False


class JaywalkingAdversity(AbstractAdversity):
    def __init__(self, location, ego_type, probability, predicted_collision_type):
        """Initialize the JaywalkingAdversity module.

        Args:
            location (str): Location of the adversarial event.
            ego_type (str): Type of the ego agent.
            probability (float): Probability of the adversarial event.
            predicted_collision_type (str): Predicted collision type.
        """
        super().__init__(location, ego_type, probability, predicted_collision_type)
        self.sumo_net = None
        self.closest_vehicle_lane_id = None

    def trigger(self, obs_dict) -> bool:
        """Determine when to trigger the JaywalkingAdversity module.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            bool: Flag to indicate if the JaywalkingAdversity module should be triggered.
        """
        self._adversarial_command_dict = addict.Dict()

        ego_id = obs_dict["ego"]["vru_id"]
        lane_id = traci.person.getLaneID(ego_id)
        
        if is_pedestrian_crossing(ego_id, self.sumo_net) or is_pedestrian_going_to_cross(ego_id, self.sumo_net):
            return False
        
        # find the most closest major lane which allows vehicles 
        lane_id = None
        ego_pose = obs_dict["ego"]["position"]
        neighbor_lanes = self.sumo_net.getNeighboringLanes(
            ego_pose[0],
            ego_pose[1],
            r=10,
        )
        sorted_lanes = sorted(
            neighbor_lanes,
            key=lambda lane: lane[1]
        )
        for lane_info in sorted_lanes:
            lane = lane_info[0]
            if lane.allows("passenger") or lane.allows("truck"):
                self.closest_vehicle_lane_id = lane.getID()
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
            ego_id = obs_dict["ego"]["vru_id"]
            
            ego_pose = obs_dict["ego"]["position"]
            target_pose = None
            # determine the ending pose, choosing from the closest lane
            closest_vehicle_lane_shape = traci.lane.getShape(self.closest_vehicle_lane_id)
            for i in range(len(closest_vehicle_lane_shape) - 1):
                ego_pose_array = np.array(ego_pose)
                s_pose = np.array(closest_vehicle_lane_shape[i])
                e_pose = np.array(closest_vehicle_lane_shape[i + 1])
                s2e = e_pose - s_pose
                s2ego = ego_pose_array - s_pose
                projection_length = np.dot(s2ego, s2e) / np.dot(s2e, s2e)
                projection = s_pose + projection_length * s2e

                if 0 <= projection_length <= 1:
                    # find the closest point on the lane
                    target_pose = projection
                    break
            
            if target_pose is None:
                ego2start = np.linalg.norm(
                    np.array(ego_pose) - np.array(closest_vehicle_lane_shape[0])
                )
                ego2end = np.linalg.norm(
                    np.array(ego_pose) - np.array(closest_vehicle_lane_shape[-1])
                )
                if ego2start < ego2end:
                    target_pose = closest_vehicle_lane_shape[0]
                else:
                    target_pose = closest_vehicle_lane_shape[-1]
            future_angle = math.degrees(
                math.atan2(
                    target_pose[0] - ego_pose[0],
                    target_pose[1] - ego_pose[1],
                )
            )
            # calculate distance to the target pose
            total_length = np.linalg.norm(
                np.array(ego_pose) - np.array(target_pose)
            )
            # calculate the speed of the ego vehicle
            current_speed = traci.person.getSpeed(ego_id)
            if (
                current_speed < 1.0
            ):  # if stop now, then the vru will accelerate to the max speed to cross the road
                speed = traci.person.getMaxSpeed(ego_id)
            else:
                speed = current_speed
            
            speed = speed * np.random.uniform(0.5, 2.0)
            
            total_length = max(total_length, speed * 10)
            duration = round(total_length / speed, 1)

            trajectory = []
            distance = speed * (traci.simulation.getDeltaT())
            radians = math.radians(future_angle)
            new_pos = (
                ego_pose[0] + distance * math.sin(radians),
                ego_pose[1] + distance * math.cos(radians),
            )
            current_time = traci.simulation.getTime()
            dt = traci.simulation.getDeltaT()
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
                duration=min(duration, 5.0),
                prob=self._probability,
                time_resolution=dt,
                keep_route_mode=6,
            )
            adversarial_command.info.update(
                {
                    "speed": speed,
                    "angle": future_angle,
                    "mode": "adversarial",
                    "adversarial_mode": "Jaywalking",
                    "time_resolution": dt,
                    "predicted_collision_type": self._predicted_collision_type,
                    "location": self._location,
                }
            )
            self._adversarial_command_dict.update(
                addict.Dict({"Jaywalking": adversarial_command})
            )
            return self._adversarial_command_dict
        return addict.Dict()
