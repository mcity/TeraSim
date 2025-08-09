from typing import Tuple
from pydantic import BaseModel

from terasim.agent.agent_controller import AgentController
from terasim.overlay import traci


class SumoMoveCommandSchema(BaseModel):
    position: Tuple[float, float]
    velocity: float
    angle: float
    keepRoute: int = 2
    speedmode: int = 32
    type: str = "SetSumoTransform"


class SUMOMOVEController(AgentController):
    def __init__(self, simulator):
        """Initialize the SUMO move controller.

        Args:
            simulator (Simulator): The simulator object.
        """
        super().__init__(simulator, control_command_schema=SumoMoveCommandSchema)

    def execute_control_command(self, veh_id, control_command, obs_dict):
        """Execute the control command.

        Args:
            veh_id (str): The ID of the vehicle.
            control_command (dict): The control command.
            obs_dict (dict): The observation dictionary.
        """
        self.set_transform_sumo(
            veh_id,
            control_command["position"],
            control_command["velocity"],
            control_command["angle"],
            control_command["keepRoute"],
            control_command["speedmode"],
        )

    def set_transform_sumo(self, veh_id, position, velocity, angle, keepRoute, speedmode):
        """Apply the SUMO movePosition command to the vehicle.

        Args:
            veh_id (str): ID of the vehicle.
            position (tuple): Position of the vehicle.
            velocity (float): Velocity of the vehicle.
            angle (float): Angle of the vehicle.
            keepRoute (int): Mode of keeping route for the vehicle.
            speedmode (int): Mode of setting speed for the vehicle.
        """
        # Move vehicle to specific position, speed, and angle without specifying the lane and the edge
        traci.vehicle.setSpeedMode(veh_id, speedmode)
        traci.vehicle.moveToXY(
            veh_id,
            edgeID="",
            laneIndex=-1,
            x=position[0],
            y=position[1],
            angle=angle,
            keepRoute=keepRoute,
        )
        if velocity is not None:
            traci.vehicle.setSpeed(veh_id, velocity)
