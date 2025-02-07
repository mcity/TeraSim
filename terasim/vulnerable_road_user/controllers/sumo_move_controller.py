from terasim.agent.agent_controller import AgentController
from terasim.overlay import traci
from typing import Tuple
from pydantic import BaseModel


class SumoMoveCommandSchema(BaseModel):
    position: Tuple[float, float]
    velocity: float
    angle: float
    keepRoute: int = 2
    type: str = "SetSumoTransform"


class SUMOMOVEController(AgentController):
    def __init__(self, simulator):
        super().__init__(simulator, control_command_schema=SumoMoveCommandSchema)

    def execute_control_command(self, vru_id, control_command, obs_dict):
        self.set_transform_sumo(
            vru_id,
            control_command["position"],
            control_command["velocity"],
            control_command["angle"],
            control_command["keepRoute"],
            control_command["speedmode"],
        )

    def set_transform_sumo(
        self, vru_id, position, angle, keepRoute, velocity
    ):
        """Apply the SUMO movePosition command to the person.

        Args:
            vru_id (str): ID of the person.
            position (tuple): Position of the person.
            angle (float): Angle of the person.
        """
        # Move vehicle to specific position, speed, and angle without specifying the lane and the edge
        traci.person.moveToXY(
            vru_id,
            edgeID="",
            x=position[0],
            y=position[1],
            angle=angle,
            keepRoute=keepRoute,
        )
        if velocity is not None:
            traci.person.setSpeed(vru_id, velocity)
