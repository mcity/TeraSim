from terasim.agent.agent_controller import AgentController
from terasim.overlay import traci
class SUMOMOVEController(AgentController):

    def _is_command_legal(self, veh_id, control_command):
        """Check if the control command is legal.

        Args:
            veh_id (str): ID of the vehicle.
            control_command (dict): Control command.

        Returns:
            bool: True if the control command is legal, False otherwise.
        """
        return control_command["type"] == "SetSumoTransform"

    def execute_control_command(self, veh_id, control_command, obs_dict):
        keepRoute = control_command.get("keepRoute", 2)
        self.set_transform_sumo(veh_id, control_command["position"], control_command["velocity"], control_command["angle"], keepRoute)
    
    def set_transform_sumo(self, veh_id, position, velocity, angle, keepRoute, speedmode=0):
        """Apply the SUMO movePosition command to the vehicle.
        
        Args:
            veh_id (str): ID of the vehicle.
            position (tuple): Position of the vehicle.
            velocity (float): Velocity of the vehicle.
            angle (float): Angle of the vehicle.
        """
        # Move vehicle to specific position, speed, and angle without specifying the lane and the edge
        traci.vehicle.setSpeedMode(veh_id, speedmode)
        traci.vehicle.moveToXY(veh_id, edgeID="", laneIndex=-1, x=position[0], y=position[1], angle=angle, keepRoute=keepRoute)
        if velocity is not None:
            traci.vehicle.setSpeed(veh_id, velocity)