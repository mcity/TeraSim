from terasim.agent.agent_controller import AgentController
from terasim.simulator import traci
import terasim.utils as utils


class StateController(AgentController):
    params = {}

    def __init__(self, simulator, params=None):
        """Initialize the traffic light state controller.

        Args:
            simulator (Simulator): The simulator object.
            params (dict, optional): The parameters of the state controller.
        """
        super().__init__(simulator, params)
        self.is_busy = False
        self.controlled_duration = 0
        self.step_size = utils.get_step_size()

    def set_traffic_light(self, tlsID, state):
        """Set the traffic light state.

        Args:
            tlsID (str): The ID of the traffic light.
            state (str): The state of the traffic light.
        """
        traci.trafficlight.setRedYellowGreenState(tlsID, state)

    def execute_control_command(self, tls_id, control_command, obs_dict):
        """Execute the control command.

        Args:
            tls_id (str): The ID of the traffic light.
            control_command (dict): The control command.
            obs_dict (dict): The observation dictionary.
        """
        # signal control
        self.set_traffic_light(tls_id, control_command)
