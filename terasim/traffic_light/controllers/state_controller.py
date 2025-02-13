import terasim.utils as utils
from terasim.agent.agent_controller import AgentController
from terasim.simulator import traci


class StateController(AgentController):
    params = {}

    def __init__(self, simulator, params=None):
        super().__init__(simulator, params)
        self.is_busy = False
        self.controlled_duration = 0
        self.step_size = utils.get_step_size()

    def set_traffic_light(self, tlsID, state):
        traci.trafficlight.setRedYellowGreenState(tlsID, state)

    def execute_control_command(self, tls_id, control_command, obs_dict):
        # signal control
        self.set_traffic_light(tls_id, control_command)
