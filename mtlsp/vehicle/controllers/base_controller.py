class BaseController:
    params = {}
    def __init__(self, simulator, params=None):
        self.simulator = simulator
        if params:
            self.params.update(params)
    
    def _install(self, vehicle):
        pass
    
    def _is_command_legal(self, veh_id, control_command):
        return True

    def execute_control_command(self, veh_id, control_command, obs_dict):
        pass