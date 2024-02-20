from terasim.agent.agent_controller import AgentController

class SUMOMOVEController(AgentController):

    def _is_command_legal(self, veh_id, control_command):
        return self.simulator.command_manager._is_command_legal(veh_id, control_command)

    def execute_control_command(self, veh_id, control_command, obs_dict):
        return self.simulator.command_manager.execute_control_command(veh_id, control_command, obs_dict)