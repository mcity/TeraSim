from addict import Dict
from loguru import logger

from terasim.agent.agent_controller import AgentController
from terasim.overlay import traci

from ..utils import (
    CommandType,
    NDECommand,
    interpolate_future_trajectory,
)


class NDEVulnerableRoadUserController(AgentController):
    def __init__(self, simulator, params=None):
        """Initialize the controller.

        Args:
            simulator (Simulator): Simulator object.
            params (dict): Parameters for the controller.
        """
        self.is_busy = False
        self.cached_control_command = None  # this is a dict, containing the control command for the vehicle with the timestep information
        self.used_to_be_busy = False
        return super().__init__(
            simulator, control_command_schema=NDECommand, params=params
        )

    def _update_controller_status(self, vru_id, current_time=None):
        """Refresh the state of the controller. This function will be called at each timestep as far as vru is still in the simulator, even if the vru is not controlled.
        
        Args:
            vru_id (str): ID of the vru.
            current_time (int): Current simulation time.
        """
        # if the controller is busy, detect if the current simulation time - the time of the cached control command is greater than the duration of the control command, then the controller is not busy anymore
        if self.is_busy:
            current_time = (
                traci.simulation.getTime() if current_time is None else current_time
            )
            if (
                current_time - self.cached_control_command.timestep
                > self.cached_control_command.cached_command.duration
            ):
                self.is_busy = False
                self.cached_control_command = None
                self.used_to_be_busy = True

    def execute_control_command(self, vru_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            vru_id (str): ID of the vru.
            control_command (dict): Control command for the vru.
            obs_dict (dict): Observation of the vru.
        """
        if self.used_to_be_busy:
            traci.person.remove(vru_id)
            return
        if not self.is_busy:
            if control_command.command_type == CommandType.DEFAULT:
                # all_checks_on(veh_id)
                return
            else:
                # other commands will have duration, which will keep the controller busy
                self.is_busy = True
                self.cached_control_command = Dict(
                    {
                        "timestep": traci.simulation.getTime(),
                        "cached_command": control_command,
                    }
                )
                self.execute_control_command_onestep(
                    vru_id, self.cached_control_command, obs_dict, first_step=True
                )
        else:
            self.execute_control_command_onestep(
                vru_id, self.cached_control_command, obs_dict, first_step=False
            )

    def execute_control_command_onestep(
        self, vru_id, cached_control_command, obs_dict, first_step=False
    ):
        """Execute the control command for one step.

        Args:
            vru_id (str): ID of the vehicle.
            cached_control_command (dict): Cached control command.
            obs_dict (dict): Observation of the vehicle.
            first_step (bool): Flag to indicate if this is the first step of the control command.
        """
        if (
            cached_control_command["cached_command"].command_type
            == CommandType.TRAJECTORY
        ):
            # pass
            self.execute_trajectory_command(
                vru_id, cached_control_command["cached_command"], obs_dict
            )
        else:
            logger.error("Invalid command type")
        return

    @staticmethod
    def execute_trajectory_command(vru_id, control_command, obs_dict):
        """Execute the trajectory command.

        Args:
            vru_id (str): ID of the vru.
            control_command (dict): Control command for the vru.
            obs_dict (dict): Observation of the vru.
        """
        assert control_command.command_type == CommandType.TRAJECTORY
        # get the closest timestep trajectory point in control_command.trajectory to current timestep
        trajectory_array = control_command.future_trajectory
        current_timestep = traci.simulation.getTime()
        closest_timestep_trajectory = min(
            trajectory_array, key=lambda x: abs(x[-1] - current_timestep)
        )
        # set the position of the vehicle to the closest timestep trajectory point
        traci.person.moveToXY(
            personID=vru_id,
            edgeID="",
            x=closest_timestep_trajectory[0],
            y=closest_timestep_trajectory[1],
            angle=closest_timestep_trajectory[2],
            keepRoute=control_command.keep_route_mode,
        )
        logger.info(
            f"Setting position of {vru_id} to {closest_timestep_trajectory[0], closest_timestep_trajectory[1]}, current position is {traci.person.getPosition(vru_id)}"
        )
