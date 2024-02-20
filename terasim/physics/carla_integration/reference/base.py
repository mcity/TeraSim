from abc import ABC, abstractmethod
from typing import NewType, Union, List, Optional, Dict, Any
from terasim.simulator import Simulator
from terasim.agent.agent import Agent, AgentId, AgentInitialInfo

# TODO: define these types or combine with types in other modules

VehicleLightState = NewType('VehicleLightState', Any) # states of the vehicle lights
VehicleCommand = NewType('VehicleCommand', Any) # steering, throttle
SensorData = NewType('SensorData', dict) # data of all the registered sensors

class PhysicSimulator(ABC):
    def __init__(self, **connect_args) -> None:
        '''
        Creates an interface of the simulator which connects to a simulator
        :param connect_args: Specify arguments for the connection. This includes host and port for Carla.
        '''
        self._simulator = None # to be assigned from outside

    @abstractmethod
    def initialize(self, simulator: Simulator, ctx) -> bool:
        '''
        This method initializes the simulation environment for a new test.

        Specifically for Carla, possible aspects include map selection, weather selection, etc.

        :return: Returns False when failed.

        TODO: The specific arguments to be accepted is to be finalized.
        '''
        pass

    @abstractmethod
    def dispose(self, simulator: Simulator, ctx) -> bool:
        '''
        This method is called when the simulation stops
        '''
        pass

    @abstractmethod
    def add_agent(self, agent: Agent, init_info: AgentInitialInfo) -> bool:
        '''
        Spawn an agent in the simulation. The actor can be a vehicle / pedestrian etc.

        :param parameters: Specify the parameters of spawned actor. The parameter includes the initial pose of the actor.
        :param agent_id: Specify the id of the actor. The id will be associated with the internal ID of the simulator.
        :param attach_to: For sensors, this argument allows the sensor to be attached to another actor.
        :return: Returns False when failed.
        '''
        pass

    @abstractmethod
    def remove_agent(self, agent: Agent) -> bool:
        '''
        Destroy an agent in the simulation according to the ID.

        Note that all the actors attached to the actor to be removed will also be destroyed.

        :return: Returns whether the destroy succeeded.
        '''
        pass

    # TODO: @abstractmethod
    def get_agent_state(self, agent_id: AgentId) -> Dict[str, Any]:
        '''
        Get the state of an actor

        TODO: which states to be reported? This can also be a in-demand getter.
              Might need another argument to define which info to be retrieved.
        '''
        pass

    # TODO: @abstractmethod
    def set_agent_state(self,
            agent_id: AgentId,
            pose = None,
            lights: Optional[VehicleLightState] = None,
            **kwargs
        ) -> bool:
        '''
        Set the state of an actor. Arguments with None values will be ignored.

        :return: Returns whether the change is successful.

        TODO: which states to be accepted?
        '''
        pass

    # TODO: @abstractmethod
    def set_agent_command(self, actor_id: AgentId, control: VehicleCommand) -> bool:
        '''
        Set the command of an actor.

        :return: Returns whether the change is successful.
        '''
        pass

    @abstractmethod
    def step(self, simulator: Simulator, ctx) -> None:
        '''
        Send all commands to the simulator and step the simulation by one step, all the states will be updated.

        :return: The generated sensor data.
        '''
        pass

    ##### Methods provide to the simulator #####

    def inject(self, simulator: Simulator):
        self._simulator = simulator

        simulator._add_vehicle_to_sim.hook("physic_add", self.add_agent, priority=-100)
        simulator._remove_vehicle_from_sim.hook("physic_remove", self.remove_agent, priority=-100)
        simulator.start_pipeline.hook("physic_start", self.initialize, priority=-100)
        simulator.step_pipeline.hook("physic_step", self.step, priority=-100)
        simulator.stop_pipeline.hook("physic_stop", self.dispose, priority=-100)
