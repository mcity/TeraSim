from copy import copy
from typing import Iterable, Dict
from terasim.agent.agent_decision_model import AgentDecisionModel
from terasim.agent.agent_sensor import AgentSensor
from terasim.agent.agent import Agent

class TrafficLight(Agent):

    def __init__(self, id, simulator,
                 sensors: Iterable[AgentSensor] = [],
                 decision_model=None,
                 controller=None,
                 **params):
        super().__init__(id, simulator, **params)

        self.sensors: Dict[str, AgentSensor] = {}
        for s in sensors:
            if s.name in sensors:
                raise ValueError("Multiple sensors with the same name!")
            self.sensors[s.name] = s
            
        if not isinstance(decision_model, AgentDecisionModel):
            raise ValueError("Installing non-decision_model instance as decision_model!")
        self.decision_model = decision_model

        self.controller = controller

    def _install(self):
        '''
        This method is designed to be called after the traffic light exists in the simulator. It installs
        the attaching objects (including sensors, the decision model and controller).
        '''
        # install sensors
        for name, sensor in self.sensors.items():
            sensor.install(self)
            self._simulator.state_manager.register_sensor(self, name)

        # install decision model
        if not isinstance(self.decision_model, AgentDecisionModel):
            raise ValueError("Installing non-decision_model instance as decision_model!")
        self.decision_model.traffic_light = self

        # install controller
        self.controller._install(self)

    def _uninstall(self):
        # uninstall sensors
        for name, sensor in self.sensors.items():
            self._simulator.state_manager.unregister_sensor(self, name)
            sensor._agent = None # remove back-reference

    def __str__(self):
        return f'TrafficLight(id: {self.id})'

    def __repr__(self):
        return self.__str__()
    
    @property
    def observation(self):
        return self._fetch_observation()

    def _fetch_observation(self):
        obs_dict = {name: self.sensors[name].observation for name in self.sensors}
        return obs_dict

    def make_decision(self):
        """make decision of control command for a traffic light 

        Returns:
            control_command : dict
        """
        obs_dict = self._fetch_observation()
        control_command, info = self.decision_model.derive_control_command_from_observation(obs_dict)
        return control_command, info
    
    def apply_control(self, control_command):
        """apply the control command of given traffic light

        Args:
            control_command (dict): the given control command of a specific decision_maker
        """
        obs_dict = self._fetch_observation()
        if type(control_command) == list:
            for c in control_command:
                self.apply_control(c)
        else:
            if self.controller._is_command_legal(self.id, control_command):
                self.controller.execute_control_command(self.id, control_command)
            else:
                # logging.warning(f"Control command {control_command} is not legal for Vehicle {id}")
                pass
            
class TrafficLightList(dict):
    def __init__(self, d):
        """A TrafficLight list that store traffic lights. It derives from a dictionary so that one can call a certain traffic light in O(1) time. Rewrote the iter method so it can iterate as a list.
        """
        super().__init__(d)

    def __add__(self, another_tls_list):
        if not isinstance(another_tls_list, TrafficLightList):
            raise TypeError('TrafficLightList object can only be added to another TrafficLightList')
        tls_list = copy(self)
        keys = self.keys()
        for tls in another_tls_list:
            if tls.id in keys:
                print(f'WARNING: traffic light with same id {tls.id} is added and overwrote the traffic light list')
            tls_list[tls.id] = tls
        return tls_list

    def add_trafficlight(self, tlslist):
        """Add vehicles to the traffic light list.

        Args:
            vlist (list(Vehicle)): List of TrafficLight object or a single TrafficLight object.
        """
        if not isinstance(tlslist, list):
            tlslist = [tlslist]

        for tls in tlslist:
            if tls.id in self.keys():
                print(f'WARNING: traffic light with same id {tls.id} exists and this traffic light is dumped and not overriding the traffic light with same id in the original list')
                continue
            self[tls.id] = tls    

    def __iter__(self):
        for k in self.keys():
            yield self[k]
