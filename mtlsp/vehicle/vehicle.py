from copy import copy
from typing import Iterable, Dict
from mtlsp.vehicle.decision_models.base_decision_model import BaseDecisionModel
from mtlsp.vehicle.sensors import BaseSensor
from mtlsp.agent import Agent
import logging

class Vehicle(Agent):
    COLOR_RED    = (255, 0,   0)
    COLOR_YELLOW = (255, 255, 0)
    COLOR_BLUE   = (0,   0,   255)
    COLOR_GREEN  = (0,   255, 0)

    DEFAULT_PARAMS = dict(
        properties = { "color": COLOR_YELLOW },
        initial_info = {},
        sync_range = 120, # agents within this range of this vehicle will be synchronized
    )

    def __init__(self, id, simulator,
                 sensors: Iterable[BaseSensor] = [],
                 decision_model=None,
                 controller=None,
                 **params):
        super().__init__(id, simulator, **params)

        self.sensors: Dict[str, BaseSensor] = {}
        for s in sensors:
            if s.name in sensors:
                raise ValueError("Multiple sensors with the same name!")
            self.sensors[s.name] = s
            
        if not isinstance(decision_model, BaseDecisionModel):
            raise ValueError("Installing non-decision_model instance as decision_model!")
        self.decision_model = decision_model

        self.controller = controller

    def _install(self):
        '''
        This method is designed to be called after the vehicle exists in the simulator. It installs
        the attaching objects (including sensors, the decision model and controller).
        '''
        # install sensors
        for name, sensor in self.sensors.items():
            sensor.install(self)
            self._simulator.state_manager.register_sensor(self, name)

        # install decision model
        if not isinstance(self.decision_model, BaseDecisionModel):
            raise ValueError("Installing non-decision_model instance as decision_model!")
        self.decision_model.vehicle = self
        self.decision_model.install()

        # install controller
        self.controller._install(self)

        # apply params
        self.simulator.set_vehicle_color(self.id, self.params.properties.color)

    def _uninstall(self):
        # uninstall sensors
        for name, sensor in self.sensors.items():
            self._simulator.state_manager.unregister_sensor(self, name)
            sensor._agent = None # remove back-reference

    def __str__(self):
        return f'Vehicle(id: {self.id})'

    def __repr__(self):
        return self.__str__()
    
    @property
    def observation(self):
        return self._fetch_observation()

    def _fetch_observation(self):
        obs_dict = {name: self.sensors[name].observation for name in self.sensors}
        return obs_dict

    def make_decision(self):
        """make decision of control command for a vehicle 

        Returns:
            control_command : dict
        """
        obs_dict = self._fetch_observation()
        control_command, info = self.decision_model.derive_control_command_from_observation(obs_dict)
        return control_command, info
    
    def apply_control(self, control_command):
        """apply the control command of given vehicle

        Args:
            control_command (dict): the given control command of a specific decision_maker
        """
        obs_dict = self._fetch_observation()
        if type(control_command) == list:
            for c in control_command:
                self.apply_control(c)
        else:
            if self.controller._is_command_legal(self.id, control_command):
                self.controller.execute_control_command(self.id, control_command, obs_dict)
            else:
                # logging.warning(f"Control command {control_command} is not legal for Vehicle {id}")
                pass

class VehicleList(dict):
    def __init__(self, d):
        """A vehicle list that store vehicles. It derives from a dictionary so that one can call a certain vehicle in O(1) time. Rewrote the iter method so it can iterate as a list.
        """
        super().__init__(d)

    def __add__(self, another_vehicle_list):
        if not isinstance(another_vehicle_list, VehicleList):
            raise TypeError('VehicleList object can only be added to another VehicleList')
        vehicle_list = copy(self)
        keys = self.keys()
        for v in another_vehicle_list:
            if v.id in keys:
                print(f'WARNING: vehicle with same id {v.id} is added and overwrote the vehicle list')
            vehicle_list[v.id] = v
        return vehicle_list

    def add_vehicles(self, vlist):
        """Add vehicles to the vehicle list.

        Args:
            vlist (list(Vehicle)): List of Vehicle object or a single Vehicle object.
        """
        if not isinstance(vlist, list):
            vlist = [vlist]

        for v in vlist:
            if v.id in self.keys():
                print(f'WARNING: vehicle with same id {v.id} exists and this vehicle is dumped and not overriding the vehicle with same id in the original list')
                continue
            self[v.id] = v    

    def __iter__(self):
        for k in self.keys():
            yield self[k]