from copy import copy
from typing import Iterable, Dict
from terasim.agent.agent_decision_model import AgentDecisionModel
from terasim.agent.agent_sensor import AgentSensor
from terasim.agent.agent import Agent


class TrafficLight(Agent):
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
        if not isinstance(self.decision_model, AgentDecisionModel):
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
