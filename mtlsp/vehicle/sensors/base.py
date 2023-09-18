from __future__ import annotations

import addict
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from mtlsp import utils
from mtlsp.agent import Agent
if TYPE_CHECKING:
    from mtlsp.simulator import Simulator

class BaseSensor(ABC):
    DEFAULT_PARAMS = {}

    def __init__(self, name = "base", **params):
        """base sensor initialization

        Args:
            params (dict): all sensor paramters
            name (str, optional): define the name of the sensor. Defaults to "base".
        """
        self._agent: Agent = None # to be assigned from outside
        self._name = name
        self._params = addict.Dict(self.DEFAULT_PARAMS)
        self._params.update(params)
    
    def __str__(self) -> str:
        """string method

        Returns:
            str: the name of the sensor
        """
        return self._name

    @property
    def name(self) -> str:
        return self._name

    @property
    def params(self) -> addict.Dict:
        return self._params

    @property
    def is_installed(self) -> bool:
        return self._agent is not None

    @property
    def _simulator(self) -> Simulator:
        assert self.is_installed, "Sensor not installed, you should install sensor before accessing the simulator."
        return self._agent._simulator

    @property
    def observation(self):
        assert self.is_installed, "Sensor not installed, you should install sensor before fetching observations."
        return self._simulator.state_manager.retrieve_variable(self._agent.id, self._name)

    def subscribe(self):
        """Install the subscriber for the sensor into the simulator.
        """
        pass

    def unsubscribe(self):
        """Remove the subscriber for the sensor from the simulator.
        """
        pass

    @abstractmethod
    def fetch(self):
        """Fetch the data from the simulator. For callback-based sensors,
        this method should directly return None.

        Note that this method should not be called by the Env users.
        """
        pass

    def install(self, parent):
        """Install the sensor to the simulator (but not subscribe to the data yet).

        For SUMO sensors, it will be no-op.
        """
        self._agent = parent

    def uninstall(self):
        """Uninstall the sensor from the simulator.

        For SUMO sensors, it will be no-op.
        """
        self._agent = None
