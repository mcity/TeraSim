"""This module defines the interface for agent sensors
"""

from __future__ import annotations

from abc import ABC, abstractmethod
import addict
from typing import TYPE_CHECKING

from terasim.overlay import traci


if TYPE_CHECKING:
    from terasim.simulator import Simulator


class AgentSensor(ABC):
    DEFAULT_PARAMS = {
        "cache": True,
    }

    def __init__(self, name="base", **params):
        """base sensor initialization

        Args:
            params (dict): all sensor paramters
            name (str, optional): define the name of the sensor. Defaults to "base".
        """
        self._agent = None  # to be assigned from outside
        self._name = name
        self._params = addict.Dict(self.DEFAULT_PARAMS)
        self._params.update(params)
        if self._params.cache:
            self._updated_observation = None
            self._updated_time = None

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
        assert (
            self.is_installed
        ), "Sensor not installed, you should install sensor before accessing the simulator."
        return self._agent._simulator

    @property
    def observation(self):
        assert (
            self.is_installed
        ), "Sensor not installed, you should install sensor before fetching observations."
        if self._params.cache:
            current_time = traci.simulation.getTime()
            if self._updated_time is None or self._updated_time < current_time:
                self._updated_observation = addict.Dict(self.fetch())
                self._updated_time = current_time
            return self._updated_observation
        else:
            return addict.Dict(self.fetch())

    @abstractmethod
    def fetch(self):
        """Fetch the data from the simulator. For callback-based sensors,
        this method should directly return None.

        Note that this method should not be called by the Env users.
        """
        pass

    def _install(self, agent):
        """Install the sensor to the simulator (but not subscribe to the data yet).

        For SUMO sensors, it will be no-op.
        """
        self._agent = agent

    def _uninstall(self):
        """Uninstall the sensor from the simulator.

        For SUMO sensors, it will be no-op.
        """
        pass
