"""This module defines the interface for agents
"""

import addict
from attrs import define
from typing import Any, Dict, Iterable, NewType

from terasim.agent.agent_decision_model import AgentDecisionModel
from terasim.agent.agent_sensor import AgentSensor

AgentId = NewType("AgentId", str)


class AgentType:
    def __init__(self) -> None:
        pass

    @staticmethod
    def default() -> "AgentType":
        return AgentType()


@define
class AgentDepartureInfo:
    time: float = None  # Time step at which the vehicle should enter the network. Defaults to None.
    lane: str = "first"  # Lane on which the vehicle should be inserted. Defaults to 'first'.
    lane_id: str = None  # specific lane id where vehicle should be inserted
    position: str = (
        "base"  # Position at which the vehicle should enter the net. Defaults to 'base'.
    )
    speed: str = "0"  # Speed with which the vehicle should enter the network. Defaults to '0'.


@define
class AgentArrivalInfo:
    lane: str = (
        "current"  # Lane at which the vehicle should leave the network. Defaults to 'current'.
    )
    position: str = (
        "max"  # Position at which the vehicle should leave the network. Defaults to 'max'.
    )
    speed: str = (
        "current"  # Speed with which the vehicle should leave the network. Defaults to 'current'.
    )


@define
class AgentInitialInfo:
    route: str
    type: AgentType = AgentType.default()
    depart: AgentDepartureInfo = AgentDepartureInfo()
    arrive: AgentArrivalInfo = AgentArrivalInfo()

    from_taz: str = (
        ""  # Traffic assignment zone where the vehicle should enter the network. Defaults to ''.
    )
    to_taz: str = (
        ""  # Traffic assignment zones where the vehicle should leave the network. Defaults to ''.
    )
    line: str = ""  # A string specifying the id of a public transport line which can be used when specifying person rides. Defaults to ''.
    person_capacity: int = 0  # Number of all seats of the added vehicle. Defaults to 0.
    person_number: int = 0  # Number of occupied seats when the vehicle is inserted. Defaults to 0.


class Agent:
    """
    A basic class holds the essential information for agents (vehicles, pedestrians, traffic lights, etc.) in the simulator
    """

    DEFAULT_PARAMS = dict(
        agent_type="DefaultAgent",
    )

    def __init__(
        self,
        id: AgentId,
        simulator: Any,
        sensors: Iterable[AgentSensor] = [],
        decision_model=None,
        controller=None,
        **params,
    ):
        """Initialize the agent.

        Args:
            id (AgentId): The ID of the agent.
            simulator (Any): The simulator object.
            sensors (Iterable[AgentSensor], optional): The sensors of the agent. Defaults to [].
            decision_model (AgentDecisionModel, optional): The decision model of the agent. Defaults to None.
            controller (AgentController, optional): The controller of the agent. Defaults to None.
            params (dict, optional): The parameters of the agent. Defaults to {}.
        """
        self._id = id
        self._simulator = simulator
        self._params = addict.Dict(self.DEFAULT_PARAMS)
        self._params.update(params)

        self.sensors: Dict[str, AgentSensor] = {}
        for s in sensors:
            if s.name in sensors:
                raise ValueError("Multiple sensors with the same name!")
            self.sensors[s.name] = s

        if not isinstance(decision_model, AgentDecisionModel):
            raise ValueError("Installing non-decision_model instance as decision_model!")
        self.decision_model = decision_model

        self.controller = controller

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"{self._params.agent_type}(id: {self.id})"

    @property
    def id(self):
        return self._id

    @property
    def observation(self):
        return self._fetch_observation()

    @property
    def params(self) -> addict.Dict:
        return self._params

    @property
    def simulator(self):
        return self._simulator

    def _install(self):
        """
        This method is designed to be called after the vehicle exists in the simulator. It installs
        the attaching objects (including sensors, the decision model and controller).
        """
        # install sensors
        for name, sensor in self.sensors.items():
            sensor._install(self)

        # install decision model
        if not isinstance(self.decision_model, AgentDecisionModel):
            raise ValueError("Installing non-decision_model instance as decision_model!")
        self.decision_model._install(self)

        # install controller
        self.controller._install(self)

        # apply params
        # self.simulator.set_vehicle_color(self.id, self.params.properties.color)

    def _fetch_observation(self):
        """Fetch the observation from all sensors.

        Returns:
            dict: The observation dictionary.
        """
        obs_dict = {name: self.sensors[name].observation for name in self.sensors}
        return obs_dict

    def _uninstall(self):
        """This method is designed to be called before the vehicle is removed from the simulator. It uninstalls the attaching sensors.
        """
        # uninstall sensors
        for name, sensor in self.sensors.items():
            sensor._uninstall()

    def apply_control(self, control_command):
        """Apply the control command of given agent.

        Args:
            control_command (dict): The given control command of a specific decision maker.
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

    def make_decision(self):
        """Make decision of control command for the agent.

        Returns:
            dict: The control command.
            dict: The information of the decision.
        """
        obs_dict = self._fetch_observation()
        control_command, info = self.decision_model.derive_control_command_from_observation(
            obs_dict
        )
        return control_command, info


class AgentList(dict):
    def __init__(self, d):
        """An agent list that store agents. It derives from a dictionary so that one can call a certain vehicle in O(1) time. Rewrote the iter method so it can iterate as a list.
        """
        super().__init__(d)

    def __iter__(self):
        for k in self.keys():
            yield self[k]
