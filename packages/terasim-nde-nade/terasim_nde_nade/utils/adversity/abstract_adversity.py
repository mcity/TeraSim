import abc
from abc import abstractmethod
import addict
from typing import Any, Dict
import uuid

class AbstractAdversity(abc.ABC):
    def __new__(cls, *args: Any, **kwargs: Any):
        instance: AbstractAdversity = super().__new__(cls)
        instance._adversity_output = []
        return instance

    def __init__(
        self,
        location,
        ego_type,
        probability,
        predicted_collision_type,
    ):
        """Initialize the AbstractAdversity class. This class is an abstract class that defines the interface for the different types of adversities that can be triggered in the simulation.

        Args:
            location (str): Location of the adversarial event.
            ego_type (str): Type of the ego agent.
            probability (float): Probability of the adversarial event.
            predicted_collision_type (str): Predicted collision type.
        """
        self._location = location
        self._ego_type = ego_type
        self._probability = float(probability)
        self._predicted_collision_type = predicted_collision_type
        self._adversarial_command_dict = addict.Dict()

    @abstractmethod
    def trigger(self, obs_dict: Dict) -> bool:
        """Determine when to trigger the adversarial event.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            bool: Flag to indicate if the adversarial event should be triggered.
        """
        pass

    @abstractmethod
    def derive_command(self, obs_dict: Dict) -> addict.Dict:
        """Derive the adversarial command based on the observation.

        Args:
            obs_dict (dict): Observation of the ego agent.

        Returns:
            addict.Dict: Adversarial command.
        """
        pass


class AbstractStaticAdversity(abc.ABC):
    def __new__(cls, *args: Any, **kwargs: Any):
        instance: AbstractStaticAdversity = super().__new__(cls)
        instance._adversity_output = []
        return instance

    def __init__(
        self,
        lane_id="",
        lane_position=-1,
        start_time=0,
        end_time=-1,
        object_type="",
        other_settings=None,
        placement_mode="lane_position",
        x=None,
        y=None,
        angle=None
    ):
        """Initialize the AbstractStaticAdversity class. This class is an abstract class that defines the interface for the different types of adversities that can be triggered in the simulation.

        Args:
            lane_id (str): Lane ID of the adversarial event.
            lane_position (int): Lane position of the adversarial event. Default is -1.
            start_time (float): Start time of the adversarial event. Default is 0.
            end_time (float): End time of the adversarial event. Default is -1 (infinite).
            object_type (str): Type of the object. Default is an empty string.
            other_settings (dict): Other settings for the adversarial event. Default is None.
            placement_mode (str): Placement mode - "lane_position" or "xy_angle". Default is "lane_position".
            x (float): X coordinate for xy_angle placement mode. Default is None.
            y (float): Y coordinate for xy_angle placement mode. Default is None.
            angle (float): Angle for xy_angle placement mode. Default is None.
        """
        self._adversity_id = uuid.uuid4()
        self._lane_id = lane_id
        self._lane_position = lane_position
        self._object_type = object_type
        self._static_adversarial_object_id_list = []
        self._other_settings = other_settings
        self._start_time = start_time
        self._end_time = end_time
        self._placement_mode = placement_mode
        self._x = x
        self._y = y
        self._angle = angle

    @property
    def start_time(self):
        return self._start_time

    @property
    def end_time(self):
        return self._end_time
    
    @property
    def adversity_id(self):
        return self._adversity_id

    @abstractmethod
    def is_effective(self) -> bool:
        """Check if the adversarial event is effective.

        Returns:
            bool: Flag to indicate if the adversarial event is effective.
        """
        pass

    @abstractmethod
    def initialize(self, time: float):
        """Initialize the adversarial event.
        """
        pass

    @abstractmethod
    def update(self, time: float):
        """Update the adversarial event.
        """
        pass
