from abc import ABC, abstractmethod


class TrafficLightFactory(ABC):
    """
    Basic traffic light factory class to help build a traffic light, each traffic light will contain three major components: sensors, controllers, and powertrains.
    Each user who would like to build a customized traffic light should build a son-class and overwrite the traffic light creation method.
    """

    @abstractmethod
    def create_traffic_light(self, tls_id, simulator):
        """Create a traffic light with the given id and simulator.

        Args:
            tls_id (str): The traffic light id.
            simulator (Simulator): The simulator object.

        Returns:
            TrafficLight: The traffic light object.
        """
        raise NotImplementedError("Create traffic light method not implemented!")
