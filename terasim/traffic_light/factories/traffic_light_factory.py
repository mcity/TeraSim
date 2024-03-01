from abc import ABC
from abc import abstractmethod


class TrafficLightFactory(ABC):
    """
        Basic Vehicle factory class to help build a vehicle, each vehicle will contain three major components: sensors, controllers, and powertrains.
        Each user who would like to build a customized vehicle should build a son-class and overwrite the vehicle creation method.
    """   
    
    @abstractmethod
    def create_traffic_light(self, tls_id, simulator):
        raise ValueError("Create vehicle method not implemented!")