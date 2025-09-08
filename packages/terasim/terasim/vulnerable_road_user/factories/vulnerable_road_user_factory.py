from abc import ABC, abstractmethod


class VulnerableRoadUserFactory(ABC):
    """
    Basic VulnerableRoadUserFactory class to help build a vru, each vru will contain three major components: sensors, controllers, and decision models.
    Each user who would like to build a customized vehicle should build a son-class and overwrite the vehicle creation method.
    """

    @abstractmethod
    def create_vulnerable_road_user(self, vru_id, simulator):
        raise NotImplementedError("Create vulnerable road user method not implemented!")
