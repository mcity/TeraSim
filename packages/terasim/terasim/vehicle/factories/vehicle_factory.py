from abc import ABC, abstractmethod


class VehicleFactory(ABC):
    """
    Basic Vehicle factory class to help build a vehicle, each vehicle will contain three major components: sensors, controllers, and decision models.
    Each user who would like to build a customized vehicle should build a son-class and overwrite the vehicle creation method.
    """

    @abstractmethod
    def create_vehicle(self, veh_id, simulator):
        """Create a vehicle with the given id and simulator.

        Args:
            veh_id (str): The vehicle id.
            simulator (Simulator): The simulator object.

        Returns:
            Vehicle: The vehicle object.
        """
        raise NotImplementedError("Create vehicle method not implemented!")
