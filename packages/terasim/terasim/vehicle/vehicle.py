from copy import copy

from terasim.agent.agent import Agent, AgentList


class Vehicle(Agent):
    COLOR_RED = (255, 0, 0)
    COLOR_YELLOW = (255, 255, 0)
    COLOR_BLUE = (0, 0, 255)
    COLOR_GREEN = (0, 255, 0)

    DEFAULT_PARAMS = dict(
        agent_type="Vehicle",
        properties={"color": COLOR_YELLOW},
        initial_info={},
        sync_range=120,  # agents within this range of this vehicle will be synchronized
    )


class VehicleList(AgentList):
    def __add__(self, another_vehicle_list):
        """Add two VehicleList objects.

        Args:
            another_vehicle_list (VehicleList): Another VehicleList object.

        Returns:
            VehicleList: The combined VehicleList object.
        """
        if not isinstance(another_vehicle_list, VehicleList):
            raise TypeError("VehicleList object can only be added to another VehicleList")
        vehicle_list = copy(self)
        keys = self.keys()
        for v in another_vehicle_list:
            if v.id in keys:
                print(
                    f"WARNING: vehicle with same id {v.id} is added and overwrote the vehicle list"
                )
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
                print(
                    f"WARNING: vehicle with same id {v.id} exists and this vehicle is dumped and not overriding the vehicle with same id in the original list"
                )
                continue
            self[v.id] = v
