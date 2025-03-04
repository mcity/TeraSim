from copy import copy

from terasim.agent.agent import Agent, AgentList


class VulnerableRoadUser(Agent):
    COLOR_RED = (255, 0, 0)
    COLOR_YELLOW = (255, 255, 0)
    COLOR_BLUE = (0, 0, 255)
    COLOR_GREEN = (0, 255, 0)

    DEFAULT_PARAMS = dict(
        agent_type="VulnerableRoadUser",
        properties={"color": COLOR_BLUE},
        initial_info={},
        sync_range=120,  # agents within this range of this vulnerable_road_user will be synchronized
    )


class VulnerableRoadUserList(AgentList):
    def __add__(self, another_vru_list):
        """Add two VulnerableRoadUserList objects.

        Args:
            another_vru_list (VulnerableRoadUserList): Another VulnerableRoadUserList object.
        
        Returns:
            VulnerableRoadUserList: The combined VulnerableRoadUserList object.
        """
        if not isinstance(another_vru_list, VulnerableRoadUserList):
            raise TypeError(
                "VulnerableRoadUserList object can only be added to another VulnerableRoadUserList"
            )
        vru_list = copy(self)
        keys = self.keys()
        for vru in another_vru_list:
            if vru.id in keys:
                print(
                    f"WARNING: vulnerable road user with same id {vru.id} is added and overwrote the vulnerable road user list"
                )
            vru_list[vru.id] = vru
        return vru_list

    def add_vulnerable_road_users(self, vru_list):
        """Add vrus to the vru list.

        Args:
            vru_list (VulnerableRoadUserList): List of vru object or a single vru object.
        """
        if not isinstance(vru_list, list):
            vru_list = [vru_list]

        for vru in vru_list:
            if vru.id in self.keys():
                print(
                    f"WARNING: vulnerable road user with same id {vru.id} exists and this vulnerable road user is dumped and not overriding the vulnerable road user with same id in the original list"
                )
                continue
            self[vru.id] = vru
