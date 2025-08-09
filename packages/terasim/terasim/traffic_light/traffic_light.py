from copy import copy

from terasim.agent.agent import Agent, AgentList


class TrafficLight(Agent):
    DEFAULT_PARAMS = dict(
        agent_type="TrafficLight",
    )


class TrafficLightList(AgentList):
    def __add__(self, another_tls_list):
        """Add two TrafficLightList objects.

        Args:
            another_tls_list (TrafficLightList): Another TrafficLightList object.

        Returns:
            TrafficLightList: The combined TrafficLightList object.
        """
        if not isinstance(another_tls_list, TrafficLightList):
            raise TypeError("TrafficLightList object can only be added to another TrafficLightList")
        tls_list = copy(self)
        keys = self.keys()
        for tls in another_tls_list:
            if tls.id in keys:
                print(
                    f"WARNING: traffic light with same id {tls.id} is added and overwrote the traffic light list"
                )
            tls_list[tls.id] = tls
        return tls_list

    def add_trafficlight(self, tlslist):
        """Add traffic light to the traffic light list.

        Args:
            tlslist (TrafficLightList): List of TrafficLight object or a single TrafficLight object.
        """
        if not isinstance(tlslist, list):
            tlslist = [tlslist]

        for tls in tlslist:
            if tls.id in self.keys():
                print(
                    f"WARNING: traffic light with same id {tls.id} exists and this traffic light is dumped and not overriding the traffic light with same id in the original list"
                )
                continue
            self[tls.id] = tls
