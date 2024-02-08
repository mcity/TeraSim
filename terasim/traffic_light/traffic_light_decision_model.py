class TrafficLightDecisionModel():
    def __init__(self):
        self._traffic_light = None
        self._traffic_light_state = None
        self._traffic_light_state_time = None

    def update(self, traffic_light):
        self._traffic_light = traffic_light
        self._traffic_light_state = traffic_light.state
        self._traffic_light_state_time = traffic_light.state_time

    def get_decision(self):
        if self._traffic_light_state == "red":
            return "stop"
        elif self._traffic_light_state == "yellow":
            return "slow"
        elif self._traffic_light_state == "green":
            return "go"
        else:
            return "stop"

    def get_state(self):
        return self._traffic_light_state

    def get_state_time(self):
        return self._traffic_light_state_time

    def get_traffic_light(self):
        return self._traffic_light

    def get_traffic_light_state(self):
        return self._traffic_light_state

    def get_traffic_light_stat