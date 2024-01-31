from typing import Set, Dict, Any
from functools import partial
from collections import namedtuple, defaultdict
import logging

from terasim.overlay import traci
from terasim.agent.agent import AgentId
from terasim.vehicle import Vehicle
from traci import constants as tc

StampedData = namedtuple("StampedData", ["data", "timestamp"])
TerasimValue = namedtuple("TerasimValue", ["data", "timestamp", "last_retrieve_time"])

class StateManager(object):
    
    def __init__(self, simulator, gc_interval=10):
        """Initialize the state manager.

        Args:
            simulator (_type_): sumo simulator
            gc_interval (int, optional): after gc_interval number, the unvisited vehicle information will be unsubscribed to save computational resources. Defaults to 10.
        """
        self._simulator = simulator
        self._gc_interval = gc_interval
        self._last_gc_timestamp = 0

        # A two-layer dictionary to store data
        # First layer: vehicle id -> vehicle data dict (next layer)
        # Second layer: vehicle sensor name -> sensor data (TerasimValue)
        self._sensor_buffer: Dict[AgentId, Dict[str, StampedData]] = defaultdict(dict)

        # A two-layer dictionary to sensor data
        # vehicle id -> { sensor name -> (is_subscribed, last_retrieval_time)}
        self._sensor_states: Dict[AgentId, Any] = defaultdict(dict)

        # A two-layer dictionary to store data
        # First layer: vehicle id -> vehicle subscription dict (next layer)
        # Second layer: vehicle sensor name -> sensor object
        self._sensor_registration: Dict[AgentId, Dict[str, Any]] = defaultdict(dict)

    def register_sensor(self, vehicle: Vehicle, sensor_name: str, subscribe = True):
        veh_id = vehicle.id
        sensor = vehicle.sensors[sensor_name]
        self._sensor_registration[veh_id][sensor_name] = sensor

        if subscribe:
            # subscribe at start
            sensor.subscribe()
            self._sensor_states[veh_id][sensor_name] = (True, traci.simulation.getTime())
        else:
            self._sensor_states[veh_id][sensor_name] = (False, 0)

    def unregister_sensor(self, vehicle: Vehicle, sensor_name: str):
        veh_id = vehicle.id
        if veh_id in self._sensor_registration and sensor_name in self._sensor_registration[veh_id]:
            del self._sensor_registration[veh_id][sensor_name]
            del self._sensor_states[veh_id][sensor_name]

    def push_variable(self, veh_id, var_name, data):
        """Push the observation to the state manager.
        
        Note that this method will not update last retrieval time.
        
        Args:
            veh_id (str): ID of the vehicle.
            variable_name (str): Name of the variable.
            data (float): Value of the variable.
        """
        current_time = traci.simulation.getTime()
        self._sensor_buffer[veh_id][var_name] = StampedData(data, current_time)

    def retrieve_variable(self, veh_id, sensor_name, current_time=None):
        """Get the value of the terasim variable.
        
        Args:
            veh_id (str): ID of the vehicle.
            terasim_variable (str): Name of the terasim variable.
        
        Returns:
            float: Value of the terasim variable.
        """
        current_time = current_time or traci.simulation.getTime()

        assert sensor_name in self._sensor_registration[veh_id]
        sensor = self._sensor_registration[veh_id][sensor_name]
        is_subscribed, retrieval_time = self._sensor_states[veh_id][sensor_name]
        if not is_subscribed:
            sensor.subscribe()
            logging.debug("The sensor {} from vehicle {} subscribed due to retrieving.".format(sensor_name, veh_id))

        # fetch if not present in the buffer
        if sensor_name not in self._sensor_buffer[veh_id] or current_time != self._sensor_buffer[veh_id][sensor_name].timestamp:
            self._sensor_buffer[veh_id][sensor_name] = StampedData(sensor.fetch(), current_time)

        # update sensor states
        self._sensor_states[veh_id][sensor_name] = (is_subscribed, max(retrieval_time, current_time))
        return self._sensor_buffer[veh_id][sensor_name]
    
    def garbage_collection(self, simulator, ctx):
        """Remove unnecessary sensor subscriptions.
        """
        current_time = traci.simulation.getTime()
        if current_time - self._last_gc_timestamp < self._gc_interval:
            # skip garbage collection if not enough interval time
            return

        logging.debug("Start state manager garbage collection.")

        # enumerate over all sensors
        for veh_id, states in self._sensor_states.items():
            for sensor_name, stats in states.items():
                is_subscribed, last_retrieval_time = stats

                # skip unsubscribed sensors
                if not is_subscribed:
                    continue

                # unsubscribe the sensor if it's not accessed for a long time
                if current_time - last_retrieval_time > self._gc_interval:
                    self._sensor_registration[veh_id][sensor_name].unsubscribe()
                    self._sensor_states[veh_id][sensor_name] = (False, 0)
                    if veh_id in self._sensor_buffer and sensor_name in self._sensor_buffer[veh_id]:
                        del self._sensor_buffer[veh_id][sensor_name]
                    logging.debug("The sensor {} from vehicle {} unsubscribed due to inactivity.".format(sensor_name, veh_id))

        self._last_gc_timestamp = current_time
