import logging
import bidict
from collections import defaultdict

from terasim.overlay import carla
from terasim.physics.carla.helper import BridgeHelper
from terasim.physics.carla.sensors import CarlaSensor
from terasim.physics.base import PhysicSimulator, SensorData
from terasim.simulator import Simulator, traci
from terasim.agent import Agent, AgentId, AgentInitialInfo

class SensorDataBuffer:
    def __init__(self) -> None:
        self._buffer = {}

    def put(self):
        pass

class CarlaPhysics(PhysicSimulator):
    """
    This class provides the interface to the Carla simulator.
    """
    SPAWN_OFFSET_Z = 25.0 # meters
    SYNC_VEHICLE_COLOR = True

    def __init__(self, host="localhost", port=2000, timeout=10, map="Town04", sync_on_demand=True) -> None:
        self._client = carla.Client(host, port)
        self._client.set_timeout(timeout)
        self._world = None # Carla world
        self._map = map

        self._agent_idmap = {} # agent_id -> agent carla_id
        self._sensor_idmap = {} # agent_id -> { sensor name -> carla_id }
        self._active_agents = set() # agents that have active subscriptions
        self._sync_on_demand = sync_on_demand

    def initialize(self, simulator, ctx) -> bool:
        try:
            if self._map:
                # load the specific map if the map argument is provided
                logging.info("Loading Carla map {}".format(self._map))
                self._world = self._client.load_world(self._map)
            else:
                # use the current map if the name is not specified
                self._world = self._client.get_world()
        except:
            logging.error("Failed to connect to the Carla simulator!")
            raise

        BridgeHelper.blueprint_library = self._world.get_blueprint_library()
        BridgeHelper.offset = simulator.sumo_net.getLocationOffset()

        # set to synchronized mode
        settings = self._world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = simulator.step_length
        self._world.apply_settings(settings)
        self._client.get_trafficmanager().set_synchronous_mode(True)

        return True
    
    def dispose(self, simulator, ctx) -> bool:
        if self._world is not None:
            # destroy all actors
            for agent_id in list(self._agent_idmap.keys()):
                self._destroy_agent_in_carla(agent_id)
            self._agent_idmap.clear()
            self._active_agents.clear()
            assert len(self._sensor_idmap) == 0

            # reset to the original setting
            settings = self._world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self._world.apply_settings(settings)

            # mark the world has been disposed
            self._world = None

    def __del__(self):
        if self._world is not None:
            self.dispose(None, None)

    def _spawn_agent_in_carla(self, agent_id, blueprint, transform):
        # spawn agent in the mid air
        transform = carla.Transform(transform.location + carla.Location(0, 0, self.SPAWN_OFFSET_Z),
                                    transform.rotation)

        carla_actor = self._world.spawn_actor(blueprint, transform)
        carla_actor.set_simulate_physics(False)

        self._agent_idmap[agent_id] = carla_actor.id
        return carla_actor

    def _destroy_agent_in_carla(self, agent_id: AgentId):        
        # destroying the sensors should be handled separately
        # TODO: assert agent_id not in self._sensor_idmap

        self._world.get_actor(self._agent_idmap.pop(agent_id)).destroy()

    def _spawn_sensor_in_carla(self, sensor: CarlaSensor):
        blueprint = BridgeHelper.blueprint_library.find(sensor.blueprint)
        for name, value in sensor.params.items():
            if blueprint.has_attribute(name):
                blueprint.set_attribute(name, str(value))
            else:
                logging.warn("Attribute {} invalid for sensor {}".format(name, repr(sensor)))

        agent_id = sensor._agent.id
        parent_actor = self._world.get_actor(self._agent_idmap[agent_id])
        carla_actor = self._world.spawn_actor(blueprint, sensor.transform, attach_to=parent_actor)

        # register id
        if agent_id not in self._sensor_idmap:
            self._sensor_idmap[agent_id] = {}
        self._sensor_idmap[agent_id][sensor.name] = carla_actor.id
        self._update_agent_context_subscription(agent_id)

        return carla_actor
    
    def _destroy_sensor_in_carla(self, sensor: CarlaSensor):
        agent_id = sensor._agent.id
        carla_id = self._sensor_idmap[agent_id].pop(sensor.name)
        actor = self._world.get_actor(self._agent_idmap.pop(carla_id))
        actor.destroy()

        # clear the sensor map for an agent if necessary
        if not self._sensor_idmap[agent_id]:
            del self._sensor_idmap[agent_id]
        self._update_agent_context_subscription(agent_id)

    def _update_agent_context_subscription(self, agent_id: AgentId):
        # determine if the agent still has carla sensors
        has_carla_sensor = False
        for sensor in self._simulator.env.vehicle_list[agent_id].sensors.values():
            if isinstance(sensor, CarlaSensor):
                has_carla_sensor = True

        if has_carla_sensor and agent_id not in self._active_agents:
            # TODO: subscribe this vehicle
            return
        if not has_carla_sensor and agent_id in self._active_agents:
            # TODO: unsubscribe this vehicle
            return

    def add_agent(self, agent: Agent, init_info: AgentInitialInfo) -> bool:
        # this method should be called after the agent exists in SUMO
        sumo_actor = BridgeHelper.get_sumo_actor(agent.id)
        carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.SYNC_VEHICLE_COLOR)
        carla_transform = BridgeHelper.get_carla_transform(
            sumo_actor.transform, sumo_actor.extent)
        assert carla_blueprint is not None
        return self._spawn_agent_in_carla(agent.id, carla_blueprint, carla_transform)

    def remove_agent(self, agent: Agent) -> bool:
        return self._destroy_agent_in_carla(agent.id)

    def step(self, simulator: Simulator, ctx) -> None:
        # Get actors to update
        sumo_actors = {}
        if self._sync_on_demand:
            # Create actors based on subscription from active ids
            for ego_id in self._active_agents:
                subs = traci.vehicle.getContextSubscriptionResults(ego_id)
                for agent_id, sub in subs.items():
                    if agent_id in sumo_actors:
                        continue

                    sumo_actors[agent_id] = BridgeHelper.get_sumo_actor_from_subscription(sub)
        else:
            # Create actors to for each agent in sumo
            for agent_id in traci.vehicle.getIDList():
                sumo_actors[agent_id] = BridgeHelper.get_sumo_actor(agent_id)

        # TODO: does this synchronization step affect the order of steps?
        #       specifically, when new vehicle is generated, do we need the vehicle object be in the list?

        # Then synchronize
        target_set, current_set = set(sumo_actors.keys()), set(self._agent_idmap.keys())
        born_actors = target_set.difference(current_set)
        dead_actors = current_set.difference(target_set)
        update_actors = current_set.intersection(target_set)

        # Spawning new sumo agents in carla (i.e, not controlled by carla).
        for agent_id in born_actors:
            sumo_actor = sumo_actors[agent_id]
            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.SYNC_VEHICLE_COLOR)
            carla_transform = BridgeHelper.get_carla_transform(
                sumo_actor.transform, sumo_actor.extent)
            assert carla_blueprint is not None

            self._spawn_agent_in_carla(agent_id, carla_blueprint, carla_transform)

        # Destroying arrived sumo actors in carla.
        for agent_id in dead_actors:
            self._destroy_agent_in_carla(agent_id)

        # Updating all sumo actors in carla.
        for agent_id in update_actors:
            sumo_actor = sumo_actors[agent_id]
            carla_actor = self._world.get_actor(self._agent_idmap[agent_id])

            carla_transform = BridgeHelper.get_carla_transform(
                sumo_actor.transform, sumo_actor.extent)
            carla_actor.set_transform(carla_transform)

        # Carla tick
        self._world.tick()
