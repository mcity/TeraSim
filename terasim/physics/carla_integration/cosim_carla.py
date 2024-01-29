#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time

import redis
import json

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================

class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 carla_simulation,
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.carla = carla_simulation

        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

        # terasim cosim settings
        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = [2.0, 159.0, 34.5]
        # redis server for communication
        self.redis_server = redis.Redis(host='localhost', port=6379, db=0)

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.

    def tick(self):
        """
        Tick to simulation synchronization
        """
        
        self.sync_sumo_to_carla()
        self.sync_carla_to_sumo()

    def sync_carla_to_sumo(self):
        carla_actor_list = self.carla.world.get_actors()
        carla_vehicle_list = [actor for actor in carla_actor_list if 'vehicle' in actor.type_id]
        carla_vehicle_ids = [vehicle.id for vehicle in carla_vehicle_list]

        sumo2carla_id_values = self.sumo2carla_ids.values()

        diff_ids = [id for id in carla_vehicle_ids if id not in sumo2carla_id_values]
        
        cosim_thirdpartysim_vehicle_info = {}

        for vehicle_id in diff_ids:
            carla_vehicle = self.carla.get_actor(vehicle_id)
            sumo_transform = BridgeHelper.get_sumo_transform(carla_vehicle.get_transform(),
                                            carla_vehicle.bounding_box.extent)
            vehicle = {
                "location": {
                    'x': sumo_transform.location.x,
                    'y': sumo_transform.location.y,
                    'z': sumo_transform.location.z,
                },
                "rotation": {
                    'x': sumo_transform.rotation.roll,
                    'y': sumo_transform.rotation.pitch,
                    'z': sumo_transform.rotation.yaw,
                },
            }

            redis_key = "CARLA_" + str(vehicle_id)
            cosim_thirdpartysim_vehicle_info[redis_key] = vehicle

        self.redis_server.set('cosim_thirdpartysim_vehicle_info', json.dumps(cosim_thirdpartysim_vehicle_info))

    def sync_sumo_to_carla(self):
        # reads sumo context from redis.
        cosim_terasim_vehicle_info_json = self.redis_server.get('cosim_terasim_vehicle_info')
        if cosim_terasim_vehicle_info_json is not None:
            cosim_terasim_vehicle_info_dict = json.loads(cosim_terasim_vehicle_info_json)
        else:
            # Destroying synchronized actors.
            for carla_actor_id in self.sumo2carla_ids.values():
                self.carla.destroy_actor(carla_actor_id)
            print("No data found for cosim_terasim_vehicle_info, destroying all actors.")
            self.sumo2carla_ids = {}
            time.sleep(2)
            return

        # iterates over sumo actors and updates them in carla.
        for sumo_actor_id, sumo_actor_value in cosim_terasim_vehicle_info_dict.items():
            sumo_actor_type_id = sumo_actor_value['type_id']
            sumo_actor_vclass_value = sumo_actor_value['vclass']
            sumo_actor_color_list = sumo_actor_value['color']
            sumo_actor_color_tuple = tuple(sumo_actor_color_list)

            sumo_actor_transform = carla.Transform()
            sumo_actor_transform.location.x = sumo_actor_value['location']['x']
            sumo_actor_transform.location.y = sumo_actor_value['location']['y']
            sumo_actor_transform.location.z = sumo_actor_value['location']['z']
            sumo_actor_transform.rotation.pitch = sumo_actor_value['rotation']['x']
            sumo_actor_transform.rotation.yaw = sumo_actor_value['rotation']['y']
            sumo_actor_transform.rotation.roll = sumo_actor_value['rotation']['z']

            sumo_actor_extent = carla.Vector3D()
            sumo_actor_extent.x = sumo_actor_value['extent']['x']
            sumo_actor_extent.y = sumo_actor_value['extent']['y']
            sumo_actor_extent.z = sumo_actor_value['extent']['z']

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor_transform, sumo_actor_extent)
            # Creating new carla actor or updating existing one.
            if sumo_actor_id not in self.sumo2carla_ids:
                carla_blueprint = BridgeHelper.get_carla_blueprint_from_sumo_redis(
                    sumo_actor_id, sumo_actor_type_id, sumo_actor_color_tuple, sumo_actor_vclass_value)

                if carla_blueprint is not None:
                    carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                    if carla_actor_id != INVALID_ACTOR_ID:
                        self.sumo2carla_ids[sumo_actor_id] = carla_actor_id

                    print("Spawn actor: ", sumo_actor_id, carla_actor_id)
                        
            else:
                carla_actor_id = self.sumo2carla_ids[sumo_actor_id]
                self.carla.synchronize_vehicle(carla_actor_id, carla_transform, lights=None)

        # Iterate over sumo2carla_ids dictionary and destroy actors that are not in sumo_actor_ids.
        for sumo_actor_id in list(self.sumo2carla_ids.keys()):
            if sumo_actor_id not in cosim_terasim_vehicle_info_dict:
                print("Destroy actor: ", sumo_actor_id)
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        self.carla.tick()

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        # Closing carla client.
        self.carla.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(carla_simulation, args.sync_vehicle_color, args.sync_vehicle_lights)
    
    try:
        while True:
            start = time.time()

            synchronization.tick()

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning synchronization')
        synchronization.close()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--sumo_cfg_file',
                           default = 'map/sumo_map/mcity.sumocfg', 
                           type=str, 
                           help='sumo configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
 
    arguments = argparser.parse_args()
    
    synchronization_loop(arguments)
