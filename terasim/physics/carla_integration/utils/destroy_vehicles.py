# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

try:
    sys.path.append(
        glob.glob('../carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse

# Set up command-line argument parsing
parser = argparse.ArgumentParser(description="Destroy vehicles in CARLA simulation.")
parser.add_argument("--id", type=int, help="Vehicle ID to destroy")

# Parse command-line arguments
args = parser.parse_args()

# Connect to the server
client = carla.Client('localhost', 2000)

# Set the timeout
client.set_timeout(10.0) 

# Get the world from the server
world = client.get_world()

# ID of vehicle to be destroyed
vehicle_id_to_destroy = args.id

# Get all the vehicles in the simulation
vehicles = world.get_actors().filter('*vehicle*')

if vehicle_id_to_destroy:
    # Destroy specific vehicle with the provided ID
    for vehicle in vehicles:
        if vehicle.id == vehicle_id_to_destroy:
            print(f'Destroying vehicle {vehicle.id}')
            vehicle.destroy()
else:
    # Destroy all vehicles
    for vehicle in vehicles:
        print(f'Destroying vehicle {vehicle.id}')
        vehicle.destroy()