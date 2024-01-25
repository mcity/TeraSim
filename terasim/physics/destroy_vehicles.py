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

# Connect to the server
client = carla.Client('localhost', 2000)

# Set the timeout
client.set_timeout(10.0) 

# Get the world from the server
world = client.get_world()

# Get all the actors in the world
actors = world.get_actors()

# Here we consider vehicle as object. So, we get all the vehicles in the simulation.
vehicles = actors.filter('*vehicle*')

# Destroy all vehicles
for vehicle in vehicles:
    print('Destroying vehicle %s' % vehicle.id)
    vehicle.destroy()