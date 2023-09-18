'''
This module re-export sumo and carla for the simulation platform
'''

import os, sys, logging, pathlib

# ===== load sumo API, use libsumo by default =====
has_libsumo = False
if os.environ.get("USE_LIBSUMO", True) != '0':
    try:
        import libsumo as traci
        has_libsumo = True
    except ImportError:
        logging.warn("Failed to find libsumo, try to traci instead.")
if not has_libsumo:
    try:
        import traci
    except ImportError:
        logging.error("Unable to find traci!")
        raise

# ===== load carla API =====
has_carla = False

# add Carla distro path
if 'CARLA_ROOT' in os.environ:
    api_path = pathlib.Path(os.environ['CARLA_ROOT']) / 'PythonAPI'
    egg_name = 'carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'
    )
    for egg in api_path.glob('./carla/dist/' + egg_name):
        sys.path.append(egg)
        break

try:
    import carla
except ImportError:
    logging.warning("Carla PythonAPI is not correctly installed!")
