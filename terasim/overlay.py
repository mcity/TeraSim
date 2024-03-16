"""
This module re-export sumo and carla for the simulation platform
"""

import os, sys, logging, pathlib

# ===== load sumo API, use libsumo by default =====
has_libsumo = False
if os.environ.get("USE_LIBSUMO", True) != "0":
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
