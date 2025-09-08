"""
This module re-export sumo and carla for the simulation platform
"""

import logging
import os


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


should_profile = os.getenv("ENABLE_PROFILING", "False").lower() in ("true", "1")
if should_profile:
    from line_profiler import LineProfiler

    def profile(func):
        def wrapper(*args, **kwargs):
            profiler = LineProfiler()
            profiler.add_function(func)
            profiler.enable_by_count()
            result = func(*args, **kwargs)
            profiler.print_stats()
            return result

        return wrapper
    
else:

    def profile(func):
        return func
