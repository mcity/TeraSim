"""
This module defines the interfaces to physics simulator, which provide
high fidelity physics and sensor simulation.
"""

# from .base import PhysicSimulator
from .terasim_cosim_plugin import TeraSimCoSimPlugin


class PhysicsPlugin:
    def __init__(self, engine, step_length, **connect_args) -> None:
        pass

    def inject(self, simulator) -> None:
        pass

    def sync_actors(self) -> None:
        pass
