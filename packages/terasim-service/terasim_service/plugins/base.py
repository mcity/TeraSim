from abc import ABC, abstractmethod

from terasim.simulator import Simulator


DEFAULT_PLUGIN_CONFIG = {
    "name": "base_plugin",
    "priority": {
        "start": -100,
        "step": -100,
        "stop": -100,
    }
}

DEFAULT_REDIS_CONFIG = {
    "host": "localhost",
    "port": 6379,
    "db": 0,
}

class BasePlugin(ABC):
    """Base class for plugin of TeraSim
    """
    def __init__(
        self,
        simulation_uuid: str,
        plugin_config: dict = DEFAULT_PLUGIN_CONFIG,
        redis_config: dict = DEFAULT_REDIS_CONFIG,
    ):
        """Initialize the base plugin.
        """
        # Unique identifier for this simulation instance
        self.simulation_uuid = simulation_uuid
        # Name of the plugin
        self.plugin_name = plugin_config.get("name", "base_plugin")
        # Priority of the plugin
        self.plugin_priority = plugin_config.get("priority", {
            "start": -100,
            "step": -100,
            "stop": -100,
        })
        # Plugin configuration
        self.plugin_config = plugin_config
        # Redis connection configuration
        self.redis_config = redis_config
        self.redis_client = None

    def inject(self, simulator: Simulator, ctx):
        """Inject the plugin into the simulation.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        self.ctx = ctx
        self.simulator = simulator

        simulator.start_pipeline.hook(f"{self.plugin_name}_start", self.on_start, priority=self.plugin_priority["start"])
        simulator.step_pipeline.hook(f"{self.plugin_name}_step", self.on_step, priority=self.plugin_priority["step"])
        simulator.stop_pipeline.hook(f"{self.plugin_name}_stop", self.on_stop, priority=self.plugin_priority["stop"])
    
    def on_start(self, simulator: Simulator, ctx):
        """Called when the simulation starts.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        pass

    def on_step(self, simulator: Simulator, ctx):
        """Called at each simulation step.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        pass

    def on_stop(self, simulator: Simulator, ctx):
        """Called when the simulation stops.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.
        """
        pass