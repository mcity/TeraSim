import importlib
from loguru import logger
from omegaconf import OmegaConf
from pathlib import Path
from pydantic import BaseModel, Field
import redis
import sys
import yaml

from terasim.logger.infoextractor import InfoExtractor
from terasim.simulator import Simulator

from terasim_nde_nade.vehicle import NDEVehicleFactory
from terasim_nde_nade.vru import NDEVulnerableRoadUserFactory

from .messages import AgentCommand


class SimulationConfig(BaseModel):
    config_file: str = Field(
        ..., description="Path to the simulation configuration file"
    )
    auto_run: bool = Field(
        False,
        description="Whether to automatically run the simulation or wait for manual control",
    )


class SimulationStatus(BaseModel):
    id: str = Field(..., description="Unique identifier for the simulation")
    status: str = Field(..., description="Current status of the simulation")
    progress: float = Field(
        0.0, description="Progress of the simulation as a percentage"
    )


class SimulationCommand(BaseModel):
    command: str = Field(
        ...,
        description="Control command for the simulation (e.g., 'pause', 'resume', 'stop')",
    )


class AgentCommandBatch(BaseModel):
    commands: list[AgentCommand] = Field(
        ..., description="List of agent commands to execute"
    )


def load_config(config_file):
    """Load the configuration file.

    Args:
        config_file (str): Path to the configuration file.

    Returns:
        dict: The configuration dictionary.
    """
    with open(config_file, "r") as file:
        return yaml.safe_load(file)


def create_environment(config, base_dir):
    """Create the environment based on the configuration.

    Args:
        config (dict): The configuration dictionary.
        base_dir (str): Base directory for the environment.

    Returns:
        Environment: The environment object.
    """
    env_module = importlib.import_module(config["environment"]["module"])
    env_class = getattr(env_module, config["environment"]["class"])

    env_params = OmegaConf.create(config["environment"]["parameters"])

    return env_class(
        av_cfg = env_params.AV_cfg,
        vehicle_factory=NDEVehicleFactory(env_params),
        vru_factory=NDEVulnerableRoadUserFactory(env_params),
        info_extractor=InfoExtractor,
        log_flag=config["environment"]["parameters"]["log_flag"],
        log_dir=base_dir,
        warmup_time_lb=config["environment"]["parameters"]["warmup_time_lb"],
        warmup_time_ub=config["environment"]["parameters"]["warmup_time_ub"],
        run_time=config["environment"]["parameters"]["run_time"],
        configuration=env_params,
    )


def create_simulator(config, base_dir):
    """Create the simulator based on the configuration.

    Args:
        config (dict): The configuration dictionary.
        base_dir (str): Base directory for the simulator.

    Returns:
        Simulator: The simulator object.
    """
    return Simulator(
        sumo_net_file_path=Path(config["input"]["sumo_net_file"]),
        sumo_config_file_path=Path(config["input"]["sumo_config_file"]),
        num_tries=config["simulator"]["parameters"]["num_tries"],
        gui_flag=config["simulator"]["parameters"]["gui_flag"],
        # gui_flag=True,
        realtime_flag=config["simulator"]["parameters"].get("realtime_flag", False),
        output_path=base_dir,
        sumo_output_file_types=config["simulator"]["parameters"][
            "sumo_output_file_types"
        ],
        seed=config["simulator"]["parameters"].get("sumo_seed", None),
        additional_sumo_args=["--start", "--quit-on-end"],
    )

def set_random_seed(seed):
    """Set the random seed for the simulation.
    """
    import random
    import numpy as np
    random.seed(seed)
    np.random.seed(seed)
    try:
        import torch
        torch.manual_seed(seed)
    except ImportError:
        pass
    logger.info(f"Setting random seed to {seed}")

# Add this function to check Redis connection
def check_redis_connection():
    """Check the connection to Redis.
    """
    try:
        redis_client = redis.Redis(host="localhost", port=6379, db=0)
        redis_client.ping()
        logger.info("Successfully connected to Redis")
    except redis.ConnectionError:
        logger.error("Failed to connect to Redis. Exiting...")
        sys.exit(1)