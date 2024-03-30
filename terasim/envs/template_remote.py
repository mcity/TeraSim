from terasim.envs.template import EnvTemplate
from loguru import logger
from terasim_redis_connector.redis_client import redis_client
import terasim.utils as utils
from terasim.overlay import traci
import json


class EnvRemoteTemplate(EnvTemplate):
    """This Env provides a basic Env implementation.

    Env developers can derived from this class or build their own implementations directly on BaseEnv
    """

    def __init__(self, vehicle_factory, info_extractor):
        super().__init__(vehicle_factory, info_extractor)
        self.redis_client = redis_client

    def on_start(self, ctx):
        # register use algorithm to the simulator
        self.user_get_func = get_user_function_from_redis(
            self.redis_client, redis_key="get_function"
        )
        self.user_set_func = get_user_function_from_redis(
            self.redis_client, redis_key="set_function"
        )
        self.user_should_continue_simulation = get_user_function_from_redis(
            self.redis_client, redis_key="should_continue_simulation_function"
        )
        self.should_continue_simulation = (
            self.user_should_continue_simulation
            if self.user_should_continue_simulation
            else self.should_continue_simulation
        )

    def on_step(self, ctx):
        input = self.user_get_func(traci)
        input = {
            "timestamp": utils.get_time(),
            "info": input,
        }
        self.redis_client.set("input", json.dumps(input))
        while True:
            binary_output = self.redis_client.get("output")
            if not binary_output:
                continue
            output = json.loads(binary_output)
            if output["timestamp"] == input["timestamp"]:
                break
        self.user_set_func(traci, output["info"])
        # Simulation stop check
        return self.should_continue_simulation()


def deserialize_function(func_data):
    try:
        code = func_data.get("code", "").strip()
        if not code:
            print("No code provided.")
            return None
        exec(code, globals())
        func_name = code.replace("def ", "").split("(")[0].strip()
        return eval(func_name)
    except Exception as e:
        print(f"Error deserializing function: {e}")
        return None


def get_user_function_from_redis(redis_client, redis_key="function_channel"):
    # Register user function
    binary_func_data = redis_client.get(redis_key)
    if not binary_func_data:
        return None
    func_data = json.loads(binary_func_data)
    if func_data:
        user_func = deserialize_function(func_data)
        return user_func
    return None
