from pathlib import Path
import random
import numpy as np
from PIL import Image

from terasim.simulator import Simulator
from terasim.vehicle import Vehicle
from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.logger.infoextractor import InfoExtractor
from terasim.physics import CarlaPhysics
from terasim.physics.carla.sensors import Camera
from terasim.envs import EnvTemplate
from terasim.vehicle.decision_models.dummy_setsumo_transform_decision_model import DummySetSUMOTranformDecisionModel
from terasim.vehicle.controllers.sumo_move_controller import SUMOMOVEController

current_path = Path(__file__).parent
maps_path = current_path / 'maps' / 'CarlaTown04'

class CarlaTestingFactory(VehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        sensor_list = [Camera()] if int(veh_id) < 5 else []
        decision_model = DummySetSUMOTranformDecisionModel()
        controller = SUMOMOVEController(simulator)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)

class CarlaTestingEnv(EnvTemplate):
    def __init__(self, vehicle_factory, info_extractor):
        super().__init__(vehicle_factory, info_extractor)
        self._counter = 0
        self._exists = []

    def on_start(self, ctx):
        # generate a random vehicle at the beginning of the simulation
        route = "r_{}".format(random.randint(1, 4))
        # TODO: self.ego = self.add_vehicle(veh_id="hero", route=route)

        return super().on_start(ctx)

    def on_step(self, ctx):
        if random.random() < 0.05:
            # occasionally generate a vehicle at each simulation step
            veh_id = "added_%d" % self._counter
            self._counter += 1

            route = "r_{}".format(random.randint(1, 4))
            # TODO: self.add_vehicle(veh_id=veh_id, route=route)
            self._exists.append(veh_id)
            print("Generated", veh_id)

        if random.random() > 0.95 and self._exists:
            # occasionally remove a vehicle at each simulation step
            veh_id = random.choice(self._exists)
            # TODO: self.remove_vehicle(veh_id)
            self._exists.remove(veh_id)
            print("Eliminated", veh_id)

        # check sensor
        VIS_AGENT_ID = '0'
        if VIS_AGENT_ID in self.vehicle_list:
            data = self.vehicle_list[VIS_AGENT_ID].sensors['camera'].observation
            if data is not None:
                data_arr = np.ndarray(shape=(data.height, data.width, 4), dtype=np.uint8, buffer=data.raw_data)
                img = Image.fromarray(data_arr[:, :, 2::-1]) # BGRA to RGB
                img.save(current_path / "output" / '0' / 'save.jpg')
                print("Received image.")

        return super().on_step(ctx)

sim = Simulator(
    sumo_net_file_path = maps_path / 'Town04.net.xml',
    sumo_config_file_path = maps_path / 'Town04.sumocfg',
    num_tries=10,
    gui_flag=True,
    output_path = current_path / "output" / '0',
    sumo_output_file_types=["fcd_all"],
)
sim.bind_env(CarlaTestingEnv(CarlaTestingFactory(), InfoExtractor))
sim.add_plugin(CarlaPhysics(host="192.168.68.105", map="Town04"))
sim.run()
