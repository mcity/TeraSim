from pathlib import Path
from mtlsp.simulator import Simulator
from mtlsp.envs import EnvTemplate
from mtlsp.logger.infoextractor import InfoExtractor
from mtlsp.vehicle.factories.dummy_vehicle_factory import DummyVehicleFactory
from mtlsp.vehicle.vehicle import Vehicle
from mtlsp.vehicle.decision_models.dummy_setsumo_transform_decision_model import DummySetSUMOTranformDecisionModel
from mtlsp.vehicle.controllers.sumo_move_controller import SUMOMOVEController

class ExampleVehicleFactory(DummyVehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        """Generate a vehicle with the given vehicle id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            veh_id (_type_): vehicle id
            simulator (_type_): simulator (sumo)

        Returns:
            Vehicle: the contructed vehicle object
        """
        sensor_list = []
        decision_model = DummySetSUMOTranformDecisionModel()
        controller = SUMOMOVEController(simulator)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)


current_path = Path(__file__).parent
maps_path = current_path / 'maps' / '3LaneHighway'

env = EnvTemplate(
    vehicle_factory=ExampleVehicleFactory(),
    info_extractor=InfoExtractor
)
sim = Simulator(
    sumo_net_file_path = maps_path / '3LaneHighway.net.xml',
    sumo_config_file_path = maps_path / '3LaneHighway.sumocfg',
    num_tries=10,
    gui_flag=False,
    output_path = current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
)
sim.bind_env(env)
sim.run()
