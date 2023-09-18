from pathlib import Path
from mtlsp.simulator import Simulator
from mtlsp.envs.template import EnvTemplate
from mtlsp.logger.infoextractor import InfoExtractor
from mtlsp.vehicle.factories.dummy_vehicle_factory import DummyVehicleFactory
from mtlsp.vehicle.sensors.ego import EgoSensor
from mtlsp.vehicle.sensors.local import LocalSensor
from mtlsp.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from mtlsp.vehicle.vehicle import Vehicle
from mtlsp.vehicle.decision_models.idm_model import IDMModel

current_path = Path(__file__).parent
maps_path = current_path / 'maps' / '3LaneHighway'

class ExampleVehicleFactory(DummyVehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        """Generate a vehicle with the given vehicle id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            veh_id (_type_): vehicle id
            simulator (_type_): simulator (sumo)

        Returns:
            Vehicle: the contructed vehicle object
        """
        sensor_list = [EgoSensor(), LocalSensor()]
        # decision_model = DummyDecisionModel(mode="random")  # mode="random" "constant"
        decision_model = IDMModel(MOBIL_lc_flag=False, stochastic_acc_flag=True)
        control_params = {
            "v_high": 40,
            "v_low": 20,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
        }
        controller = HighEfficiencyController(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)

env = EnvTemplate(
    vehicle_factory = ExampleVehicleFactory(),
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
