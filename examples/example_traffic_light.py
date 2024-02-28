import sys 
from pathlib import Path
sys.path.append('/home/chris/github/TeraSim/')

from terasim.simulator import Simulator
from terasim.envs.template import EnvTemplate
from terasim.logger.infoextractor import InfoExtractor
from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from terasim.traffic_light.traffic_light_sensor import BaseTrafficLightSensor
from terasim.traffic_light.traffic_light_decision_model import TrafficLightDecisionModel
from terasim.traffic_light.traffic_light_controller import TrafficLightController
from terasim.traffic_light.traffic_light_factory import TrafficLightFactory
from terasim.traffic_light.traffic_light import TrafficLight
from terasim.vehicle.vehicle import Vehicle
from terasim.vehicle.decision_models.idm_model import IDMModel

current_path = Path(__file__).parent
maps_path = current_path / 'maps' / 'mcity'

class ExampleVehicleFactory(VehicleFactory):

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

class ExampleTrafficFactory(TrafficLightFactory):

    def create_traffic_light(self, tls_id, simulator):
        """Generate a vehicle with the given vehicle id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            veh_id (_type_): vehicle id
            simulator (_type_): simulator (sumo)

        Returns:
            Vehicle: the contructed vehicle object
        """
        sensor_list = [BaseTrafficLightSensor()]
        # decision_model = DummyDecisionModel(mode="random")  # mode="random" "constant"
        decision_model = TrafficLightDecisionModel()
        control_params = {
            "v_high": 40,
            "v_low": 20,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
        }
        controller = TrafficLightController(simulator, control_params)
        return TrafficLight(tls_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)

env = EnvTemplate(
    tls_factory = ExampleTrafficFactory(),
    info_extractor=InfoExtractor
)

sim = Simulator(
    sumo_net_file_path = maps_path / 'mcity.net.xml',
    sumo_config_file_path = maps_path / 'mcity_medium_traffic.sumocfg',
    num_tries=10,
    gui_flag=True,
    output_path = current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
)
sim.bind_env(env)
sim.run()
