from pathlib import Path

from terasim.envs.template import EnvTemplate
from terasim.logger.data_recorder_info_extractor import DataRecorderInfoExtractor
from terasim.simulator import Simulator
from terasim.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from terasim.vehicle.decision_models.idm_model import IDMModel
from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.vehicle import Vehicle

current_path = Path(__file__).parent
# maps_path = current_path / "maps" / "3LaneHighway"
maps_path = current_path / "maps" / "Mcity"


class ExampleVehicleFactory(VehicleFactory):
    def create_vehicle(self, veh_id, simulator):
        """Generate a vehicle with the given vehicle id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            veh_id (_type_): vehicle id
            simulator (_type_): simulator (sumo)

        Returns:
            Vehicle: the contructed vehicle object
        """
        sensor_list = [EgoSensor(), LocalSensor(obs_range=40)]
        # decision_model = DummyDecisionModel(mode="random")  # mode="random" "constant"
        decision_model = IDMModel(MOBIL_lc_flag=False, stochastic_acc_flag=True)
        controller = HighEfficiencyController(simulator)
        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )

output_path = current_path / "output" / "0"

recording_config = {
    "output_path": str(output_path / "recordings"),
    "format": "json",
    "description": "Example simulation with data recording"
}

env = EnvTemplate(
    vehicle_factory=ExampleVehicleFactory(), 
    info_extractor=lambda env: DataRecorderInfoExtractor(env, recording_config)
)

sim = Simulator(
    sumo_net_file_path=maps_path / "map.net.xml",
    sumo_config_file_path=maps_path / "sim.sumocfg",
    num_tries=10,
    gui_flag=False,
    output_path=output_path,
    sumo_output_file_types=["collision"],
)

sim.bind_env(env)
print("Starting simulation with data recording...")
sim.run()
print("Simulation completed. Check the recordings directory for output files.")