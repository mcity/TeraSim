from pathlib import Path
from terasim.simulator import Simulator
from terasim.envs.template import EnvTemplate
from terasim.logger.infoextractor import InfoExtractor
from terasim.vehicle.factories.dummy_vehicle_factory import DummyVehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from terasim.vehicle.vehicle import Vehicle
from terasim.vehicle.decision_models.idm_model import IDMModel

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
        v = Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)
        def v_on_mounted(v, ctx):
            if v.id == 'BV_flow3.0':

                traci = ctx['traci']
                traci.vehicle.setColor(v.id, (255, 0, 0, 255))
                traci.vehicle.setSpeed(v.id, 20)
                
                print('ego mounted')
            return
        def v_on_unmounted(v, ctx):
            if v.id == 'BV_flow3.0':
                print('ego unmountedd')
            return
        
        def v_on_before_step(v, ctx):
            if v.id == 'BV_flow3.0':
                print('ego before step')
            return
    
        def v_on_stepped(v, ctx):
            if v.id == 'BV_flow3.0':
                print('ego stepped')
            return
        
        v.on_mounted(v_on_mounted)
        v.on_unmounted(v_on_unmounted)
        v.on_before_step(v_on_before_step)
        v.on_stepped(v_on_stepped)
        return v

env = EnvTemplate(
    vehicle_factory = ExampleVehicleFactory(),
    info_extractor=InfoExtractor
)
sim = Simulator(
    sumo_net_file_path = maps_path / '3LaneHighway.net.xml',
    sumo_config_file_path = maps_path / '3LaneHighway.sumocfg',
    num_tries=10,
    gui_flag=True,
    output_path = current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
)
sim.bind_env(env)
sim.run()
