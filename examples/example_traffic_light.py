
from pathlib import Path
from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor

current_path = Path(__file__).parent
maps_path = current_path / 'maps' / 'mcity'

from terasim.simulator import traci
from terasim.envs.template_traffic_light import EnvTrafficLightTemplate

from terasim.agent.agent_decision_model import AgentDecisionModel
from terasim.agent.agent_controller import AgentController

from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.vehicle import Vehicle
from terasim.vehicle.decision_models.dummy_setsumo_transform_decision_model import DummySetSUMOTranformDecisionModel

from terasim.traffic_light.traffic_light import TrafficLight
from terasim.traffic_light.controllers.state_controller import StateController
from terasim.traffic_light.decision_models.dummy_state_decision_model import DummyStateDecisionModel
from terasim.traffic_light.sensors.ego_state_sensor import EgoStateSensor
from terasim.traffic_light.factories.base_traffic_light_factory import BaseTrafficLightFactory

# ExampleTrafficDecision
class ExampleStateDecisionModel(AgentDecisionModel):

    def derive_control_command_from_observation(self, obs_dict):
        return self.get_decision(), None

    def get_decision(self):
        # set the traffic light with 120s cycle and 18s green, 3s yellow, 32s green, 3s yellow, 30s green, 3s yellow, 30s green, 3s yellow
        sim_step = traci.simulation.getCurrentTime()
        cicle_time = sim_step/1000 % 120
        if cicle_time <= 18:
            return "rGrrrGrrr"
        elif cicle_time <= 21:
            return "ryrrryrrr"
        elif cicle_time <= 53:
            return "GrrrGrrrr"
        elif cicle_time <= 56:
            return "yrrryrrrr"
        elif cicle_time <= 86:
            return "rrrGrrrrG"
        elif cicle_time <= 89:
            return "rrryrrrry"
        elif cicle_time <= 117:
            return "rrGrrrGGr"
        else:
            return "rryrrryyr"
        
class ExampleVehicleFactory(VehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        """Generate a vehicle with the given vehicle id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            veh_id (_type_): vehicle id
            simulator (_type_): simulator (sumo)

        Returns:
            Vehicle: the contructed vehicle object
        """
        sensor_list = [EgoSensor()]
        # decision_model = DummyDecisionModel(mode="random")  # mode="random" "constant"
        decision_model = DummySetSUMOTranformDecisionModel()
        controller = AgentController(simulator)
        return Vehicle(veh_id, simulator, sensors=sensor_list,
                       decision_model=decision_model, controller=controller)

class ExampleTrafficFactory(BaseTrafficLightFactory):

    def create_traffic_light(self, tls_id, simulator):
        
        """Generate a traffic light with the given tls id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            tls_id (_type_): traffic light id
            simulator (_type_): simulator (sumo)

        Returns:
            TrafficLight: the contructed traffic light object
        """
        if tls_id == "NODE_17":
            sensor_list = [EgoStateSensor()]
            decision_model = ExampleStateDecisionModel()
            controller = StateController(simulator)
            return TrafficLight(tls_id, simulator, sensors=sensor_list,
                        decision_model=decision_model, controller=controller)
        else:
            sensor_list = [EgoStateSensor()]
            decision_model = DummyStateDecisionModel()
            controller = StateController(simulator)
            return TrafficLight(tls_id, simulator, sensors=sensor_list,
                        decision_model=decision_model, controller=controller)
        

env = EnvTrafficLightTemplate(
    vehicle_factory = ExampleVehicleFactory(),
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
