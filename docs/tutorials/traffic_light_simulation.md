# Tutorial for creating agents and environment for the traffic light simulation
This tutorial will guide you through the process of creating agents and environment for the traffic light simulation. The traffic light simulation is a typical multi-agent system, which includes vehicles and traffic lights. In this tutorial, we will show you how to create the decision model, controller, and sensor for the agents, and how to define the environment for the simulation. The detailed code is shown [here](../../examples/example_traffic_light.py). Let's break the code down:

## 1. Code explanation

### 1.1. Import necessary packages and data
```python
from pathlib import Path
from terasim.simulator import Simulator
from terasim.logger.infoextractor import InfoExtractor

current_path = Path(__file__).parent
maps_path = current_path / 'maps' / 'Mcity'

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
from terasim.traffic_light.factories.traffic_light_factory import TrafficLightFactory
```

### 1.2. Define control logic of the agents, including vehicles and traffic lights
In the simulation, there are two kinds of agents: `Vehicle` and `TrafficLight`. For each `Agent`, we need to define the decision model, controller, and sensor. Here is an example of traffic light decision model.

#### 1.2.1. Example of traffic light decision model
Please note that any customized decision model should inherit the `AgentDecisionModel` class and implement the `derive_control_command_from_observation` function. The specific `derive_control_command_from_observation` fucntion is used to derive the control command from the observation. The observation is a dictionary containing the information of the agent. Here is an example of traffic light decision model, which creates a 120 seconds traffic control cycle with 8 phases. 

```python
# Example of traffic light decision model
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
```

### 1.2.2. Example of vehicle factory
Please note that any customized vehicle factory should inherit the `VehicleFactory` class and implement the `create_vehicle` function. The specific `create_vehicle` fucntion is used to insert vehicles, which are composed of a decision model, a controller, and a list of sensors. Here is an example of vehicle factory, which generates SUMO-controlled vehicles in the simulation. 
```python
# Example of vehicle factory
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
```

### 1.2.3. Example of traffic light factory
Please note that any customized traffic light factory should inherit the `TrafficLightFactory` class and implement the `create_traffic_light` function. The specific `create_traffic_light` fucntion is used to insert traffic lights, which are composed of a decision model, a controller, and a list of sensors. Here is an example of traffic light factory, which generates user-controlled traffic lights in the simulation. In the following example, the traffic light with id "NODE_17" is controlled by the `ExampleStateDecisionModel` shown before, and the other traffic lights are controlled by the `DummyStateDecisionModel` which uses the all green logic.
```python
# Example of traffic light factory
class ExampleTrafficLightFactory(TrafficLightFactory):

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
```

## 1.3. Create the simulated environment
In the simulation, the environment is defined by the `EnvTrafficLightTemplate` class. It is composed of `ExampleVehicleFactory`, `ExampleTrafficLightFactory`, and `InfoExtractor`. The first two components are defined before and `InfoExtractor` is used to log useful data.
```python
# Example of environment
env = EnvTrafficLightTemplate(
    vehicle_factory = ExampleVehicleFactory(),
    tls_factory = ExampleTrafficLightFactory(),
    info_extractor=InfoExtractor
)
```
## 1.4. Create the simulator and run the simulation
Then, you need to create the `Simulator`, and define the `sumo_net_file_path` and `sumo_config_file_path` (you can replace the path with your own path). You can decide whether it will display **sumo-gui** interface with `gui_flag`. Then, use `bind_env(env)` function to bind the environment to the simulation, and `run()` funtion to start the simulation.
```python
sim = Simulator(
    sumo_net_file_path = maps_path / 'Mcity.net.xml',
    sumo_config_file_path = maps_path / 'example_tls_Mcity.sumocfg',
    num_tries=10,
    gui_flag=True,
    output_path = current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
)

sim.bind_env(env)
sim.run()
```

## 2. Usage
```sh
python examples/example_traffic_light.py
```

## 3. Demo
![](../videos/demo_traffic_light_simulation.gif)
