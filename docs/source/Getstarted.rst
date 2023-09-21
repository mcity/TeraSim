Get Started
==============

Here is an example usage of MTL Simulation Platform.

.. code:: python3

    from terasim.simulator import Simulator
    from terasim.envs.env import BaseEnv
    from terasim.controller.vehicle_controller.idmcontroller import IDMController
    from terasim.controller.vehicle_controller.globalcontroller import DummyGlobalController
    from terasim.logger.infoextractor import InfoExtractor

    env = BaseEnv(
        global_controller_dict={"BV":DummyGlobalController, "CAV":DummyGlobalController},
        independent_controller_dict={"BV":IDMController, "CAV":IDMController},
        info_extractor=InfoExtractor
    )
    sim = Simulator(
        sumo_net_file_path = './maps/3LaneHighway/3LaneHighway.net.xml',
        sumo_config_file_path = './maps/3LaneHighway/3LaneHighway.sumocfg',
        num_tries=50,
        step_size=0.1,
        action_step_size=1,
        sublane_flag=True,
        gui_flag=True,
        output=["fcd_all"],
        experiment_path="./demo_path"
    )
    sim.bind_env(env)
    sim.run(0)

The essential step to implement MTL Simulation Platform is shown as follows.

* Import modules from MTL Simulation Platform;
    .. code:: python3

        from terasim.simulator import Simulator
        from terasim.envs.env import BaseEnv
        from terasim.controller.vehicle_controller.idmcontroller import IDMController
        from terasim.controller.vehicle_controller.globalcontroller import DummyGlobalController
        from terasim.logger.infoextractor import InfoExtractor

* Initialize the Environment;
    .. code:: python3

        env = BaseEnv(
            global_controller_dict={"BV":DummyGlobalController, "CAV":DummyGlobalController},
            independent_controller_dict={"BV":IDMController, "CAV":IDMController},
            info_extractor=InfoExtractor
        )

    This step sets the simulation environment, including global controllers and independent controllers for different vehicle types, and the information extractor aimed to record the experiment data. In the example experiment, there are two types of vehicles, i.e. background vehicles (BVs), and connected and automated vehicles (CAVs). Two DummyGlobalController objects are defined to control BVs and CAVs, respectively. All the vehicles make decisions using the IDMController objects, which implement the Intelligent Driving model for car-following behaviors and the MOBIL model for lane-change behaviors. The InfoExtractor object is constructed to record nothing.

* Initialize the Simulator;
    .. code:: python3

        sim = Simulator(
            sumo_net_file_path = './maps/3LaneHighway/3LaneHighway.net.xml',
            sumo_config_file_path = './maps/3LaneHighway/3LaneHighway.sumocfg',
            num_tries=50,
            step_size=0.1,
            action_step_size=1,
            sublane_flag=True,
            gui_flag=True,
            output=["fcd_all"],
            experiment_path="./demo_path"
        )
    
    This step set the simulation parameters:

    * **SUMO network file,**
    * SUMO simulation configuration file,
        The configuration file usually contains the path of the route file, the path of the network file and some basic simulation setting, such as the ending time.
    * **Simulation step size in seconds,**
    * **Vehicle action step size in seconds,**
    * Whether to implement the SUMO Sublane model,
        Basically, the SUMO Sublane Model can help make the vehicle's lateral dynamics become more realistic. For more reference, please check https://sumo.dlr.de/docs/Simulation/SublaneModel.html.
    * **Whether to open the SUMO graphical user interface,**
    * Output format,
        Currently, there are five output formats supported:

        * "fcd": This type of output file contains the location, speed, and acceleration along with other information for every vehicle **surrounding the "CAV"** at every time step.
        * "fcd_all": This type of output file contains the location, speed, and acceleration along with other information for **every vehicle in the network** at every time step.
        * "traj": This type of output file contains the trajectories of each vehicle including type, current speed and acceleration.
        * "lc": This type of output file contains all the lane change behaviors including lane change reason, behavior time and so on. 
        * "collision": This type of output file contains the collision information, including the collision vehicle ID, collision time, and so on. 

    * Experimental output path.


* Combine the Simulator with the Environment;
    .. code:: python3

        sim.bind_env(env)

* Run experiment.
    .. code:: python3

        sim.run(0)

    The input integer is used to define the experiment index in case that a large number experiments are conducted simultaneously.