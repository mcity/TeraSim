New Environment
================
The Environment class is used to define the simulation environment.
Before defining a new Environment class, **please import the original Environment class** so that the function to add vehicles and the functions related to the maintenance of the vehicle list can be implemented.

.. code:: python3

    from core.envs.env import Environment

A new Environment class should at least have the following functions:

* :func:`__init__`:
    This function can initialize an Environment object. It will define the default controller type for the individual autonomous vehicle, the default controller type for the list of autonomous vehicles, the default controller type for the list of background vehicles, whether to track CAV in SUMO simulation, and the information extractor for the simulation.

* :func:`_step`:
    This function defines the necessary actions in every time step except from maintaining the vehicle lists. It should at least contain the following functions:

    * :func:`self.global_av_controller.step`:
        This function computes the actions for all the autonomous vehicles. 
    * :func:`self.global_bv_controller.step`:
        This function computes the actions for all the background vehicles.
    * :func:`self.info_extractor.get_snapshot_info`:
        This function asks the information extractor to record the simulation information at every time step.

    Some additional functions can be added. For example,

    * The function to initialize the vehicle flow of the SUMO simulation. It is possible to use the function :func:`self.add_background_vehicles` to add vehicles.
    * The function to set the initial parameters of vehicles, such as the function to set the maximum lateral speed of the vehicle with :func:`self.simulator.set_vehicle_max_lateralspeed`. 
    * The function to subscribe vehicle information using :func:`self.simulator.subscribe_vehicle`. 

* :func:`terminate_check`:
    This function defines when to terminate a simulation.
