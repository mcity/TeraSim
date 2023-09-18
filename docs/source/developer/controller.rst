New Controller
===============
Based on the controlled object, there are two types of Controller: Vehicle Controller and Signal Controller.

Vehicle Controller
-------------------
Based on the controlled number of objects, there are two types of Vehicle Controller: Global Controller and Individual Controller.

Global Controller
~~~~~~~~~~~~~~~~~~~~~
The Global Controller class is used to control a list of vehicles.
Before defining a new Global Controller class, **please import the original Global Controller class**.

.. code:: python3

    from core.controller.vehicle_controller.globalcontroller import GlobalController

A new Global Controller class should at least have the following functions:

* :func:`__init__`:
    This function can initialize a Global Controller object. It will bind the Environment class, set the default controller type for individual vehicles.

* :func:`step`:
    This function defines the necessary actions for the vehicle list in every time step, including resetting vehicle control state, and controlling individual vehicles with :func:`vehicle.controller.step`.

* :func:`apply_control_permission`:
    This function defines when to compute the control commands.

Individual Controller
~~~~~~~~~~~~~~~~~~~~~~~~~~
The Individual Controller class is used to control one vehicle. Before defining a new Individual Controller class, **please import the original Controller class**, so that the basic information of a controller is set.

.. code:: python3

    from core.controller.vehicle_controller.controller import Controller

A new Controller class should at least have the following functions:

* :func:`__init__`:
    This function can initialize a Controller object. It will define the controller type of the individual vehicle.

* :func:`step`:
    This function defines the necessary actions for the vehicle in every time step.

Signal Controller
------------------