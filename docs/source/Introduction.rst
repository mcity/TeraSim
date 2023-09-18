Introduction
==================

.. image:: ../source/_static/arch.svg
    :width: 100%
    :align: center
    :alt: algorithm-flowchart

Simulation of Urban Mobility (SUMO)
-------------------------------------
SUMO is a well-known open-source traffic simulation platform. Commonly, the input contains four parts. The network file contains all the information about the traffic network, including the settings of the traffic lights, and it should follow SUMO’s standard. The demand file is composed of the vehicle route, vehicle volume and so on, The configuration file tells the system where to find the network file and the demand file. In addition, the simulation setting such as step size, simulation duration and output settings are provided in this sumo config file. Sometimes, we need to add other element to the simulation, such as the loop detectors, so, the additional files are necessary to make the platform become more comprehensive. SUMO’s main function in our platform is to provide the microscopic simulation. It will consider all the traffic facilities and traffic participants. Some of the facilities SUMO will consider is the network and the traffic light.  As for traffic lights, SUMO provide two default modes, one is the static signal program and the other is the actuated mode which should be implemented with detectors. As for the traffic participants, SUMO will simulate pedestrians and vehicles as well. There are several types of vehicle model in SUMO, such as the IDM and ACC model for car following and the LC2013, SL2015 and DK2008 for lane change.

Map Transformer
~~~~~~~~~~~~~~~~~~~
This tool :func:`tools.map_import.transform_map` is aimed to transform different types of map to standard SUMO network.
Currently, the tool supports three types of map:

* Open Street Map
* OpenDrive High Definition Map
* Shapefiles

Flow Generator
~~~~~~~~~~~~~~~
This tool is aimed to generate naturalistic traffic flow.
There are two modes:

* Based on shortest path rule, please check :func:`tools.flow_setting.generate_routes` for more reference.
* Based on turning-ratio of junctions, please check :func:`tools.flow_setting.generate_routes_turnratio` for more reference.

Traffic Simulation Operating System
------------------------------------

Synchronization
~~~~~~~~~~~~~~~~~~~~~~
One important part of the Operating System is the Synchronization. Its main function is to communicate with SUMO Traci server. To be specific, the OS will get all the traffic information of SUMO simulation and send to the traffic participants, including vehicles, pedestrians and traffic lights. After receiving the raw data, all the traffic participants will determine their actions at every time step and send back to the Synchronization part for update.

Vehicle
~~~~~~~~~~~~
Vehicle, as one of the most important traffic participants, has four main functions:

* Generate the specific traffic flow based on certain rules,
* Observe the surroundings and extract useful information,
* Make decisions for the next time step based on some certain information,
* Control their microscopic states, including position, speed, acceleration and heading.

As for the **Decision-making** component, the default one is to completely follow SUMO models, such as IDM car following model and LC2013 lane change model. For research purpose, the NDE controller and NADE controller have been developed to simulate human driving behaviors. The NDE controller decides the vehicle maneuver by sampling the corresponding NDD distribution. As for the NADE controller, it first calculates the challenge of each maneuver, then estimates the criticality. Finally, it will sample the criticality array and get the next action.

Vulnerable Road Users
~~~~~~~~~~~~~~~~~~~~~~
This part is aimed to develop models for pedestrians and it will developed in the future. 

Traffic Light
~~~~~~~~~~~~~~~
As for the Traffic Light, it will has a similar structure with Vehicle. Basically, every traffic light will extract the traffic information and set its signal state every time step. This component is being developed.

Network
~~~~~~~~~
Network is an indispensable part of the MTL Simulation Platform. Currently, it contains the static information, such as the length, width, and speed limits of lanes, the road connections at a junction, and so on. In the future, we plan to expand the functionalities and add dynamic information, including vehicle information of certain roads, to the Network.

Data Processor
~~~~~~~~~~~~~~
Extra data processors are developed for specific purposes. 

* Information extractor
    Its main function is to get the snapshot of each time step, including the vehicle information. 
* Measurement
    Its main function is to estimate the safety of the scenario based on some metrics.
* Logging module
    It is implemented to record the initialization settings for replay. It also stores the termination reasons, such as the crash information, the controller decision before crash.