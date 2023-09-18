Overview
===================================================

A SUMO-based simulation platform, including several applications, such as AV testing simulation, real-time implementation and so on.

About the Platform
------------------------

This platform is aiming at providing a SUMO-based simulation platform. The main objectives of this platform contains:

* Modeling human driving behaviors and naturalistic collision situation;
* Developing comprehensive AV testing environment;
* Solving complex traffic research problems.

In this platform, the developed Naturalistic Driving Environment (NDE) is applied to improve the vehicle models to make the simulation become more realistic. Besides, the developed Naturalistic Adversarial Driving Environment (NADE) is implemented to model realistic collisions and improve testing efficiency. This platform also integrates the co-simulation between SUMO and Carla, and the Mcity Augmented-reality system, which makes it possible to test AV technology with simulation and testing facilities.

Architecture
-------------------
The whole architecture of MTL Simulation Platform is shown below:

.. image:: ../source/_static/arch.svg
    :width: 100%
    :align: center
    :alt: algorithm-flowchart

The left side shows the basic logic of SUMO simulation. With the traffic network and traffic demand as inputs, SUMO could conduct the microscopic simulation based on the embedded models for all the traffic participants, such as pedestrians, vehicles, traffic lights, and so on. SUMO also provides the Traci server for interaction with python. Then, the traffic information obtained from the SUMO simulation through Traci is used to construct the traffic simulation OS for extra control. Besides, all the traffic information can be extracted and output for further analysis. There are multiple applications based on the MTL Simulation Platform. For example, the NDE/NADE controller has been developed to conduct the safety testing for AV technology, and it is also possible to implemented the developed learning-based traffic light controller in this platform. The whole platform can synchronize with other simulation platform. For example, the simulation platform is designed to be able to communicate with the MCity AR system. In addition, through the specific data interface, the Carla simulator can obtain the information about the network, vehicles and traffic lights, and then visualize them.

.. raw:: html

   <!-- CONTRIBUTING -->

Contributing
------------------------

Contributions are what make the open source community such an amazing
place to be learn, inspire, and create. Any contributions you make are
**greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch
   (``git checkout -b feature/AmazingFeature``)
3. Commit your Changes (``git commit -m 'Add some AmazingFeature'``)
4. Push to the Branch (``git push origin feature/AmazingFeature``)
5. Open a Pull Request

.. raw:: html

   <!-- LICENSE -->

License
------------------------

Distributed under the MIT License.

Contact
-------------

Haowei Sun - haoweis@umich.edu - Michigan Traffic Lab

Haojie Zhu - zhuhj@umich.edu - Michigan Traffic Lab

Shuo Feng- fshuo@umich.edu - Michigan Traffic Lab

Henry Liu - henryliu@umich.edu - Michigan Traffic Lab

Project Link: https://github.com/michigan-traffic-lab/TeraSim
