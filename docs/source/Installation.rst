Installation
======================================

The installation process is shown as follows.

Prerequisites
------------------------

The platform is mainly developed and tested on Ubuntu 18.04 LTS. Besides, this project has been tested on Ubuntu 20.04 LTS. Therefore, we recommend running the following process on Ubuntu 18.04 LTS or later version. 


SUMO Installation
------------------------
Also, this project is based on a modified version of SUMO, in which we customize the background vehicle driving model, including the car-following model and the lane-change model. To access the SUMO project, please go to https://github.com/michigan-traffic-lab/SUMO_NDE.git for
more references.

.. code:: bash

    sudo apt-get install cmake python3 g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev libgl2ps-dev git
    git clone https://github.com/michigan-traffic-lab/SUMO_NDE.git
    export SUMO_HOME="$PWD/SUMO_NDE"
    mkdir SUMO_NDE/build/cmake-build && cd SUMO_NDE/build/cmake-build
    cmake ../..
    make -j$(nproc)


Carla Installation
----------------------------
If Carla visualization tools are needed, you need have Carla installed
in the environment. Also, the carla Python API package should be
installed to your site packages. Please refer to
https://carla.readthedocs.io/en/latest/#getting-started for more
details.

Other packages
-----------------------
Additionally, there are some packages used in the project, please run
the following commands to install them.

.. code:: bash

    pip3 install -r requirements.txt


Data folder
------------------------
In order to utilize the naturalistic driving data, additional databases are needed here. Please download the database on https://drive.google.com/drive/u/1/folders/1ATsjn-eB32Qd7aNtpE\_Lqfal1w-46Znr.

There are two ways to load the data - put the ``Data`` folder directly under the directory
that you are running the script or: - set environment variable
``NDD_DATA_PATH`` the path to the ``Data`` folder, e.g.:
``bash  export NDD_DATA_PATH="/home/user/Data"``

.. raw:: html

   <!-- USAGE EXAMPLES -->