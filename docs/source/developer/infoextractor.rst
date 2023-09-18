New Information Extractor
==========================

An Information Extractor class is aimed to extract information of simulation.
Before defining a new Information Extractor class, **please import the original Information Extractor class**.

.. code:: python3

    from core.logger.infoextrator import Infoextrator

A new Information Extractor class should at least have the following functions:

* :func:`__init__`:
    This function combines the Environment class. Besides, this function can record some simulation settings.

* :func:`add_initialization_info`:
    This function deals with the initialization information.

* :func:`get_snapshot_info`:
    This function deals with the snapshot information.

* :func:`get_terminate_info`:
    This function deals with the termination information.
