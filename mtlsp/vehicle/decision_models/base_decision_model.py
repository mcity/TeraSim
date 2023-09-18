from abc import ABC, abstractmethod
import numpy as np
from bidict import bidict
import mtlsp.utils as utils

class BaseDecisionModel(ABC):
    """DecisionModel class deal with the control of the vehicle based on observation
    """    
    def __init__(self):
        """Initialize a BaseDecisionModel object.

        Args:
            controllertype (str, optional): Type of the controller. Defaults to None.
        """        
        self.control_log = {} # This will have the control log result for each controller
    
    def derive_control_command_from_observation(self, obs_dict):
        raise ValueError("Decision model decision function not implemented!")
    
    def reset(self):
        pass

    def install(self):
        utils.set_vehicle_speedmode(self.vehicle.id)
        utils.set_vehicle_lanechangemode(self.vehicle.id)
        self.vehicle.simulator.set_vehicle_color(self.vehicle.id, self.vehicle.COLOR_YELLOW)




