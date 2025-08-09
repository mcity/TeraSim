from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.vehicle import Vehicle

from .conflict_generation_model import ConflictGenerationModel
from .nde_controller import NDEController
from .nde_decision_model import NDEDecisionModel
from .nde_ego_sensor import NDEEgoSensor
from terasim.overlay import traci
from .emergency_vehicle_model import EmergencyVehicleModel

class NDEVehicleFactory(VehicleFactory):
    def __init__(self, cfg=None):
        super().__init__()
        self.cfg = cfg

    def create_vehicle(self, veh_id, simulator):
        """Create a vehicle object.

        Args:
            veh_id (str): Vehicle ID.
            simulator (Simulator): Simulator object.

        Returns:
            Vehicle: Vehicle object.
        """
        sensor_list = [
            NDEEgoSensor(cache=True, cache_history=True, cache_history_duration=1)
        ]
        if veh_id == "AV":
            decision_model = NDEDecisionModel(
                MOBIL_lc_flag=True,
                stochastic_acc_flag=False,
                reroute=False,
                dynamically_change_vtype=False,
            )
        elif traci.vehicle.getVehicleClass(veh_id) == "emergency":
            decision_model = EmergencyVehicleModel()
        else:
            decision_model = ConflictGenerationModel(
                MOBIL_lc_flag=True,
                stochastic_acc_flag=False,
                dynamically_change_vtype=False,
                cfg=self.cfg,
            )

        controller = NDEController(simulator)
        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )
