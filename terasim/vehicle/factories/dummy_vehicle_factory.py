from terasim.vehicle.factories.vehicle_fatory import VehicleFactory
from terasim.vehicle.controllers.high_efficiency_controller import HighEfficiencyController
from terasim.vehicle.vehicle import Vehicle
from terasim.vehicle.decision_models.dummy_decision_model import DummyDecisionModel


class DummyVehicleFactory(VehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        sensors = {}
        decision_model = DummyDecisionModel(mode="constant")
        control_params = {
            "step_size": simulator.step_size,
            "action_step_size": simulator.action_step_size,
            "sublane_flag": simulator.sublane_flag,
            "lc_duration": simulator.lc_duration,
        }
        controller = HighEfficiencyController(simulator, control_params)
        return Vehicle(veh_id, simulator, sensors=sensors, decision_model=decision_model, controller=controller)
        