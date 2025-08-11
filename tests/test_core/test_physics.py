import pytest

from terasim.physics import DummyPhysics


def test_dummy():
    dummy_model = "model"
    sim = DummyPhysics(step_length=0.2)
    sim.add_agent(dummy_model, "car_a")
    sim.add_agent(dummy_model, "sensor_a", attach_to="car_a")
    sim.spawn_actors({"car_b": dummy_model, "car_c": dummy_model})

    sim.get_agent_state("car_a")
    sim.get_agent_state("sensor_a")
    sim.set_agent_state("car_a", lights=None)
    sim.set_actor_command("car_a", None)

    with pytest.raises(AssertionError):
        sim.remove_agent("car_d")
    sim.remove_agent("car_a")
    with pytest.raises(AssertionError):
        sim.destroy_actors(["car_a", "car_b"])
    sim.destroy_actors(["car_b", "car_c"])

    with pytest.raises(AssertionError):
        # sensor should be removed when its parent is removed
        sim.remove_agent("sensor_a")
