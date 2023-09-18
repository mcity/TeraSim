from .base import PhysicSimulator, SensorData

class DummyPhysics(PhysicSimulator):
    """
    This class provide a dummy physics simulator for sanity check.
    """

    def __init__(self, step_length: float, **connect_args) -> None:
        self._step_length = step_length
        self._actor_pool = set()
        self._actor_dep = dict() # child -> parent
        print("Connected with args:", list(connect_args.keys()))

    def initialize(self, **kwargs) -> bool:
        print("Initialized a scene with args:", list(kwargs.keys()))
        return True
    
    def dispose(self, **kwargs) -> bool:
        print("Dispose the scene with args:", list(kwargs.keys()))
        return True

    def add_agent(self, model, actor_id, attach_to=None):
        assert model is not None, "The actor model must be specified!"
        assert actor_id not in self._actor_pool, "The actor id already exists!"
        assert attach_to is None or attach_to in self._actor_pool,\
            "The id of the parent doesn't exist!"

        self._actor_pool.add(actor_id)
        print("Spawned actor", actor_id, end="")
        if attach_to is not None:
            self._actor_pool.add(actor_id)
            self._actor_dep[actor_id] = attach_to
            print(" attaching to", attach_to)
        else:
            print("") # newline
        return True

    def spawn_actors(self, models_map, attach_to=None):
        assert all(actor not in self._actor_pool for actor in models_map.keys()),\
            "At least one of the actor ids already exists!"
        
        self._actor_pool.update(models_map.keys())
        print("Spawned actors:", list(models_map.keys()), end="")

        if attach_to is not None:
            for aid in models_map.keys():
                self._actor_dep[aid] = attach_to
            print(" attaching to", attach_to)
        else:
            print("") # newline

        return True

    def remove_agent(self, actor_id):
        assert actor_id in self._actor_pool,\
            "The actor to be destroyed doesn't exist!"

        # remove actors attaching to this actor
        children_to_remove = []
        for (child, parent) in self._actor_dep.items():
            if parent == actor_id:
                children_to_remove.append(child)
        self.destroy_actors(children_to_remove)

        self._actor_pool.remove(actor_id)
        print("Destroyed actor", actor_id)
        return True

    def destroy_actors(self, actor_ids):
        if len(actor_ids) == 0:
            # shortcut for empty list
            return True

        actor_ids = set(actor_ids)
        assert all(aid in self._actor_pool for aid in actor_ids),\
            "At least one of the actor to be destroyed doesn't exist!"

        # remove actors attaching to the deleted actor
        children_to_remove = []
        for (child, parent) in self._actor_dep.items():
            if parent in actor_ids:
                children_to_remove.append(child)
        self.destroy_actors(children_to_remove)

        self._actor_pool = self._actor_pool.difference(actor_ids)
        print("Destroyed actors:", actor_ids)
        return True

    def get_agent_state(self, actor_id):
        assert actor_id in self._actor_pool,\
            "The actor to be queried doesn't exist!"

        print("Query actor", actor_id)
        return dict(pose=None, lights=None)

    def set_agent_state(self, actor_id, pose=None, lights=None, **kwargs):
        assert actor_id in self._actor_pool,\
            "The actor to be modified doesn't exist!"

        print("Update actor", actor_id)
        return True

    def set_actor_command(self, actor_id, control):
        assert actor_id in self._actor_pool,\
            "The actor to be modified doesn't exist!"

        print("Update actor", actor_id, "command")
        return True

    def tick(self):
        return SensorData(dict())

    def __del__(self):
        print("Destroyed remaining actors:", self._actor_pool)
