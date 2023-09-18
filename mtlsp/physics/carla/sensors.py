from enum import Enum
from mtlsp.overlay import carla
from mtlsp.vehicle.sensors.base import BaseSensor
from threading import Event

class CarlaSensor(BaseSensor):
    DEFAULT_PARAMS = dict(
        location = (0, 0, 0), # (x, y, z) in UE4 coordinate
        rotation = (0, 0, 0), # (pitch, yaw, roll) in UE4 coordinate
    )

    def __init__(self, name="carla_sensor", timeout=1.0, **params):
        '''
        timeout: timeout in seconds for waiting sensor data
        '''
        super().__init__(name, **params)

        x, y, z = self._params.pop('location')
        location = carla.Location(x, y, z)
        pitch, yaw, roll = self._params.pop('rotation')
        rotation = carla.Rotation(pitch, yaw, roll)
        self._transform = carla.Transform(location, rotation)

        self._actor = None # to be installed
        self._event = Event()
        self._timeout = timeout

    @property
    def blueprint(self) -> str:
        '''
        Get the name of the blueprint used to define the sensor in Carla
        '''
        raise NotImplementedError()

    @property
    def transform(self) -> carla.Transform:
        return self._transform

    def install(self, parent):
        super().install(parent)

        # create the sensor in Carla
        for p in self._simulator.plugins:
            if p.__class__.__name__ == 'CarlaPhysics':
                self._actor = p._spawn_sensor_in_carla(self)
                break

    def uninstall(self):        
        super().uninstall()

        for p in self._simulator.plugins:
            if p.__class__.__name__ == 'CarlaPhysics':
                p._destroy_sensor_in_carla(self)
                self._actor = None
                break

    def fetch(self):
        # Carla sensors return data through callbacks, so the fetch method will be waiting for the return.
        if self._actor.is_listening:
            self._event.wait(self._timeout)

class CameraType(Enum):
    RGB = "rgb"
    Depth = "depth"
    Semantic = "semantic_segmentation"
    Instance = "instance_segmentation"
    DVS = "dvs"
    OpticalFlow = "optical_flow"

class Camera(CarlaSensor):
    DEFAULT_PARAMS = dict(
        type = CameraType.RGB,
        image_size = (800, 600),
        **CarlaSensor.DEFAULT_PARAMS
    )

    def __init__(self, name="camera", **params):
        super().__init__(name, **params)
        self._type = self._params.pop('type')
        assert isinstance(self._type, CameraType)

        if self._params.image_size:
            width, height = self._params.pop('image_size')
            self._params.image_size_x = width
            self._params.image_size_y = height

    @property
    def blueprint(self) -> str:
        return "sensor.camera." + str(self._type.value)

    def subscribe(self):
        # TODO: lock?
        def store_data(data):
            sm = self._simulator.state_manager
            sm.push_variable(self._agent.id, self._name, data)
            self._event.set()
        self._actor.listen(store_data)

    def unsubscribe(self):
        self._actor.stop()
