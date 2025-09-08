from pydantic import BaseModel

from .Header import Header


class VehiclePlanning(BaseModel):
    header: Header = Header()

    # Time resolution (seconds)
    time_resolution: float = 0.0

    go: int = 0  # 0 represents not running while 1 represent running

    ## List of x coordinate of the vehicle's center in UTM coordinate (meters)
    x_list: list[float] = []
    ## List of y coordinate of the vehicle's center in UTM coordinate (meters)
    y_list: list[float] = []
    ## List of desired speed of the vehicle (meters per second)
    speed_list: list[float] = []
    ## List of desired orientation of the vehicle ranging from -pi to pi, where 0 means the vehicle is heading to the east, pi/2 means the vehicle is heading to the north (radians)
    orientation_list: list[float] = []
