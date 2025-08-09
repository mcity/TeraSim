from pydantic import BaseModel


class Actor(BaseModel):
    # Acceleration
    ## angular acceleration of the vehicle (radians per second squared)
    angular_acc: float = 0.0
    ## longitudinal acceleration of the vehicle (meters per second squared)
    accel_long: float = 0.0
    ## lateral acceleration of the vehicle (meters per second squared)
    accel_lat: float = 0.0

    # Position
    ## x position of the vehicle in the UTM coordinate system (meters)
    x: float = 0.0
    ## y position of the vehicle in the UTM coordinate system (meters)
    y: float = 0.0
    ## elevation of the vehicle (meters)
    z: float = 0.0

    # Orientation
    ## orientation of the vehicle, ranging from -pi to pi, where 0 means the vehicle is heading to the east, pi/2 means the vehicle is heading to the north (radians)
    orientation: float = 0.0

    # Slope
    ## slope of the vehicle (radians)
    slope: float = 0.0

    # Size (https://www.autoscout24.de/auto/technische-daten/mercedes-benz/vito/vito-111-cdi-kompakt-2003-2014-transporter-diesel/)
    ## length of the vehicle (meters)
    length: float = 5.0
    ## width of the vehicle (meters)
    width: float = 1.8
    ## height of the vehicle (meters)
    height: float = 1.5

    # Speed
    ## angular speed of the vehicle (radians per second)
    angular_speed: float = 0.0
    ## longitudinal speed of the vehicle (meters per second)
    speed_long: float = 0.0
    ## lateral speed of the vehicle (meters per second)
    speed_lat: float = 0.0

    # additional information of the vehicle
    direction_x: float = 0.0
    direction_y: float = 0.0

    # additional information of the vehicle
    type: str = ""
    additional_information: str = ""
