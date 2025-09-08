from pydantic import BaseModel


class AgentState(BaseModel):
    # Acceleration
    ## angular acceleration of the agent (radians per second squared)
    angular_acc: float = 0.0
    ## longitudinal acceleration of the agent (meters per second squared)
    accel_long: float = 0.0
    ## lateral acceleration of the agent (meters per second squared)
    accel_lat: float = 0.0

    # Position
    ## x position of the agent in the UTM coordinate system (meters)
    x: float = 0.0
    ## y position of the agent in the UTM coordinate system (meters)
    y: float = 0.0
    ## elevation of the agent (meters)
    z: float = 0.0

    # Orientation
    ## orientation of the agent, ranging from -pi to pi, where 0 means the agent is heading to the east, pi/2 means the agent is heading to the north (radians)
    orientation: float = 0.0

    # Slope
    ## slope of the agent (radians)
    slope: float = 0.0

    # Size (https://www.autoscout24.de/auto/technische-daten/mercedes-benz/vito/vito-111-cdi-kompakt-2003-2014-transporter-diesel/)
    ## length of the agent (meters)
    length: float = 5.0
    ## width of the agent (meters)
    width: float = 1.8
    ## height of the agent (meters)
    height: float = 1.5

    # Speed
    ## angular speed of the agent (radians per second)
    angular_speed: float = 0.0
    ## longitudinal speed of the agent (meters per second)
    speed_long: float = 0.0
    ## lateral speed of the agent (meters per second)
    speed_lat: float = 0.0

    # additional information of the agent
    direction_x: float = 0.0
    direction_y: float = 0.0

    # additional information of the agent
    type: str = ""
    additional_information: str = ""
