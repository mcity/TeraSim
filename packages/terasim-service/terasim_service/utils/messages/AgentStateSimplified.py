from pydantic import BaseModel


class AgentStateSimplified(BaseModel):
    # Position
    ## x position of the agent in the SUMO coordinate system (meters)
    x: float = 0.0
    ## y position of the agent in the SUMO coordinate system (meters)
    y: float = 0.0
    ## elevation of the agent (meters)
    z: float = 0.0
    
    ## longitude of the agent (degrees)
    lon: float = 0.0
    ## latitude of the agent (degrees)
    lat: float = 0.0

    # Orientation in the SUMO coordinate system
    sumo_angle: float = 0.0

    # Size (https://www.autoscout24.de/auto/technische-daten/mercedes-benz/vito/vito-111-cdi-kompakt-2003-2014-transporter-diesel/)
    ## length of the agent (meters)
    length: float = 5.0
    ## width of the agent (meters)
    width: float = 1.8
    ## height of the agent (meters)
    height: float = 1.5

    # Speed
    speed: float = 0.0

    # Orientation
    orientation: float = 0.0
    acceleration: float = 0.0
    angular_velocity: float = 0.0

    # additional information of the agent
    type: str = ""
