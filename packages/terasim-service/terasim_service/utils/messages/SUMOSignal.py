from pydantic import BaseModel


class SUMOSignal(BaseModel):
    # Position
    ## x position of the signalized intersection in the UTM coordinate system (meters)
    x: float = 0.0
    ## y position of the signalized intersection in the UTM coordinate system (meters)
    y: float = 0.0

    # traffic light status defined in SUMO
    tls: str = ""

    # Other information
    information: str = ""
