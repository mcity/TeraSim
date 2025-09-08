from pydantic import BaseModel

from .Header import Header


class VehicleControl(BaseModel):
    header: Header = Header()

    # brake command as in percentage [0.0, 1.0]
    brake_cmd: float = 0.0

    # throttle command as in percentage [0.0, 1.0]
    throttle_cmd: float = 0.0

    ## steering_cmd as in radians [-2.5pi, 2.5pi]
    steering_cmd: float = 0.0

    # definition of gears
    # NONE   	 =0
    # PARK       =1
    # REVERSE    =2
    # REVERSE    =2
    # NEUTRAL    =3
    # DRIVE      =4
    # LOW        =5
    gear_cmd: int = 0
