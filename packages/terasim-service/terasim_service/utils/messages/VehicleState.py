from pydantic import BaseModel

from .Header import Header


class VehicleState(BaseModel):
    header: Header = Header()

    # Vehicle Speed
    speed_x: float = 0.0
    speed_y: float = 0.0
    speed_z: float = 0.0

    # Drive by wire status
    by_wire_enabled: bool = False

    # Gear state
    # NONE   	 =0
    # PARK       =1
    # REVERSE    =2
    # REVERSE    =2
    # NEUTRAL    =3
    # DRIVE      =4
    # LOW        =5
    gear_pos: int = 0

    # Engine state
    engine_rpm: float = 0.0

    # Throttle state
    throttle_cmd: float = 0.0
    throttle_input: float = 0.0
    throttle_state: float = 0.0
    throttle_enabled: bool = False
    throttle_override: bool = False
    throttle_driver: bool = False
    throttle_timeout: bool = False

    # Brake state
    brake_cmd: float = 0.0
    brake_input: float = 0.0
    brake_state: float = 0.0
    brake_torq_cmd: float = 0.0
    brake_torq_input: float = 0.0
    brake_torq_state: float = 0.0
    brake_boo_output: bool = False
    brake_enabled: bool = False
    brake_override: bool = False
    brake_driver: bool = False
    brake_timeout: bool = False

    # Steering state
    steer_cmd: float = 0.0
    steer_state: float = 0.0
    steer_torque: float = 0.0
    steer_enabled: bool = False
    steer_override: bool = False
    steer_timeout: bool = False

    # Wheel state
    wheel_v_front_left: float = 0.0
    wheel_v_front_right: float = 0.0
    wheel_v_rear_left: float = 0.0
    wheel_v_rear_right: float = 0.0
