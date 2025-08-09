from pydantic import BaseModel
from typing import Dict

from .Header import Header
from .SUMOSignal import SUMOSignal


class SUMOSignalDict(BaseModel):
    # header of the message
    header: Header = Header()

    # dictionary of the SUMO signals, with the signal ID as the key and the signal information as the value
    data: Dict[str, SUMOSignal] = {}

    av_next_tls: str = ""
    av_next_dist: float = 0.0
