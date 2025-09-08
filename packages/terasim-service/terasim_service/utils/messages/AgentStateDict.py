from pydantic import BaseModel
from typing import Dict

from .Header import Header
from .Agent import Actor


class ActorDict(BaseModel):
    # header of the message
    header: Header = Header()

    # dictionary of the vehicles, with the vehicle ID as the key and the vehicle information as the value
    data: Dict[str, Actor] = {}
