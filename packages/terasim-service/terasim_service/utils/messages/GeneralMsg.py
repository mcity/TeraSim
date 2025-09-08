from pydantic import BaseModel

from .Header import Header


class GeneralMsg(BaseModel):
    # header of the general Redis message
    header: Header = Header()

    # data of the Redis message
    data: str = ""
