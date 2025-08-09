from pydantic import BaseModel
import time


class Header(BaseModel):
    # timestamp of the Redis message
    timestamp: float = time.time()
    # information of the Redis message
    information: str = ""
