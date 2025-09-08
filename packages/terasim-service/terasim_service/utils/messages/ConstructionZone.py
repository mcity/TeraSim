from pydantic import BaseModel

from .Header import Header


class ConstructionZone(BaseModel):
    header: Header = Header()
    closed_lane_id: list[str] = [] # List of closed lane IDs
    closed_lane_shapes: list[list[list[float]]] = []  # Nested list of shapes for each lane
