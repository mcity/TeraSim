"""
TeraSim-Cosmos Package

A bridge package that converts TeraSim traffic simulation data (SUMO map and FCD)
into NVIDIA Cosmos-Drive compatible inputs for world model training and video generation.
"""

from .converter import TeraSimToCosmosConverter
from .street_view_analysis import StreetViewRetrievalAndAnalysis

__version__ = "0.1.0"
__all__ = ["TeraSimToCosmosConverter", "StreetViewRetrievalAndAnalysis"]