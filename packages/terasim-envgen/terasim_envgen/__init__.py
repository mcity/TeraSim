"""
TeraSim Environment Generation Package

Automatically configures SUMO environment on import.
This package provides tools for generating realistic driving environments
for autonomous vehicle testing including map generation, traffic flow
generation, and scenario creation.
"""

import os
import sys
from pathlib import Path
import logging

logger = logging.getLogger(__name__)

def _setup_sumo_environment():
    """
    Automatically setup SUMO environment.
    Checks multiple locations and configures SUMO_HOME and tools path.
    
    Returns:
        bool: True if SUMO was found and configured, False otherwise
    """
    
    # Skip if already configured
    if os.getenv("SUMO_HOME"):
        sumo_home = os.getenv("SUMO_HOME")
        tools_path = os.path.join(sumo_home, "tools")
        if os.path.exists(tools_path):
            logger.debug(f"SUMO_HOME already set: {sumo_home}")
            return True
    
    # List of possible SUMO locations (in priority order)
    possible_locations = [
        # 1. User-level installation (created by setup_environment.sh)
        Path.home() / ".terasim" / "deps" / "sumo",
        
        # 2. Project-level deps (for development)
        Path(__file__).parent.parent.parent.parent / "deps" / "sumo",
        
        # 3. Check for .sumo_home file with saved path
        _get_saved_sumo_path(),
        
        # 4. System-wide installation
        Path("/usr/share/sumo"),
        Path("/usr/local/share/sumo"),
        
        # 5. Common Windows locations
        Path("C:/Program Files/Eclipse/Sumo"),
        Path("C:/Program Files (x86)/Eclipse/Sumo"),
    ]
    
    # Find the first valid SUMO installation
    for sumo_path in possible_locations:
        if sumo_path and sumo_path.exists() and (sumo_path / "tools").exists():
            # Set environment variables
            os.environ["SUMO_HOME"] = str(sumo_path)
            os.environ["SUMO_TOOLS_PATH"] = str(sumo_path / "tools")
            
            # Add tools to Python path
            tools_path = str(sumo_path / "tools")
            if tools_path not in sys.path:
                sys.path.insert(0, tools_path)
            
            logger.info(f"SUMO environment configured: SUMO_HOME={sumo_path}")
            return True
    
    # If no SUMO found, log warning but don't fail
    logger.warning(
        "SUMO tools not found. Some features may not work. "
        "Run 'bash setup_environment.sh' to install SUMO tools automatically."
    )
    return False

def _get_saved_sumo_path():
    """
    Get saved SUMO_HOME path from .sumo_home file.
    
    Returns:
        Path or None: Saved SUMO_HOME path if found
    """
    try:
        sumo_home_file = Path.home() / ".terasim" / "deps" / ".sumo_home"
        if sumo_home_file.exists():
            content = sumo_home_file.read_text().strip()
            if content.startswith("SUMO_HOME="):
                sumo_path = content.split("=", 1)[1]
                return Path(sumo_path)
    except Exception:
        pass
    return None

# Run setup when package is imported
_sumo_configured = _setup_sumo_environment()

# Export main components
from .core.map_searcher import MapSearcher
from .core.map_converter import MapConverter
from .core.traffic_flow_generator import TrafficFlowGenerator
from .core.integrated_generator import IntegratedScenarioGenerator

__all__ = [
    "MapSearcher",
    "MapConverter", 
    "TrafficFlowGenerator",
    "IntegratedScenarioGenerator",
]

__version__ = "0.2.0"