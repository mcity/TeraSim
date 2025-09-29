import sys
from pathlib import Path

# Add the parent directory to sys.path if running directly from examples
parent_dir = Path(__file__).resolve().parent.parent
if parent_dir not in sys.path:
    sys.path.insert(0, str(parent_dir))

from terasim_service.api import create_app
import uvicorn

if __name__ == "__main__":
    # Create the FastAPI application
    app = create_app()
    
    # Run the application
    uvicorn.run(app, host="0.0.0.0", port=8000)
