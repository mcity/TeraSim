import uvicorn

from terasim_service.api import create_app

def main():
    # Create the FastAPI application
    app = create_app()
    
    # Run the application with uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main() 