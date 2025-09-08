import json
import os
import threading
from pathlib import Path

# Create locks for each file path
_FILE_LOCKS = {}
_LOCK_LOCK = threading.Lock()  # Lock to protect the _FILE_LOCKS dictionary

def _get_file_lock(file_path):
    """Get a lock for a specific file, create it if it doesn't exist"""
    file_path_str = str(file_path)
    with _LOCK_LOCK:
        if file_path_str not in _FILE_LOCKS:
            _FILE_LOCKS[file_path_str] = threading.Lock()
        return _FILE_LOCKS[file_path_str]

def load_metadata(file_path):
    """
    Load metadata from a specified path
    
    Args:
        file_path: Path to the metadata file
        
    Returns:
        dict: Metadata dictionary, returns empty dict if file doesn't exist
    """
    file_path = Path(file_path)
    if not file_path.exists():
        return {}
    
    with _get_file_lock(file_path):
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except json.JSONDecodeError:
            # If file exists but is not valid JSON, return empty dict
            return {}

def save_metadata(file_path, data):
    """
    Save metadata to a specified path
    If the file doesn't exist, it will be created
    If parent directories don't exist, they will be created as well
    
    Args:
        file_path: Path to the metadata file
        data: Metadata dictionary to save
        
    Returns:
        bool: True if operation was successful
    """
    file_path = Path(file_path)
    
    # Ensure directory exists
    file_path.parent.mkdir(parents=True, exist_ok=True)
    
    with _get_file_lock(file_path):
        with open(file_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
    
    return True

def update_metadata(file_path, key, value):
    """
    Update a specific key-value pair in the metadata file
    If the file doesn't exist, it will be created with the specified key-value pair
    
    Args:
        file_path: Path to the metadata file
        key: Key to update
        value: New value
        
    Returns:
        bool: True if file existed and was updated, False if a new file was created
    """
    file_path = Path(file_path)
    file_existed = file_path.exists()
    
    with _get_file_lock(file_path):
        data = load_metadata(file_path)
        data[key] = value
        save_metadata(file_path, data)
    
    return file_existed

def get_metadata_value(file_path, key, default=None):
    """
    Get a specific value from the metadata
    
    Args:
        file_path: Path to the metadata file
        key: Key to retrieve
        default: Default value if key doesn't exist
        
    Returns:
        Value for the key, or default if key doesn't exist
    """
    data = load_metadata(file_path)
    return data.get(key, default)

def delete_metadata_key(file_path, key):
    """
    Delete a specific key from the metadata
    
    Args:
        file_path: Path to the metadata file
        key: Key to delete
    """
    file_path = Path(file_path)
    with _get_file_lock(file_path):
        data = load_metadata(file_path)
        if key in data:
            del data[key]
            save_metadata(file_path, data)
