"""
Automatic MCP tool wrapper generator
Creates MCP tools from existing Python classes and methods
"""

from mcp.server import Server
from mcp import types
import importlib
import inspect
import traceback
import json
from typing import Any, Dict, List, Optional
from .tool_config import TOOL_MAPPINGS

def auto_register_all_tools(server: Server):
    """Register all tools based on configuration"""
    
    registered_count = 0
    
    for tool_group, config in TOOL_MAPPINGS.items():
        try:
            # Import the module
            module = importlib.import_module(config["module"])
            target_class = getattr(module, config["class"])
            
            # Register each method as a tool
            for method_name, method_config in config["methods"].items():
                tool_name = f"{tool_group}_{method_name}"
                
                success = create_tool_wrapper(
                    server=server,
                    tool_name=tool_name,
                    target_class=target_class,
                    method_name=method_name,
                    method_config=method_config,
                    init_args=config["init_args"]
                )
                
                if success:
                    registered_count += 1
                    print(f"   ✅ Registered: {tool_name}")
                else:
                    print(f"   ❌ Failed to register: {tool_name}")
                    
        except Exception as e:
            print(f"   ❌ Error registering {tool_group}: {str(e)}")
            traceback.print_exc()
    
    print(f"📊 Total registered tools: {registered_count}")
    return registered_count

def create_tool_wrapper(
    server: Server, 
    tool_name: str, 
    target_class, 
    method_name: str, 
    method_config: Dict[str, Any],
    init_args: Dict[str, Any]
) -> bool:
    """Create a single MCP tool wrapper"""
    
    try:
        # Get method info
        method = getattr(target_class, method_name)
        method_sig = inspect.signature(method)
        
        # Create the wrapper function
        async def tool_wrapper(arguments: dict) -> List[types.TextContent]:
            """Auto-generated MCP tool wrapper"""
            try:
                # Validate and prepare arguments
                validated_args = validate_arguments(arguments, method_config["parameters"])
                
                # Create instance with init args
                if init_args:
                    instance = target_class(**init_args)
                else:
                    instance = target_class()
                
                # Call the method
                result = method(instance, **validated_args)
                
                # Format result
                formatted_result = format_tool_result(result, tool_name)
                
                return [types.TextContent(
                    type="text",
                    text=formatted_result
                )]
                
            except Exception as e:
                error_msg = f"❌ Error in {tool_name}: {str(e)}"
                print(error_msg)
                traceback.print_exc()
                
                return [types.TextContent(
                    type="text",
                    text=error_msg
                )]
        
        # Set function metadata
        tool_wrapper.__name__ = tool_name
        tool_wrapper.__doc__ = method_config.get("description", f"Execute {tool_name}")
        
        # Register with server
        server.call_tool()(tool_wrapper)
        
        return True
        
    except Exception as e:
        print(f"Failed to create wrapper for {tool_name}: {str(e)}")
        return False

def validate_arguments(arguments: dict, param_config: dict) -> dict:
    """Validate and prepare arguments based on parameter configuration"""
    
    validated = {}
    
    for param_name, param_info in param_config.items():
        if param_name in arguments:
            # Use provided value
            validated[param_name] = arguments[param_name]
        elif "default" in param_info:
            # Use default value
            validated[param_name] = param_info["default"]
        elif not param_info.get("optional", False):
            # Required parameter missing
            raise ValueError(f"Required parameter '{param_name}' is missing")
    
    return validated

def format_tool_result(result: Any, tool_name: str) -> str:
    """Format tool execution result for display"""
    
    try:
        if result is None:
            return f"✅ {tool_name} completed successfully (no return value)"
        
        # Handle different result types
        if isinstance(result, (str, int, float, bool)):
            return f"✅ {tool_name} completed: {result}"
        
        elif isinstance(result, (list, tuple)):
            if len(result) == 0:
                return f"✅ {tool_name} completed: empty list"
            elif len(result) <= 5:
                return f"✅ {tool_name} completed: {result}"
            else:
                return f"✅ {tool_name} completed: list with {len(result)} items: {result[:3]}..."
        
        elif isinstance(result, dict):
            if len(result) <= 3:
                return f"✅ {tool_name} completed: {result}"
            else:
                keys = list(result.keys())[:3]
                return f"✅ {tool_name} completed: dict with {len(result)} keys: {keys}..."
        
        else:
            # Convert to string, but limit length
            result_str = str(result)
            if len(result_str) > 500:
                result_str = result_str[:500] + "..."
            return f"✅ {tool_name} completed: {result_str}"
            
    except Exception as e:
        return f"✅ {tool_name} completed (result formatting error: {str(e)})"

def get_tool_info(tool_name: str) -> Optional[Dict[str, Any]]:
    """Get information about a specific tool"""
    
    for tool_group, config in TOOL_MAPPINGS.items():
        for method_name, method_config in config["methods"].items():
            if f"{tool_group}_{method_name}" == tool_name:
                return {
                    "name": tool_name,
                    "group": tool_group,
                    "method": method_name,
                    "description": method_config.get("description", ""),
                    "parameters": method_config.get("parameters", {}),
                    "module": config["module"],
                    "class": config["class"]
                }
    
    return None

def list_available_tools() -> List[str]:
    """List all available tools that can be registered"""
    
    tools = []
    for tool_group, config in TOOL_MAPPINGS.items():
        for method_name in config["methods"].keys():
            tools.append(f"{tool_group}_{method_name}")
    
    return sorted(tools)