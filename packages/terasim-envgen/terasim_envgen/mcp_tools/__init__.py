"""
MCP Tools for TeraSim-Agent
Provides automatic tool registration and server initialization
"""

from mcp.server import Server
import mcp.server.stdio
from .tool_config import TOOL_MAPPINGS
from .auto_wrapper import auto_register_all_tools
import asyncio

# Global server instance
_server = None

def get_server() -> Server:
    """Get the global MCP server instance"""
    global _server
    if _server is None:
        _server = Server("terasim-agent")
    return _server

async def start_mcp_server():
    """Start the MCP server with auto-registered tools"""
    server = get_server()
    
    # Auto-register all tools based on configuration
    auto_register_all_tools(server)
    
    print(f"🚀 TeraSim-Agent MCP Server starting...")
    # Note: Cannot access tool handlers directly in current MCP version
    print(f"📋 Tools registration completed")
    
    # Start stdio server
    async with mcp.server.stdio.stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            server.create_initialization_options()
        )

def main():
    """Entry point for MCP server"""
    asyncio.run(start_mcp_server())

if __name__ == "__main__":
    main()