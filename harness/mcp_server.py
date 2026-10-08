"""The harness tools as an MCP server, so Claude Code can act as the agent.

Same tools, same Toolbox, same rules as agent.py: only the LLM runtime differs.
Claude Code starts this process itself (see agent_cc.py); you don't run it by hand.
"""
import asyncio
import json

import mcp.types as types
from mcp.server.lowlevel import Server
from mcp.server.stdio import stdio_server

from .tools import TOOL_SCHEMAS, Toolbox

server = Server("lqr-harness")
toolbox = Toolbox("agent")


@server.list_tools()
async def list_tools():
    return [types.Tool(name=t["name"], description=t["description"], inputSchema=t["input_schema"])
            for t in TOOL_SCHEMAS]


@server.call_tool()
async def call_tool(name, arguments):
    # simulations block for seconds to a minute: run them off the event loop
    out = await asyncio.to_thread(toolbox.call, name, dict(arguments or {}))
    return [types.TextContent(type="text", text=json.dumps(out))]


async def main():
    async with stdio_server() as (read, write):
        await server.run(read, write, server.create_initialization_options())


if __name__ == "__main__":
    asyncio.run(main())
