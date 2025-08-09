from pydantic import BaseModel


class AgentCommand(BaseModel):
    agent_id: str = ""
    agent_type: str = "" # chosen from ["vehicle", "vru"]
    command_type: str = "" # chosen from ["set_state",]
    data: dict = {}