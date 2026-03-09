from fastapi import FastAPI, APIRouter, Request
from typing import Optional
from pydantic import BaseModel
import subprocess
import json
import re


class CommandRequest(BaseModel):
    cmd: Optional[str] = None


router = APIRouter()


@router.post(
    "/cmd",
    response_model=dict,
    openapi_extra={
        "requestBody": {
            "content": {
                "application/json": {
                    "schema": {
                        "type": "object",
                        "properties": {
                            "cmd": {"type": "string", "description": "Command to execute"}
                        },
                        "required": ["cmd"]
                    },
                    "example": {
                        "cmd": 'ros2 service call /drive_command custom_drive_pkg/srv/DriveCommand "{forward: 0.0, rotate: 180.0}"'
                    }
                }
            }
        }
    }
)
async def on_cmd_received(request: Request):
    """Execute ROS2 or shell commands"""
    
    cmd = None
    
    # Try to parse the request body
    try:
        body = await request.json()
        cmd = body.get("cmd")
    except json.JSONDecodeError:
        # Handle malformed JSON with nested quotes
        try:
            raw_body = await request.body()
            body_str = raw_body.decode('utf-8')
            
            # Fix nested quotes in ROS2 commands
            # Pattern: "cmd": "text"inner"text"
            pattern = r'"cmd"\s*:\s*"([^"]*)"([^"]*)"([^"]*)"'
            match = re.search(pattern, body_str)
            
            if match:
                before = match.group(1)
                middle = match.group(2)
                after = match.group(3)
                fixed_body = f'{{"cmd": "{before}\\"{middle}\\"{after}"}}'
                body = json.loads(fixed_body)
                cmd = body.get("cmd")
        except Exception as e:
            return {"status": "failed", "error": f"JSON parsing error: {str(e)}"}
    
    if not cmd:
        return {"status": "failed", "error": "cmd required"}
    
    # Execute ROS/shell command (source ROS2 environment first)
    try:
        # Wrap command to source ROS2 before executing
        bash_cmd = f"bash -c 'source /opt/ros/jazzy/setup.bash && source ~/leo_ws/install/setup.bash 2>/dev/null || true && {cmd}'"
        
        result = subprocess.run(
            bash_cmd,
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30
        )

        return {
            "status": "success",
            "request": cmd,
            "output": result.stdout,
            "error": result.stderr
        }
    except subprocess.CalledProcessError as e:
        return {
            "status": "failed",
            "error": f"Command failed with exit code {e.returncode}",
            "output": e.stdout if e.stdout else "",
            "stderr": e.stderr if e.stderr else ""
        }
    except subprocess.TimeoutExpired:
        return {"status": "failed", "error": "Command timeout"}
    except Exception as e:
        return {"status": "failed", "error": str(e)}


@router.get("/ping")
def on_ping():
    return {"status": "success", "message": "pong"}