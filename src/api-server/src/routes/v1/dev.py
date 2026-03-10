import subprocess
from fastapi import APIRouter
from pydantic import BaseModel

router = APIRouter()

class CommandPayload(BaseModel):
    #change to be an array of strings which will then all be processed in order
    command: list

    #payload = [('product_ids[]', id) for id in ids_list]
    #response = requests.post(url, data=payload, headers=headers)

class CommandOutput(BaseModel):
    output: str
    status_code: str
    error: str

@router.post("/dev", response_model=CommandOutput)
#change to payload: for x in CommandPayload.list or something like it
async def dev(payload: CommandPayload):
    try:
        #change this to for loop over an array command. Requires change to command class
        # Execute the command
        payload = [('commands[]', elem) for elem in command]
        for i in payload:
            result = subprocess.run(payload.command, shell=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output = result.stdout.decode('utf-8')
            error = result.stderr.decode('utf-8')

        return CommandOutput(output=output, error=error)
    except subprocess.CalledProcessError as e:
        # Return the error output if the command fails
        return CommandOutput(output="", error=e.stderr.decode('utf-8'), status_code=e.returncode)
    except Exception as e:
        # Return the exception if the command fails
        return CommandOutput(output="", error=str(e), status_code=1)