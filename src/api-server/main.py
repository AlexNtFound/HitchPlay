from fastapi import FastAPI
from fastapi.responses import RedirectResponse
from pydantic import BaseModel
from src.routes.v1.dev import router as dev_router
from src.routes.v1.cmd import router as cmd_router
import os
app = FastAPI()
app.include_router(dev_router, prefix="/v1")
app.include_router(cmd_router)
class DevPayload(BaseModel):
    name: str

@app.get("/healthz", include_in_schema=False)
def healthz():
    return {"ok": True, "env": os.getenv("APP_ENV", "dev")}

@app.get("/")
async def root():
    return RedirectResponse(url="/docs")

if __name__ == "__main__":
    import uvicorn
    import os

    reload_mode = os.getenv("APP_ENV", "dev") == "dev"
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=reload_mode)
