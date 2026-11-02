import asyncio
import json
import os
from pathlib import Path
from typing import Literal
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
import xml.etree.ElementTree as ET
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

ROOT = Path(__file__).resolve().parents[1]

class Update(BaseModel):
    show_robot_coordinates: bool | None = None
    show_origin: bool | None = None
    mode: Literal["offline", "simulation", "online", "custom"] | None = None
    speed: float | None = Field(default=None, ge=0, le=2, allow_inf_nan=False)

class JointUpdate(BaseModel):
    value: float = Field(allow_inf_nan=False)

class Settings(BaseModel):
    speed: float = Field(ge=0, le=2, allow_inf_nan=False)
    show_origin: bool

def create_app(settings_path=None):
    settings_path = Path(settings_path) if settings_path else ROOT / "work/settings.json"
    app = FastAPI(docs_url=None, redoc_url=None)
    state = dict(show_robot_coordinates=False, show_origin=True, mode="offline", speed=1.0, revision=0, progress=0, processing=False)
    if settings_path.exists():
        state.update(Settings.model_validate_json(settings_path.read_text()).model_dump())
    joint_specs = {}
    for joint in ET.parse(ROOT / "resources/urdf/model.urdf").getroot().findall("joint"):
        kind = joint.get("type")
        if kind not in {"revolute", "continuous", "prismatic"} or joint.find("mimic") is not None:
            continue
        limit = joint.find("limit")
        joint_specs[joint.get("name")] = dict(type=kind,
            lower=float(limit.get("lower", "0")) if limit is not None and kind != "continuous" else None,
            upper=float(limit.get("upper", "0")) if limit is not None and kind != "continuous" else None)
    state["joints"] = {name: 0.0 for name in joint_specs}
    clients = set()
    lock = asyncio.Lock()

    async def publish():
        for socket in tuple(clients):
            try:
                await asyncio.wait_for(socket.send_json(state), 2)
            except Exception:
                clients.discard(socket)

    @app.get("/api/state")
    async def get_state():
        async with lock:
            return dict(state)

    @app.patch("/api/state")
    async def update(value: Update):
        async with lock:
            state.update(value.model_dump(exclude_none=True))
            state["revision"] += 1
            await publish()
            return dict(state)

    @app.put("/api/settings")
    async def save_settings(value: Settings):
        async with lock:
            settings_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = settings_path.with_suffix(".tmp")
            try:
                temporary.write_text(json.dumps(value.model_dump(), indent=2))
                os.replace(temporary, settings_path)
            except OSError as exc:
                raise HTTPException(500, "Could not save settings") from exc
            state.update(value.model_dump())
            state["revision"] += 1
            await publish()
            return dict(state)

    @app.get("/api/joints")
    async def joints():
        return joint_specs

    @app.patch("/api/joints/{name}")
    async def set_joint(name: str, value: JointUpdate):
        spec = joint_specs.get(name)
        if spec is None:
            raise HTTPException(404, "Joint is not manually adjustable")
        if spec["lower"] is not None and not spec["lower"] <= value.value <= spec["upper"]:
            raise HTTPException(422, "Joint limit exceeded")
        async with lock:
            state["joints"][name] = value.value
            state["revision"] += 1
            await publish()
            return dict(state)

    @app.post("/api/process")
    async def process():
        async with lock:
            if state["processing"]:
                return {"accepted": False}
            state.update(processing=True, progress=0)
            state["revision"] += 1
            await publish()
        async def work():
            for step in range(1, 11):
                await asyncio.sleep(.15)
                async with lock:
                    state.update(progress=step * 10, processing=step < 10)
                    state["revision"] += 1
                    await publish()
        asyncio.create_task(work())
        return {"accepted": True}

    @app.websocket("/ws")
    async def websocket(socket: WebSocket):
        await socket.accept()
        async with lock:
            clients.add(socket)
            await socket.send_json(state)
        try:
            while True:
                await socket.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(socket)

    @app.get("/")
    async def index():
        return FileResponse(ROOT / "resource/index.html")

    app.mount("/resource", StaticFiles(directory=ROOT / "resource"), name="resource")
    app.mount("/resources", StaticFiles(directory=ROOT / "resources"), name="models")
    return app

app = create_app()
