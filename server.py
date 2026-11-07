"""FastAPI server: serves the page, /state and /world JSON, control endpoints.

    uvicorn server:app --reload
"""
import threading
from contextlib import asynccontextmanager
from pathlib import Path

from dotenv import load_dotenv

load_dotenv()

from fastapi import Body, FastAPI  # noqa: E402
from fastapi.responses import FileResponse  # noqa: E402

from sim import Sim, benchmark  # noqa: E402

sim = Sim(seed=22, mode="ours")  # demo seed: hiker in sector B, drone 1's last sector
STATIC = Path(__file__).parent / "static"


@asynccontextmanager
async def lifespan(_app):
    threading.Thread(target=sim.run_forever, daemon=True, name="sim-loop").start()
    yield


app = FastAPI(title="Mountain Rescue Drones", lifespan=lifespan)


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/world")
def world():
    with sim.lock:
        return sim.world_json()


@app.get("/state")
def state(view: int = -1):
    return sim.to_json(view)


@app.post("/control/start")
def start():
    with sim.lock:
        if sim.status not in ("complete", "failed"):
            sim.running = True
    return {"ok": True}


@app.post("/control/pause")
def pause():
    sim.running = False
    return {"ok": True}


@app.post("/control/step")
def step():
    with sim.lock:
        sim.running = False
        if not sim.agents_busy():
            sim.step()
    return {"ok": True}


@app.post("/control/reset")
def reset(body: dict = Body(default={})):
    sim.reset(body.get("seed"), body.get("mode"))
    return {"ok": True}


@app.post("/control/speed")
def speed(body: dict = Body(...)):
    sim.tps = max(1, min(60, int(body.get("ticks_per_second", 8))))
    return {"ok": True}


@app.post("/control/packet_loss")
def packet_loss(body: dict = Body(...)):
    sim.packet_loss = max(0.0, min(1.0, float(body.get("value", 0))))
    return {"ok": True}


@app.post("/control/comms")
def comms(body: dict = Body(...)):
    sim.comms_enabled = bool(body.get("enabled", True))
    return {"ok": True}


@app.post("/control/kill")
def kill(body: dict = Body(...)):
    sim.kill(int(body["drone_id"]))
    return {"ok": True}


@app.post("/control/agent")
def agent_mode(body: dict = Body(...)):
    with sim.lock:
        if body.get("mode") in ("rules", "llm"):
            sim.agent_mode = body["mode"]
        if "wait" in body:
            sim.wait_for_agents = bool(body["wait"])
    return {"ok": True}


@app.post("/control/reveal")
def reveal(body: dict = Body(...)):
    sim.reveal = bool(body.get("enabled", False))
    return {"ok": True}


@app.post("/bench")
def bench(body: dict = Body(default={})):
    seeds = max(1, min(20, int(body.get("seeds", 5))))
    return {"rows": benchmark(seeds)}
