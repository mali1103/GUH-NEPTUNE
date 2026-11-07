"""FastAPI server: serves the page, /state and /world JSON, control endpoints, benchmarks.

    uvicorn server:app --reload
"""
import json
import threading
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Body, FastAPI
from fastapi.responses import FileResponse

from bench import run_all
from sim import MODES, Compare, Sim

ROOT = Path(__file__).parent
STATIC = ROOT / "static"
BENCH_FILE = ROOT / "bench_results.json"

sim = Sim(seed=2, mode="ours", start="random")  # demo: base top-left, hiker in drone 1's far sector
compare = Compare(seed=23, start="random")       # the three modes side by side (/compare); seed 23 is typical


@asynccontextmanager
async def lifespan(_app):
    threading.Thread(target=sim.run_forever, daemon=True, name="sim-loop").start()
    threading.Thread(target=compare.run_forever, daemon=True, name="compare-loop").start()
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
        sim.step()
    return {"ok": True}


@app.post("/control/reset")
def reset(body: dict = Body(default={})):
    mode = body.get("mode")
    sim.reset(body.get("seed"), mode if mode in MODES else None, body.get("start"), body.get("beacon"))
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


@app.post("/control/reveal")
def reveal(body: dict = Body(...)):
    sim.reveal = bool(body.get("enabled", False))
    return {"ok": True}


@app.get("/bench")
def bench_cached():
    """Last saved benchmark (python bench.py writes it), so the demo shows results instantly."""
    if BENCH_FILE.exists():
        return json.loads(BENCH_FILE.read_text(encoding="utf-8"))
    return {"missing": True}


@app.post("/bench")
def bench_run(body: dict = Body(default={})):
    seeds = max(1, min(20, int(body.get("seeds", 5))))
    results = run_all(seeds)
    BENCH_FILE.write_text(json.dumps(results, indent=1), encoding="utf-8")
    return results


# ---------------------------------------------------------------- side-by-side comparison
@app.get("/compare")
def compare_page():
    return FileResponse(STATIC / "compare.html")


@app.get("/compare/world")
def compare_world():
    return compare.world_json()


@app.get("/compare/state")
def compare_state():
    return compare.to_json()


@app.post("/compare/control/{what}")
def compare_control(what: str, body: dict = Body(default={})):
    if what == "start":
        if not compare.done():
            compare.running = True
    elif what == "pause":
        compare.running = False
    elif what == "step":
        compare.running = False
        compare.step()
    elif what == "reset":
        compare.reset(body.get("seed"), body.get("start"), body.get("beacon"))
    elif what == "speed":
        compare.tps = max(1, min(60, int(body.get("ticks_per_second", 8))))
    elif what == "packet_loss":
        compare.set(packet_loss=max(0.0, min(1.0, float(body.get("value", 0)))))
    elif what == "comms":
        compare.set(comms_enabled=bool(body.get("enabled", True)))
    elif what == "kill":
        compare.kill(int(body["drone_id"]))
    elif what == "reveal":
        compare.set(reveal=bool(body.get("enabled", False)))
    else:
        return {"ok": False, "error": f"unknown control {what}"}
    return {"ok": True}
