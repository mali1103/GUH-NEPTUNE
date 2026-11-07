"""Simulation: tick loop, comms, async agent decisions, metrics, headless runner."""
import random
import threading
import time
from collections import deque
from itertools import combinations

import agent
from baseline import Hub
from drone import COLORS, Drone
from world import (
    COMMS_RANGE, GRID_SIZE, MAX_TICKS, NUM_DRONES, RENDEZVOUS_POINT,
    SECTOR_LABELS, SECTOR_SIZE, World, dist,
)

LLM_TIMEOUT_FREE = 5.0  # measured ~3s per Haiku decision; 2s fell back every time
LLM_TIMEOUT_WAIT = 8.0


class Sim:
    def __init__(self, seed=42, mode="ours", start="random"):
        self.lock = threading.RLock()
        self.tps = 8
        self.running = False
        self.agent_mode = "rules"
        self.wait_for_agents = True
        self.packet_loss = 0.0
        self.comms_enabled = True
        self.reveal = False
        self.scenario = {}
        self.start = start
        self.reset(seed, mode)

    # ------------------------------------------------------------ setup
    def reset(self, seed=None, mode=None, start=None):
        with self.lock:
            self.seed = self.seed if seed is None else int(seed)
            self.mode = self.mode if mode is None else mode
            self.start = self.start if start is None else start
            self.world = World(self.seed, self.start)
            self.rng = random.Random(self.seed * 31 + 7)
            base = self.mode == "baseline"
            self.drones = [Drone(i, self.world, baseline=base) for i in range(NUM_DRONES)]
            for d in self.drones:
                d.pending_event = "LAUNCH"
            self.hub = Hub(self.world) if base else None
            self.tick = 0
            self.status = "ready"
            self.running = False
            self.links = []
            self.log = deque(maxlen=300)
            self.metrics = {"found_tick": None, "found_by": None, "all_know_tick": None,
                            "msgs_sent": 0, "msgs_dropped": 0}
            self.llm = {"calls": 0, "ok": 0, "fallbacks": 0, "latency": 0.0, "in": 0, "out": 0,
                        "last_error": None}
            self.decision_seq = 0
            if not base:
                for d in self.drones:
                    self.add_log(0, d.id, "plan", "LAUNCH", d.action, d.arg,
                                 f"Pre-launch plan: own sectors {d.my_sectors}, then {d.priority[3:]}.")

    def add_log(self, tick, drone, source, event, action, arg, reason):
        self.log.append({"tick": tick, "drone": drone, "source": source, "event": event,
                         "action": action, "arg": arg, "reason": reason})

    def send_message(self):
        self.metrics["msgs_sent"] += 1
        if self.rng.random() < self.packet_loss:
            self.metrics["msgs_dropped"] += 1
            return False
        return True

    # ------------------------------------------------------------ controls
    def kill(self, i):
        with self.lock:
            d = self.drones[i]
            if d.alive:
                d.alive = False
                d.pending = None
                d.reason = "Destroyed (killed from the control panel)."
                self.add_log(self.tick, i, "sim", "KILLED", "-", None, "Drone lost: no more movement or radio.")

    # ------------------------------------------------------------ tick
    def agents_busy(self):
        return any(d.pending for d in self.drones)

    def step(self):
        with self.lock:
            if self.status in ("complete", "failed"):
                self.running = False
                return
            self.status = "running"
            self.tick += 1
            t = self.tick
            self._scenario(t)

            for d in self.drones:
                if d.move_and_sense(t) == "FOUND" and self.metrics["found_tick"] is None:
                    self.metrics["found_tick"] = t
                    self.metrics["found_by"] = d.id
                    self.add_log(t, d.id, "sensor", "HIKER_FOUND", "-", None,
                                 f"Hiker found at {self.world.hiker}!")

            if self.mode == "ours":
                self._gossip(t)
                for d in self.drones:
                    ev = d.check_event(t)
                    if ev:
                        self._decide(d, t, *ev)
            else:
                for d in self.drones:
                    ev = d.check_event(t)
                    if ev:
                        self._baseline_event(d, t, *ev)
                self.hub.tick(self, t)

            alive = [d for d in self.drones if d.alive]
            if self.metrics["found_tick"] and self.metrics["all_know_tick"] is None \
                    and alive and all(d.hiker_found for d in alive):
                self.metrics["all_know_tick"] = t
            if all((not d.alive) or d.landed for d in self.drones) or t >= MAX_TICKS:
                self.status = "complete" if self.metrics["found_tick"] else "failed"
                self.running = False

    def _scenario(self, t):
        sc = self.scenario
        if sc.get("kill_at") == t:
            self.kill(sc.get("kill_id", 1))
        if sc.get("blackout_at") == t:
            self.comms_enabled = False

    def _gossip(self, t):
        self.links = []
        if not self.comms_enabled:
            return
        alive = [d for d in self.drones if d.alive]
        for a, b in combinations(alive, 2):
            if dist(a.pos, b.pos) > COMMS_RANGE or not self.world.line_of_sight(a.pos, b.pos):
                continue
            ok_ab, ok_ba = self.send_message(), self.send_message()
            ma, mb = a.message(t), b.message(t)
            if ok_ab:
                b.receive(ma, t)
            if ok_ba:
                a.receive(mb, t)
            self.links.append({"a": a.id, "b": b.id, "ok": ok_ab or ok_ba})

    # ------------------------------------------------------------ decisions (ours)
    def _decide(self, d, t, event, arg):
        state = d.build_state(t, event, arg)
        cands, default = agent.build_candidates(state)
        c = cands[default]
        if self.agent_mode == "llm" and agent.llm_available() and event != "LOW_BATTERY":
            timeout = LLM_TIMEOUT_WAIT if self.wait_for_agents else LLM_TIMEOUT_FREE
            if d.pending:  # a newer event supersedes the in-flight decision
                d.pending["future"].cancel()
            self.decision_seq += 1
            d.pending = {"id": self.decision_seq, "start": time.monotonic(), "timeout": timeout,
                         "cands": cands, "default": default, "event": event, "tick": t,
                         "future": agent.submit_llm(state, cands, default, list(d.memory), timeout)}
            self.llm["calls"] += 1
            if not self.wait_for_agents:
                d.apply(c["action"], c["arg"], c["note"], "rules (provisional)", t, event)
        else:
            if d.pending:  # e.g. LOW_BATTERY: drop any slower decision still in flight
                d.pending["future"].cancel()
                d.pending = None
            d.apply(c["action"], c["arg"], c["note"], "rules", t, event)
            self.add_log(t, d.id, "rules", event, c["action"], c["arg"], c["note"])

    def poll_agents(self):
        """Apply finished LLM decisions; fall back to rules on error/timeout."""
        with self.lock:
            for d in self.drones:
                p = d.pending
                if not p:
                    continue
                if not d.alive:
                    d.pending = None
                    continue
                fut, t = p["future"], self.tick
                if fut.done():
                    d.pending = None
                    try:
                        choice, reason, usage, latency = fut.result()
                    except Exception as e:  # noqa: BLE001 - any failure means rules fallback
                        self._fallback(d, p, agent.describe_error(e))
                        continue
                    c = p["cands"][choice]
                    self.llm["ok"] += 1
                    self.llm["latency"] += latency
                    self.llm["in"] += usage["in"]
                    self.llm["out"] += usage["out"]
                    tag = "llm" if choice == p["default"] else "llm (override)"
                    if d.apply(c["action"], c["arg"], reason, tag, t, p["event"]):
                        self.add_log(p["tick"], d.id, tag, p["event"], c["action"], c["arg"], reason)
                elif time.monotonic() - p["start"] > p["timeout"] + 0.5:
                    d.pending = None
                    self._fallback(d, p, "timeout")

    def _fallback(self, d, p, why):
        c = p["cands"][p["default"]]
        self.llm["fallbacks"] += 1
        self.llm["last_error"] = why
        if d.apply(c["action"], c["arg"], c["note"], f"fallback: {why}", self.tick, p["event"]):
            self.add_log(p["tick"], d.id, f"rules (LLM {why})", p["event"], c["action"], c["arg"], c["note"])

    # ------------------------------------------------------------ baseline drones
    def _baseline_event(self, d, t, event, arg):
        if event == "LOW_BATTERY":
            d.needs_orders = False  # don't wait for (or accept) hub orders any more
            d.apply("RETURN_HOME", None, "Low battery: returning home (autonomous safety).", "drone", t, event)
            self.add_log(t, d.id, "drone", event, "RETURN_HOME", None, d.reason)
            return
        if d.action == "SEARCH_SECTION":
            d.last_sector = d.arg
        d.pending_event = event
        d.needs_orders = True
        d.apply("HOVER", None, f"{event}: hovering until the hub sends orders.", "drone", t, event)

    # ------------------------------------------------------------ loop
    def run_forever(self):
        while True:
            if self.running:
                if self.agent_mode == "llm":
                    self.poll_agents()
                if not (self.wait_for_agents and self.agents_busy()):
                    self.step()
                    time.sleep(1.0 / max(1, self.tps))
                else:
                    time.sleep(0.05)
            else:
                if self.agents_busy():
                    self.poll_agents()
                time.sleep(0.05)

    # ------------------------------------------------------------ output
    def summary(self):
        cells = GRID_SIZE * GRID_SIZE
        union, dup = set(), set()
        for d in self.drones:
            dup |= union & d.my_cells
            union |= d.my_cells
        m = dict(self.metrics)
        m["coverage"] = round(len(union) / cells, 3)
        m["duplicated"] = round(len(dup) / max(1, len(union)), 3)
        m["ticks"] = self.tick
        return m, union, dup

    def to_json(self, view=-1):
        with self.lock:
            m, union, dup = self.summary()
            g = GRID_SIZE
            if 0 <= view < len(self.drones):
                known = self.drones[view].searched
                grid = [view if (x, y) in known else -1 for y in range(g) for x in range(g)]
            else:
                owner = {}
                for d in self.drones:
                    for c in d.my_cells:
                        owner.setdefault(c, d.id)
                grid = [(3 if (x, y) in dup else owner.get((x, y), -1)) for y in range(g) for x in range(g)]
            hiker_known = self.metrics["found_tick"] is not None
            llm = dict(self.llm)
            llm["avg_latency"] = round(llm["latency"] / llm["ok"], 2) if llm["ok"] else None
            llm["available"] = agent.llm_available()
            return {
                "tick": self.tick, "status": self.status, "running": self.running, "seed": self.seed,
                "start": self.start,
                "mode": self.mode, "agent_mode": self.agent_mode, "wait_for_agents": self.wait_for_agents,
                "tps": self.tps, "packet_loss": self.packet_loss, "comms_enabled": self.comms_enabled,
                "view": view, "grid": grid, "links": self.links,
                "hiker": self.world.hiker if (hiker_known or self.reveal) else None,
                "hiker_found": hiker_known,
                "drones": [{
                    "id": d.id, "pos": d.pos, "color": COLORS[d.id], "battery": round(d.battery, 3),
                    "alive": d.alive, "landed": d.landed, "action": d.action, "arg": d.arg,
                    "reason": d.reason, "source": d.source, "thinking": bool(d.pending),
                    "knows_found": d.hiker_found,
                    "known_pct": round(len(d.searched) / (g * g), 3),
                    "presumed_lost": [j for j in d.teammates if not d.alive_presumed(j, self.tick)],
                    "best_signal": d.best_signal,
                } for d in self.drones],
                "metrics": m, "llm": llm,
                "log": list(self.log)[-80:],
            }

    def world_json(self):
        return {"size": GRID_SIZE, "heights": [v for row in self.world.height for v in row],
                "sector_size": SECTOR_SIZE, "comms_range": COMMS_RANGE,
                "base": self.world.base, "start": self.start,
                "plan": {i: s for i, s in self.world.own_sectors.items()},
                "rendezvous": RENDEZVOUS_POINT, "sectors": list(SECTOR_LABELS), "seed": self.seed,
                "mode": self.mode}


# ---------------------------------------------------------------- headless
def run_headless(seed, mode="ours", packet_loss=0.0, scenario=None, start="random"):
    sim = Sim(seed, mode, start)
    sim.packet_loss = packet_loss
    sim.scenario = scenario or {}
    while sim.status not in ("complete", "failed"):
        sim.step()
    m, _, _ = sim.summary()
    m["status"] = sim.status
    return m


SCENARIOS = [
    ("0% loss", 0.0, {}),
    ("50% loss", 0.5, {}),
    ("90% loss", 0.9, {}),
    ("blackout @t10", 0.0, {"blackout_at": 10}),
    ("drone 1 killed @t40", 0.0, {"kill_at": 40, "kill_id": 1}),
]


def benchmark(seeds=5, start_seed=1):
    rows = []
    for name, loss, sc in SCENARIOS:
        for mode in ("ours", "baseline"):
            runs = [run_headless(s, mode, loss, sc) for s in range(start_seed, start_seed + seeds)]
            found = [r for r in runs if r["found_tick"]]
            know = [r["all_know_tick"] for r in found if r["all_know_tick"]]
            rows.append({
                "scenario": name, "mode": mode, "runs": len(runs),
                "found": len(found),
                "avg_time_to_find": round(sum(r["found_tick"] for r in found) / len(found), 1) if found else None,
                "avg_all_know": round(sum(know) / len(know), 1) if know else None,
                "avg_coverage": round(sum(r["coverage"] for r in runs) / len(runs), 3),
                "avg_duplicated": round(sum(r["duplicated"] for r in runs) / len(runs), 3),
            })
    return rows
