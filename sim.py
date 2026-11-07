"""Simulation: tick loop, comms, decisions, metrics, headless runner and benchmarks.

Modes:
  ours      decentralised: pre-launch plan + radio gossip + rendezvous (this project)
  baseline  naive central hub at base; drones hover when they can't reach it
  solo      no coordination at all: no plan, no radio, each drone searches the whole
            mountain in its own random order (shows what teamwork is worth)
"""
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
    SECTOR_LABELS, SECTOR_SIZE, World, cheb, dist,
)

MODES = ["ours", "baseline", "solo"]


class Sim:
    def __init__(self, seed=42, mode="ours", start="random", beacon="on"):
        self.lock = threading.RLock()
        self.tps = 8
        self.running = False
        self.packet_loss = 0.0
        self.comms_enabled = True
        self.reveal = False
        self.scenario = {}
        self.start = start
        self.beacon = beacon
        self.reset(seed, mode)

    # ------------------------------------------------------------ setup
    def reset(self, seed=None, mode=None, start=None, beacon=None):
        with self.lock:
            self.seed = self.seed if seed is None else int(seed)
            self.mode = self.mode if mode is None else mode
            self.start = self.start if start is None else start
            self.beacon = self.beacon if beacon is None else beacon
            self.world = World(self.seed, self.start, self.beacon)
            self.rng = random.Random(self.seed * 31 + 7)
            base, solo = self.mode == "baseline", self.mode == "solo"
            self.drones = [Drone(i, self.world, baseline=base, solo=solo) for i in range(NUM_DRONES)]
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
            self.belief_err_sum = 0.0
            self.belief_err_n = 0
            for d in self.drones:
                if solo:
                    self.add_log(0, d.id, "plan", "LAUNCH", d.action, d.arg,
                                 f"No team plan: searching the whole mountain in my own order {''.join(d.priority)}.")
                elif not base:
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
                d.reason = "Destroyed (killed from the control panel)."
                self.add_log(self.tick, i, "sim", "KILLED", "-", None, "Drone lost: no more movement or radio.")

    # ------------------------------------------------------------ tick
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

            if self.mode == "baseline":
                for d in self.drones:
                    ev = d.check_event(t)
                    if ev:
                        self._baseline_event(d, t, *ev)
                self.hub.tick(self, t)
            else:
                if self.mode == "ours":
                    self._gossip(t)
                else:
                    self.links = []  # solo: no radio at all
                for d in self.drones:
                    ev = d.check_event(t)
                    if ev:
                        self._decide(d, t, *ev)

            self._track_beliefs()
            alive = [d for d in self.drones if d.alive]
            if self.metrics["found_tick"] and self.metrics["all_know_tick"] is None \
                    and alive and all(d.hiker_found for d in alive):
                self.metrics["all_know_tick"] = t
            if all(d.finished for d in self.drones) or t >= MAX_TICKS:
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

    def _track_beliefs(self):
        """How far off is each drone's picture of its teammates? (ground-truth metric)"""
        flying = [d for d in self.drones if d.alive and not d.landed]
        for d in flying:
            for o in flying:
                if o.id != d.id:
                    self.belief_err_sum += cheb(d.teammates[o.id]["position"], o.pos)
                    self.belief_err_n += 1

    # ------------------------------------------------------------ decisions
    def _decide(self, d, t, event, arg):
        """Each drone decides for itself, from its own state only."""
        action, argument, reason = agent.decide(d.build_state(t, event, arg))
        if d.apply(action, argument, reason, "rules", t, event):
            self.add_log(t, d.id, "rules", event, action, argument, reason)

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
                self.step()
                time.sleep(1.0 / max(1, self.tps))
            else:
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
        m["belief_error"] = round(self.belief_err_sum / self.belief_err_n, 1) if self.belief_err_n else 0.0
        m["battery_swaps"] = sum(d.swaps for d in self.drones)
        m["lowest_battery"] = round(min(d.min_battery for d in self.drones), 3)
        m["flight_ticks"] = sum(d.flight_ticks for d in self.drones)
        m["ran_flat"] = sum(1 for d in self.drones if not d.alive and d.battery <= 0)
        m["ticks"] = self.tick
        return m, union, dup

    def to_json(self, view=-1, log_lines=80):
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
            flying = [d for d in self.drones if d.alive and not d.landed]
            now_err = [cheb(d.teammates[o.id]["position"], o.pos) for d in flying for o in flying if o.id != d.id]
            m["belief_error_now"] = round(sum(now_err) / len(now_err), 1) if now_err else 0.0
            return {
                "tick": self.tick, "status": self.status, "running": self.running, "seed": self.seed,
                "start": self.start, "mode": self.mode,
                "tps": self.tps, "packet_loss": self.packet_loss, "comms_enabled": self.comms_enabled,
                "view": view, "grid": grid, "links": self.links,
                "hiker": self.world.hiker if (hiker_known or self.reveal) else None,
                "hiker_found": hiker_known,
                "beacon": {"mode": self.beacon, "alive": self.world.beacon_alive,
                           "died_at": self.world.beacon_died_at},
                "drones": [{
                    "id": d.id, "pos": d.pos, "color": COLORS[d.id], "battery": round(d.battery, 3),
                    "alive": d.alive, "landed": d.landed, "action": d.action, "arg": d.arg,
                    "reason": d.reason, "source": d.source,
                    "knows_found": d.hiker_found,
                    "known_pct": round(len(d.searched) / (g * g), 3),
                    "presumed_lost": [j for j in d.teammates if not d.alive_presumed(j, self.tick)],
                    "best_signal": d.best_signal,
                    "search_mode": "camera" if d.camera_mode else "beacon",
                    "recharging": d.recharge_left,
                    "swaps": d.swaps, "min_battery": round(d.min_battery, 3),
                    "flight_ticks": d.flight_ticks,
                    # what this drone BELIEVES about each teammate (may be stale or wrong)
                    "beliefs": {j: {"pos": t["position"], "action": t["current_action"],
                                    "target": t["current_target"], "age": self.tick - t["last_heard"],
                                    "alive": d.alive_presumed(j, self.tick)}
                                for j, t in d.teammates.items()},
                    "estimate": ({"center": d.estimate["center"], "radius": d.estimate["radius"],
                                  "left": round(d.estimate_left(), 2),
                                  "cells": [[c[0], c[1], round(p, 4)] for c, p in sorted(
                                      d.estimate["probs"].items(), key=lambda kv: -kv[1])[:150]]}
                                 if d.estimate else None),
                } for d in self.drones],
                "metrics": m,
                "log": list(self.log)[-log_lines:],
            }

    def world_json(self):
        return {"size": GRID_SIZE, "heights": [v for row in self.world.height for v in row],
                "sector_size": SECTOR_SIZE, "comms_range": COMMS_RANGE,
                "base": self.world.base, "start": self.start, "beacon": self.beacon,
                "plan": {d.id: d.my_sectors for d in self.drones},
                "rendezvous": RENDEZVOUS_POINT, "sectors": list(SECTOR_LABELS), "seed": self.seed,
                "mode": self.mode}


# ---------------------------------------------------------------- side by side
class Compare:
    """The three modes on the same map, stepped in lockstep. Every control (packet loss,
    blackout, killing a drone) hits all three at the same tick, so it's a fair race."""

    def __init__(self, seed=2, start="random", beacon="on"):
        self.lock = threading.RLock()
        self.tps = 8
        self.running = False
        self.sims = [Sim(seed, m, start, beacon) for m in MODES]

    def reset(self, seed=None, start=None, beacon=None):
        with self.lock:
            self.running = False
            for s in self.sims:
                s.reset(seed, None, start, beacon)

    def set(self, **attrs):
        with self.lock:
            for s in self.sims:
                for k, v in attrs.items():
                    setattr(s, k, v)

    def kill(self, i):
        with self.lock:
            for s in self.sims:
                s.kill(i)

    def done(self):
        return all(s.status in ("complete", "failed") for s in self.sims)

    def step(self):
        with self.lock:
            for s in self.sims:
                s.step()
            if self.done():
                self.running = False

    def run_forever(self):
        while True:
            if self.running:
                self.step()
                time.sleep(1.0 / max(1, self.tps))
            else:
                time.sleep(0.05)

    def to_json(self):
        with self.lock:
            sims = [s.to_json(-1, log_lines=4) for s in self.sims]
            found = [(s["metrics"]["found_tick"], i) for i, s in enumerate(sims) if s["metrics"]["found_tick"]]
            return {"running": self.running, "tps": self.tps, "done": self.done(),
                    "first": min(found)[1] if found else None, "sims": sims}

    def world_json(self):
        with self.lock:
            w = self.sims[0].world_json()
            w["plans"] = [s.world_json()["plan"] for s in self.sims]
            return w


# ---------------------------------------------------------------- headless
def run_headless(seed, mode="ours", packet_loss=0.0, scenario=None, start="random", beacon="on"):
    sim = Sim(seed, mode, start, beacon)
    sim.packet_loss = packet_loss
    sim.scenario = scenario or {}
    while sim.status not in ("complete", "failed"):
        sim.step()
    m, _, _ = sim.summary()
    m["status"] = sim.status
    return m


def _aggregate(runs):
    found = [r for r in runs if r["found_tick"]]
    know = [r["all_know_tick"] for r in found if r["all_know_tick"]]
    n = len(runs)
    return {
        "runs": n, "found": len(found),
        "avg_time_to_find": round(sum(r["found_tick"] for r in found) / len(found), 1) if found else None,
        "avg_all_know": round(sum(know) / len(know), 1) if know else None,
        "avg_coverage": round(sum(r["coverage"] for r in runs) / n, 3),
        "avg_duplicated": round(sum(r["duplicated"] for r in runs) / n, 3),
        "avg_belief_error": round(sum(r["belief_error"] for r in runs) / n, 1),
        "avg_swaps": round(sum(r["battery_swaps"] for r in runs) / n, 1),
        "avg_lowest_battery": round(sum(r["lowest_battery"] for r in runs) / n, 3),
        "worst_lowest_battery": min(r["lowest_battery"] for r in runs),
        "avg_flight_ticks": round(sum(r["flight_ticks"] for r in runs) / n),
        "ran_flat": sum(r["ran_flat"] for r in runs),
    }


SCENARIOS = [
    ("0% loss", 0.0, {}),
    ("50% loss", 0.5, {}),
    ("90% loss", 0.9, {}),
    ("blackout @t10", 0.0, {"blackout_at": 10}),
    ("drone 1 killed @t40", 0.0, {"kill_at": 40, "kill_id": 1}),
    ("beacon dies on contact", 0.0, {"beacon": "dies"}),
    ("beacon off (no signal)", 0.0, {"beacon": "off"}),
]


def benchmark(seeds=5, start_seed=1):
    """Failure scenarios x modes."""
    rows = []
    for name, loss, sc in SCENARIOS:
        sc = dict(sc)
        beacon = sc.pop("beacon", "on")
        for mode in MODES:
            runs = [run_headless(s, mode, loss, sc, beacon=beacon) for s in range(start_seed, start_seed + seeds)]
            rows.append({"scenario": name, "mode": mode, **_aggregate(runs)})
    return rows


LOSS_LEVELS = [0.0, 0.25, 0.5, 0.75, 0.9, 1.0]


def resilience_curve(seeds=5, start_seed=1):
    """Found rate and time to find vs packet loss, per mode. Solo uses no radio, so it is
    flat; 'ours' at 100% loss is the pre-launch plan alone (no messages get through)."""
    seed_range = range(start_seed, start_seed + seeds)
    curve = {}
    for mode in MODES:
        if mode == "solo":
            agg = _aggregate([run_headless(s, "solo") for s in seed_range])
            curve[mode] = [{"loss": x, **agg} for x in LOSS_LEVELS]
        else:
            curve[mode] = [{"loss": x, **_aggregate([run_headless(s, mode, x) for s in seed_range])}
                           for x in LOSS_LEVELS]
    return curve


def coordination_breakdown(seeds=5, start_seed=1):
    """What each layer of teamwork is worth (radio otherwise perfect)."""
    seed_range = range(start_seed, start_seed + seeds)
    rows = [
        ("No coordination (each drone alone)", [run_headless(s, "solo") for s in seed_range]),
        ("Pre-launch plan only (radio dead)", [run_headless(s, "ours", 1.0) for s in seed_range]),
        ("Plan + radio sharing (ours)", [run_headless(s, "ours") for s in seed_range]),
        ("Central hub (baseline)", [run_headless(s, "baseline") for s in seed_range]),
    ]
    return [{"setup": name, **_aggregate(runs)} for name, runs in rows]
