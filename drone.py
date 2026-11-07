"""Drone: autopilot, local knowledge, gossip merge and event detection.

A drone only ever acts on its own knowledge. The only ground truth it touches is
its own sensor (cells under it, beacon reading, "is the hiker right here") and
the pre-loaded terrain map.
"""
from collections import deque

from world import (
    BASE_POSITION, BATTERY_DRAIN, BATTERY_RESERVE_TICKS, BEACON_RANGE, LOST_TIMEOUT,
    MISSED_MEETUP_WAIT, OWN_SECTORS, RELAY_HOVER_TICKS, RENDEZVOUS_INTERVAL,
    RENDEZVOUS_POINT, SECTOR_LABELS, SECTOR_SIZE, SENSOR_RADIUS, SIGNAL_FOLLOW_LIMIT,
    cells_in_radius, cheb, in_bounds, priority_order, sector_bounds, sector_of,
    sector_rows, step_toward,
)

COLORS = ["#ff6b6b", "#4dabf7", "#ffd43b"]


class Drone:
    def __init__(self, drone_id, world, baseline=False):
        self.id = drone_id
        self.world = world
        self.baseline = baseline
        self.pos = BASE_POSITION
        self.battery = 1.0
        self.alive = True
        self.landed = False

        self.my_sectors = list(OWN_SECTORS[drone_id])
        self.priority = priority_order(drone_id)
        self.action = "HOVER" if baseline else "SEARCH_SECTION"
        self.arg = None if baseline else self.my_sectors[0]
        self.reason = "Waiting for the hub." if baseline else "Pre-launch plan: my first sector."
        self.source = "plan"

        # knowledge
        self.searched = set()
        self.sector_count = {s: 0 for s in SECTOR_LABELS}
        self.my_cells = set()            # ground truth: cells this drone physically swept
        self.best_signal = None
        self.readings = deque(maxlen=6)
        self.hiker_found = False
        self.hiker_position = None
        self.teammates = {
            j: {"position": BASE_POSITION, "last_heard": 0, "current_action": "SEARCH_SECTION",
                "current_target": OWN_SECTORS[j][0], "hiker_found": False, "missed_at": -1}
            for j in OWN_SECTORS if j != drone_id
        }
        self.in_contact = set()

        # autopilot bookkeeping
        self.cur_row = None
        self.follow_start = 0
        self.relay_ticks = 0
        self.rdv_target = None           # the scheduled rendezvous tick we are heading to
        self.handled_rdv = None
        self.auto_event = None
        self.found_event_done = False
        self.low_batt_done = False
        self.known_lost = set()
        self.signal_cooldown = -999
        self.memory = deque(maxlen=5)
        self.pending = None              # in-flight LLM decision
        self.needs_orders = baseline     # baseline: waiting for the hub

    # ------------------------------------------------------------ knowledge
    def add_cells(self, cells):
        for c in cells:
            if c not in self.searched:
                self.searched.add(c)
                self.sector_count[sector_of(c)] += 1

    def progress(self, s):
        return self.sector_count[s] / (SECTOR_SIZE * SECTOR_SIZE)

    def alive_presumed(self, j, tick):
        t = self.teammates[j]
        return tick - t["last_heard"] <= LOST_TIMEOUT and t["missed_at"] <= t["last_heard"]

    def alive_teammates(self, tick):
        return [j for j in self.teammates if self.alive_presumed(j, tick)]

    def next_rendezvous(self, tick):
        return (tick // RENDEZVOUS_INTERVAL + 1) * RENDEZVOUS_INTERVAL

    def message(self, tick):
        """Everything this drone knows. Receivers only read it (no copies needed)."""
        return {
            "sender": self.id,
            "searched": self.searched,
            "best_signal": self.best_signal,
            "hiker_found": self.hiker_found,
            "hiker_position": self.hiker_position,
            "self_info": {"position": self.pos, "last_heard": tick, "current_action": self.action,
                          "current_target": self.arg, "hiker_found": self.hiker_found, "missed_at": -1},
            "teammates": self.teammates,
        }

    def receive(self, msg, tick):
        new = msg["searched"] - self.searched
        if new:
            self.add_cells(new)
        bs = msg["best_signal"]
        if bs and (self.best_signal is None or bs["strength"] > self.best_signal["strength"]):
            self.best_signal = dict(bs)
        if msg["hiker_found"] and not self.hiker_found:
            self.hiker_found = True
            self.hiker_position = msg["hiker_position"]
        infos = {msg["sender"]: msg["self_info"]}
        infos.update(msg["teammates"])
        for j, info in infos.items():
            if j == self.id:
                continue
            mine = self.teammates[j]
            missed = max(mine["missed_at"], info["missed_at"])
            if info["last_heard"] > mine["last_heard"]:
                mine.update({k: info[k] for k in ("position", "last_heard", "current_action",
                                                   "current_target", "hiker_found")})
            mine["missed_at"] = missed
            if info["hiker_found"]:
                mine["hiker_found"] = True
        self.in_contact.add(msg["sender"])

    # ------------------------------------------------------------ autopilot
    def _search_target(self, s):
        x0, y0, x1, y1 = sector_bounds(s)
        R = SENSOR_RADIUS

        def band(r):
            return [(x, y) for y in range(max(y0, r - R), min(y1, r + R + 1))
                    for x in range(x0, x1) if (x, y) not in self.searched]

        rows = sector_rows(s)
        todo = None
        if self.cur_row in rows and band(self.cur_row):
            todo = self.cur_row
        else:
            open_rows = [r for r in rows if band(r)]
            if not open_rows:
                return None
            todo = min(open_rows, key=lambda r: (abs(r - self.pos[1]), r))
            self.cur_row = todo
        cols = sorted({c[0] for c in band(todo)})
        tx = min(cols, key=lambda x: (abs(x - self.pos[0]), x))
        return (tx, todo)

    def _signal_target(self, tick):
        recent = [r for r in self.readings if tick - r[2] <= 30]
        if not recent and self.best_signal:
            recent = [(self.best_signal["strength"], tuple(self.best_signal["position"]), tick)]
        if not recent or tick - self.follow_start > SIGNAL_FOLLOW_LIMIT:
            return None
        w = sum(s * s for s, _, _ in recent)
        cx = round(sum(s * s * p[0] for s, p, _ in recent) / w)
        cy = round(sum(s * s * p[1] for s, p, _ in recent) / w)
        center = (cx, cy)
        r = BEACON_RANGE - 1
        cands = [c for c in cells_in_radius(center, r) if c not in self.searched]
        if not cands:
            return None
        return min(cands, key=lambda c: (2 * cheb(c, center) + cheb(c, self.pos), c))

    def _target(self, tick):
        a, arg = self.action, self.arg
        if a == "SEARCH_SECTION":
            t = self._search_target(arg)
            return (t, None) if t else (self.pos, "SECTOR_COMPLETE")
        if a == "FOLLOW_SIGNAL":
            t = self._signal_target(tick)
            return (t, None) if t else (self.pos, "SIGNAL_LOST")
        if a == "GO_TO_RENDEZVOUS":
            return RENDEZVOUS_POINT, None
        if a == "GO_TO_TEAMMATE":
            tp = self.teammates[arg]["position"]
            told = self.hiker_found and self.teammates[arg]["hiker_found"]
            if cheb(self.pos, tp) <= 1 or arg in self.in_contact or told:
                return self.pos, "ARRIVED"
            return tp, None
        if a == "CLIMB_TO_RELAY":
            ridge = self.world.nearest_ridge(self.pos)
            if ridge is None:
                return self.pos, "ARRIVED"
            if self.pos == ridge or self.world.is_ridge(self.pos):
                self.relay_ticks += 1
                if self.relay_ticks >= RELAY_HOVER_TICKS:
                    return self.pos, "ARRIVED"
                return self.pos, None
            return ridge, None
        if a == "RETURN_HOME":
            return BASE_POSITION, None
        return self.pos, None  # HOVER

    def move_and_sense(self, tick):
        """Phase 1 of a tick: fly one step, drain battery, sense."""
        if not self.alive or self.landed:
            self.in_contact = set()
            return
        target, ev = self._target(tick)  # uses last tick's contacts
        self.in_contact = set()
        if ev:
            self.auto_event = ev
        if target and target != self.pos:
            nxt = step_toward(self.pos, target)
            if in_bounds(nxt):
                self.pos = nxt
        self.battery = max(0.0, self.battery - BATTERY_DRAIN)
        if self.battery <= 0:
            self.alive = False
            self.reason = "Battery depleted."
            return
        if self.action == "RETURN_HOME" and self.pos == BASE_POSITION:
            self.landed = True
        cells = cells_in_radius(self.pos, SENSOR_RADIUS)
        self.add_cells(cells)
        self.my_cells.update(cells)
        reading = self.world.beacon_reading(self.pos)
        if reading > 0:
            self.readings.append((reading, self.pos, tick))
            if self.best_signal is None or reading > self.best_signal["strength"]:
                self.best_signal = {"strength": reading, "position": self.pos, "tick": tick}
        if self.world.hiker_within(self.pos) and not self.hiker_found:
            self.hiker_found = True
            self.hiker_position = self.world.hiker
            return "FOUND"

    # ------------------------------------------------------------ events
    def _signal_confirmed(self, tick):
        """Debounce noise: need a reading this tick and another within the last 5 ticks."""
        recent = [r for r in self.readings if tick - r[2] < 5]
        return len(recent) >= 2 and recent[-1][2] == tick

    def check_event(self, tick):
        """Phase 3 of a tick (after gossip): return (event, arg) or None."""
        if not self.alive or self.landed:
            return None
        auto, self.auto_event = self.auto_event, None
        for j in self.in_contact:
            self.teammates[j]["last_heard"] = tick

        if self.action == "RETURN_HOME":
            return None
        need_home = (cheb(self.pos, BASE_POSITION) + BATTERY_RESERVE_TICKS) * BATTERY_DRAIN
        if not self.low_batt_done and self.battery <= need_home:
            self.low_batt_done = True
            return ("LOW_BATTERY", None)
        if self.hiker_found and not self.found_event_done:
            self.found_event_done = True
            return ("HIKER_FOUND", None)
        if auto:
            return (auto, None)
        if self.baseline:
            if not self.hiker_found and self.action != "FOLLOW_SIGNAL" and self._signal_confirmed(tick) \
                    and tick - self.signal_cooldown > 20:
                self.signal_cooldown = tick
                return ("SIGNAL_DETECTED", None)
            return None

        # rendezvous waiting
        if self.action == "GO_TO_RENDEZVOUS" and cheb(self.pos, RENDEZVOUS_POINT) <= 1:
            alive = self.alive_teammates(tick)
            missing = [j for j in alive if j not in self.in_contact]
            if not missing:
                self.rdv_target = None
                return ("MEETUP_COMPLETE", None)
            if self.rdv_target is not None and tick >= self.rdv_target + MISSED_MEETUP_WAIT:
                for j in missing:
                    self.teammates[j]["missed_at"] = tick
                    self.known_lost.add(j)
                self.rdv_target = None
                return ("MISSED_MEETUP", missing)

        if not self.hiker_found and self.action != "FOLLOW_SIGNAL" and self._signal_confirmed(tick) \
                and tick - self.signal_cooldown > 20:
            self.signal_cooldown = tick
            return ("SIGNAL_DETECTED", None)

        nr = self.next_rendezvous(tick)
        if not self.hiker_found and self.handled_rdv != nr and self.action != "GO_TO_RENDEZVOUS" \
                and tick >= nr - cheb(self.pos, RENDEZVOUS_POINT) - 2:
            self.handled_rdv = nr
            return ("RENDEZVOUS_DUE", None)

        for j in self.teammates:
            lost = not self.alive_presumed(j, tick)
            if lost and j not in self.known_lost:
                self.known_lost.add(j)
                return ("LOST_CONTACT", j)
            if not lost and j in self.known_lost:
                self.known_lost.discard(j)
        return None

    # ------------------------------------------------------------ decisions
    def build_state(self, tick, event=None, event_arg=None):
        return {
            "id": self.id,
            "tick": tick,
            "position": self.pos,
            "battery": round(self.battery, 3),
            "current_action": self.action,
            "current_target": self.arg,
            "my_sectors": self.my_sectors,
            "priority_order": self.priority,
            "searched_cells": self.searched,
            "sector_progress": {s: round(self.progress(s), 2) for s in SECTOR_LABELS},
            "best_signal": self.best_signal,
            "recent_signal": bool(self.readings and tick - self.readings[-1][2] <= 30),
            "hiker_found": self.hiker_found,
            "hiker_position": self.hiker_position,
            "teammates": {
                j: {"position": t["position"], "last_heard": t["last_heard"],
                    "current_action": t["current_action"], "current_target": t["current_target"],
                    "alive_presumed": self.alive_presumed(j, tick),
                    "knows_hiker_found": t["hiker_found"]}
                for j, t in self.teammates.items()
            },
            "in_contact_with": sorted(self.in_contact),
            "next_rendezvous_tick": self.next_rendezvous(tick),
            "rendezvous_point": RENDEZVOUS_POINT,
            "nearest_ridge": self.world.nearest_ridge(self.pos),
            "base_position": BASE_POSITION,
            "event": event,
            "event_arg": event_arg,
        }

    def apply(self, action, arg, reason, source, tick, event=None):
        if action == "GO_TO_RENDEZVOUS" and self.action != "GO_TO_RENDEZVOUS":
            self.rdv_target = self.next_rendezvous(tick) if tick % RENDEZVOUS_INTERVAL else tick
            if self.handled_rdv is not None and self.handled_rdv > tick:
                self.rdv_target = self.handled_rdv
        if action == "FOLLOW_SIGNAL" and self.action != "FOLLOW_SIGNAL":
            self.follow_start = tick
        if action == "CLIMB_TO_RELAY":
            self.relay_ticks = 0
        if action == "SEARCH_SECTION" and arg != self.arg:
            self.cur_row = None
        if action != "FOLLOW_SIGNAL" and self.action == "FOLLOW_SIGNAL":
            self.signal_cooldown = tick
        self.action, self.arg, self.reason, self.source = action, arg, reason, source
        self.memory.append({"tick": tick, "event": event, "action": action, "arg": arg,
                            "reason": reason[:100]})
