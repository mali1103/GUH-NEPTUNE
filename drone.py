"""Drone: autopilot, local knowledge, gossip merge and event detection.

A drone only ever acts on its own knowledge. The only ground truth it touches is
its own sensors (beacon receiver, camera, "is the hiker right here") and the
pre-loaded terrain map.

Two search modes:
  * beacon mode (normal): fly 7-wide lanes listening for the beacon.
  * camera mode: the beacon is off or dead, so fly 3-wide lanes looking with the camera.
    Cells cleared by the beacon *while it was still working* stay cleared.
"""
import math
import random
from collections import deque

from world import (
    BATTERY_DRAIN, BATTERY_RESERVE_TICKS, TICK_SECONDS, BEACON_CLEAR_RADIUS, BEACON_MODEL_NOISE, BEACON_RANGE,
    ESTIMATE_DONE_MASS, LOST_TIMEOUT, MISSED_MEETUP_WAIT, RECHARGE_TICKS, RELAY_HOVER_TICKS,
    RENDEZVOUS_INTERVAL, RENDEZVOUS_POINT, SECTOR_LABELS, SECTOR_SIZE, SENSOR_RADIUS,
    SIGNAL_FOLLOW_LIMIT, SIGNAL_THRESHOLD, cells_in_disk, cells_in_radius, cheb, in_bounds,
    sector_bounds, sector_cells, sector_of, sector_rows, step_toward,
)

COLORS = ["#ff6b6b", "#4dabf7", "#ffd43b"]
NEVER = -10**9


def _norm_cdf(x):
    return 0.5 * (1 + math.erf(x / math.sqrt(2)))


class Drone:
    def __init__(self, drone_id, world, baseline=False, solo=False):
        self.id = drone_id
        self.solo = solo                 # no plan, no radio: each drone searches alone
        self.world = world
        self.baseline = baseline
        self.base = world.base
        self.pos = world.base
        self.battery = 1.0
        self.alive = True
        self.landed = False
        self.recharge_left = None        # ticks until relaunch while swapping battery at base
        self.swaps = 0                   # battery swaps so far
        self.min_battery = 1.0           # lowest charge reached
        self.flight_ticks = 0            # ticks spent in the air (energy used)
        self.resume = None               # what it was doing when it turned home for battery

        if solo:  # no team plan: the whole mountain, in this drone's own random order
            order = list(SECTOR_LABELS)
            random.Random(world.seed * 100 + drone_id).shuffle(order)
            self.my_sectors, self.priority = order, list(order)
        else:
            self.my_sectors = list(world.own_sectors[drone_id])
            self.priority = list(world.priority[drone_id])
        self.action = "HOVER" if baseline else "SEARCH_SECTION"
        self.arg = None if baseline else self.my_sectors[0]
        self.reason = "Waiting for the hub." if baseline else "Pre-launch plan: my first sector."
        self.source = "plan"

        # knowledge (all shared by gossip except the observation log)
        self.cleared_at = {}             # cell -> tick it was cleared by the beacon sweep
        self.searched = set()            # = cleared_at keys (kept as a set for fast diffs)
        self.seen_close = set()          # cells seen by the camera
        self.sector_count = {s: 0 for s in SECTOR_LABELS}
        self.my_cells = set()            # ground truth: cells this drone physically swept
        self.best_signal = None
        self.readings = deque(maxlen=6)  # recent positive readings (strength, pos, tick)
        self.obs = deque(maxlen=600)     # my own readings incl. zeros, for estimating
        self.heard = set()               # every real beacon reading the team heard (shared)
        self.camera_mode = False         # beacon presumed off/dead: search by sight
        self.beacon_valid_before = NEVER  # beacon-cleared cells before this tick still count
        self.beacon_note = None          # why we're in camera mode
        self.estimate = None             # probability map after the beacon died
        self.hiker_found = False
        self.hiker_position = None
        self.teammates = {
            j: {"position": world.base, "last_heard": 0, "current_action": "SEARCH_SECTION",
                "current_target": None if solo else world.own_sectors[j][0], "hiker_found": False,
                "missed_at": -1}
            for j in world.own_sectors if j != drone_id
        }
        self.in_contact = set()

        # autopilot bookkeeping
        self.cur_row = None
        self.follow_start = 0
        self.relay_ticks = 0
        self.relay_target = None
        self.rdv_target = None
        self.handled_rdv = None
        self.auto_event = None
        self.found_event_done = False
        self.low_batt_done = False
        self.known_lost = set()
        self.signal_cooldown = -999
        self.needs_orders = baseline

    # ------------------------------------------------------------ knowledge
    def add_cells(self, cells, tick):
        for c in cells:
            if c not in self.searched:
                self.searched.add(c)
                self.cleared_at[c] = tick
                self.sector_count[sector_of(c)] += 1

    def is_done(self, c):
        """Has this cell been ruled out, given what we believe about the beacon?"""
        if not self.camera_mode:
            return c in self.searched
        return c in self.seen_close or self.cleared_at.get(c, 10**9) < self.beacon_valid_before

    def progress(self, s):
        if not self.camera_mode:
            return self.sector_count[s] / (SECTOR_SIZE * SECTOR_SIZE)
        return sum(1 for c in sector_cells(s) if self.is_done(c)) / (SECTOR_SIZE * SECTOR_SIZE)

    @property
    def finished(self):
        return (not self.alive) or (self.landed and self.recharge_left is None)

    def alive_presumed(self, j, tick):
        if self.solo:
            return False  # knows nothing about any teammate
        t = self.teammates[j]
        return tick - t["last_heard"] <= LOST_TIMEOUT and t["missed_at"] <= t["last_heard"]

    def alive_teammates(self, tick):
        return [j for j in self.teammates if self.alive_presumed(j, tick)]

    def next_rendezvous(self, tick):
        if not RENDEZVOUS_INTERVAL:
            return 10**9  # scheduled meetups are off
        return (tick // RENDEZVOUS_INTERVAL + 1) * RENDEZVOUS_INTERVAL

    def enter_camera_mode(self, valid_before, note):
        if self.camera_mode:
            if valid_before >= self.beacon_valid_before:
                return
            self.beacon_valid_before = valid_before  # trust the beacon less, never more
        else:
            self.camera_mode = True
            self.beacon_valid_before = valid_before
        self.beacon_note = note
        self.cur_row = None

    def message(self, tick):
        """Everything this drone knows. Receivers only read it (no copies needed)."""
        return {
            "sender": self.id,
            "searched": self.searched,
            "cleared_at": self.cleared_at,
            "seen_close": self.seen_close,
            "heard": self.heard,
            "camera_mode": (self.beacon_valid_before, self.beacon_note) if self.camera_mode else None,
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
            their = msg["cleared_at"]
            for c in new:
                self.searched.add(c)
                self.cleared_at[c] = their[c]
                self.sector_count[sector_of(c)] += 1
        self.seen_close |= msg["seen_close"]
        self.heard |= msg["heard"]
        if msg["camera_mode"]:
            vb, note = msg["camera_mode"]
            self.enter_camera_mode(vb, f"from drone {msg['sender']}: {note}")
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

    # ------------------------------------------------------------ estimate
    def strong_contact(self):
        """Did this drone hear the beacon clearly enough that silence now means it died?"""
        pos = [o for o in self.obs if o[0] > 0]
        return len(pos) >= 2 and max(o[0] for o in pos) >= 0.2

    def _obs_loglik(self, s, p, c):
        """log P(reading s at p | hiker at c, beacon alive), using the public signal model."""
        mu = self.world.beacon_model(p, c)
        sig = BEACON_MODEL_NOISE
        if s > 0:
            return -0.5 * ((s - mu) / sig) ** 2 - math.log(sig)
        return math.log(max(1e-12, _norm_cdf((SIGNAL_THRESHOLD - mu) / sig)))

    def _candidate_cells(self, obs):
        cands = set()
        for s, p, _ in obs:
            if s > 0:
                cands.update(cells_in_radius(p, BEACON_RANGE + 1))
        return [c for c in cands if c not in self.seen_close]  # camera already looked there

    def beacon_death_evidence(self):
        """Log-likelihood ratio: 'beacon died after the last reading' vs 'still alive'.
        Each hypothesis gets its best-fitting hiker position."""
        t_last = max(o[2] for o in self.obs if o[0] > 0)
        cands = self._candidate_cells(self.obs)
        if not cands:
            return 99.0  # nowhere left it could be: something is wrong with the beacon
        silent_if_dead = math.log(_norm_cdf(SIGNAL_THRESHOLD / BEACON_MODEL_NOISE))
        best_alive = best_dead = -1e18
        for c in cands:
            pre = post = 0.0
            n_post = 0
            for s, p, t in self.obs:
                if t <= t_last:
                    pre += self._obs_loglik(s, p, c)
                else:
                    post += self._obs_loglik(s, p, c)
                    n_post += 1
            best_alive = max(best_alive, pre + post)
            best_dead = max(best_dead, pre + n_post * silent_if_dead)
        return best_dead - best_alive

    def build_estimate(self, tick):
        """Bayesian estimate of where the hiker is: every real reading the team heard, plus
        this drone's own 'heard nothing here' readings nearby, up to the last time anyone
        heard the beacon (after that it may have been dead, so silence proves nothing)."""
        if not self.heard:
            return None
        t_last = max(t for _, _, t in self.heard)
        spots = {p for _, p, _ in self.heard}
        reach = 2 * BEACON_RANGE + 1
        data = {(p, t): s for s, p, t in self.obs
                if t <= t_last and any(cheb(p, q) <= reach for q in spots)}
        data.update({(p, t): s for s, p, t in self.heard})
        obs = [(s, p, t) for (p, t), s in data.items()]
        logp = {c: sum(self._obs_loglik(s, p, c) for s, p, _ in obs)
                for c in self._candidate_cells(obs)}
        if not logp:
            return None
        m = max(logp.values())
        w = {c: math.exp(v - m) for c, v in logp.items()}
        tot = sum(w.values())
        probs = {c: v / tot for c, v in w.items() if v / tot > 1e-4}
        cx = sum(c[0] * p for c, p in probs.items())
        cy = sum(c[1] * p for c, p in probs.items())
        spread = math.sqrt(sum(((c[0] - cx) ** 2 + (c[1] - cy) ** 2) * p for c, p in probs.items()))
        return {"probs": probs, "center": (round(cx), round(cy)), "radius": round(spread, 1),
                "t_last": t_last, "built": tick, "best": max(probs, key=probs.get)}

    def estimate_left(self):
        if not self.estimate:
            return 0.0
        return sum(p for c, p in self.estimate["probs"].items() if c not in self.seen_close)

    def _estimate_target(self):
        if self.estimate_left() < ESTIMATE_DONE_MASS:
            return None
        probs = self.estimate["probs"]
        best = max((c for c in probs if c not in self.seen_close),
                   key=lambda c: (probs[c] / (1 + 0.25 * cheb(self.pos, c)), c))
        return best

    # ------------------------------------------------------------ autopilot
    def _search_target(self, s):
        x0, y0, x1, y1 = sector_bounds(s)
        R = SENSOR_RADIUS if self.camera_mode else BEACON_CLEAR_RADIUS

        def band(r):
            return [(x, y) for y in range(max(y0, r - R), min(y1, r + R + 1))
                    for x in range(x0, x1) if not self.is_done((x, y))]

        rows = sector_rows(s, R)
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
            return None, ("BEACON_LOST" if self.strong_contact() else "SIGNAL_LOST")
        w = sum(s * s for s, _, _ in recent)
        center = (round(sum(s * s * p[0] for s, p, _ in recent) / w),
                  round(sum(s * s * p[1] for s, p, _ in recent) / w))
        # A run of silence after strong contact: is the beacon dead, or are we just behind a hill?
        last = list(self.obs)[-5:]
        if self.strong_contact() and len(last) == 5 and all(o[0] == 0 for o in last) \
                and self.beacon_death_evidence() > 6:
            return None, "BEACON_LOST"
        cands = [c for c in cells_in_radius(center, BEACON_RANGE - 1) if c not in self.seen_close]
        if not cands:
            return None, ("BEACON_LOST" if self.strong_contact() else "SIGNAL_LOST")
        return min(cands, key=lambda c: (2 * cheb(c, center) + cheb(c, self.pos), c)), None

    def _target(self, tick):
        a, arg = self.action, self.arg
        if a == "SEARCH_SECTION":
            t = self._search_target(arg)
            return (t, None) if t else (self.pos, "SECTOR_COMPLETE")
        if a == "FOLLOW_SIGNAL":
            t, ev = self._signal_target(tick)
            return (t, None) if t else (self.pos, ev)
        if a == "SEARCH_LAST_SIGNAL":
            t = self._estimate_target()
            return (t, None) if t else (self.pos, "ESTIMATE_EXHAUSTED")
        if a == "GO_TO_RENDEZVOUS":
            return RENDEZVOUS_POINT, None
        if a == "GO_TO_TEAMMATE":
            tp = self.teammates[arg]["position"]
            told = self.hiker_found and self.teammates[arg]["hiker_found"]
            if cheb(self.pos, tp) <= 1 or arg in self.in_contact or told:
                return self.pos, "ARRIVED"
            return tp, None
        if a == "CLIMB_TO_RELAY":
            if self.relay_target is None:
                self.relay_target = self.world.relay_point(self.pos)
            if self.pos == self.relay_target:
                self.relay_ticks += 1
                if self.relay_ticks >= RELAY_HOVER_TICKS or self.in_contact:
                    return self.pos, "ARRIVED"
                return self.pos, None
            return self.relay_target, None
        if a == "RETURN_HOME":
            return self.base, None
        return self.pos, None  # HOVER

    def move_and_sense(self, tick):
        """Phase 1 of a tick: fly one step, drain battery, sense."""
        if not self.alive:
            self.in_contact = set()
            return
        if self.landed:
            self.in_contact = set()
            if self.recharge_left is not None:      # battery swap at base
                self.recharge_left -= 1
                if self.recharge_left <= 0:
                    self.recharge_left = None
                    self.landed = False
                    self.battery = 1.0
                    self.low_batt_done = False
                    self.auto_event = "RELAUNCHED"
                    self.swaps += 1
            return
        target, ev = self._target(tick)  # uses last tick's contacts
        self.in_contact = set()
        if ev:
            self.auto_event = ev
        cost = 1.0  # one tick of flight (hovering costs the same)
        if target and target != self.pos:
            nxt = step_toward(self.pos, target)
            if in_bounds(nxt):
                cost += self.world.climb_cost(self.pos, nxt)  # going uphill costs extra
                self.pos = nxt
        self.battery = max(0.0, self.battery - cost * BATTERY_DRAIN)
        self.min_battery = min(self.min_battery, self.battery)
        self.flight_ticks += 1
        if self.battery <= 0:
            self.alive = False
            self.reason = "Battery depleted."
            return
        if self.action == "RETURN_HOME" and self.pos == self.base:
            self.landed = True
            if self.low_batt_done and not self.hiker_found:
                self.recharge_left = RECHARGE_TICKS
                self.reason = (f"Swapping battery at base (~{RECHARGE_TICKS * TICK_SECONDS // 60} min), "
                               f"then going back to finish what I was doing.")
        cleared = cells_in_disk(self.pos, BEACON_CLEAR_RADIUS)  # would have heard the beacon here
        self.add_cells(cleared, tick)
        self.my_cells.update(cleared)
        self.seen_close.update(cells_in_radius(self.pos, SENSOR_RADIUS))
        reading = self.world.beacon_reading(self.pos, tick)
        self.obs.append((reading, self.pos, tick))
        if reading > 0:
            self.readings.append((reading, self.pos, tick))
            if self._signal_confirmed(tick):  # two readings within 5 ticks: not a noise spike
                self.heard.update(r for r in self.readings if tick - r[2] < 5)
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

        # whole mountain swept for the beacon and nothing: it must be off. (Checked before the
        # "everything searched, go home" path, and before deciding what to do after a battery
        # swap, so the team re-searches by camera instead of going home.)
        if not self.baseline and not self.camera_mode and not self.hiker_found \
                and not self.low_batt_done and len(self.searched) == len(self.world.height) ** 2:
            if self.heard:   # it was heard once, then went quiet: it died
                t_last = max(t for _, _, t in self.heard)
                self.enter_camera_mode(t_last, f"beacon not heard since tick {t_last}")
                self.estimate = self.build_estimate(tick)
            else:
                self.enter_camera_mode(NEVER, "whole mountain swept with no beacon signal")
            return ("BEACON_SILENT", None)
        if auto == "RELAUNCHED":
            return ("RELAUNCHED", None)
        if self.action == "RETURN_HOME":
            return None
        # the real cost of flying home (climbs included) plus a reserve
        need_home = (self.world.cost_home(self.pos) + BATTERY_RESERVE_TICKS) * BATTERY_DRAIN
        if not self.low_batt_done and self.battery <= need_home:
            self.low_batt_done = True
            if self.action in ("SEARCH_SECTION", "FOLLOW_SIGNAL", "SEARCH_LAST_SIGNAL"):
                self.resume = {"action": self.action, "arg": self.arg, "pos": self.pos,
                               "row": self.cur_row, "tick": tick}
            return ("LOW_BATTERY", None)
        if self.hiker_found and not self.found_event_done:
            self.found_event_done = True
            return ("HIKER_FOUND", None)
        if auto == "BEACON_LOST" and not self.baseline:
            t_last = max(t for _, _, t in self.heard)
            self.enter_camera_mode(t_last, f"beacon went silent after tick {t_last}")
            self.estimate = self.build_estimate(tick)
            return ("BEACON_LOST", t_last)
        if auto:
            if auto == "BEACON_LOST":
                auto = "SIGNAL_LOST"
            return (auto, None)
        if self.baseline:
            if not self.hiker_found and self.action != "FOLLOW_SIGNAL" and self._signal_confirmed(tick) \
                    and tick - self.signal_cooldown > 20:
                self.signal_cooldown = tick
                return ("SIGNAL_DETECTED", None)
            return None
        if self.solo:  # no teammates to meet or worry about
            if not self.hiker_found and self.action not in ("FOLLOW_SIGNAL", "SEARCH_LAST_SIGNAL")                     and self._signal_confirmed(tick) and tick - self.signal_cooldown > 20:
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

        if not self.hiker_found and self.action not in ("FOLLOW_SIGNAL", "SEARCH_LAST_SIGNAL") \
                and self._signal_confirmed(tick) and tick - self.signal_cooldown > 20:
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
        est = None
        if self.estimate:
            est = {"center": self.estimate["center"], "radius": self.estimate["radius"],
                   "beacon_last_heard": self.estimate["t_last"],
                   "probability_left": round(self.estimate_left(), 2)}
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
            # round DOWN: 440/441 must read as 99%, not 100% (or the last cell is never visited)
            "sector_progress": {s: math.floor(self.progress(s) * 100) / 100 for s in SECTOR_LABELS},
            "search_mode": "camera" if self.camera_mode else "beacon",
            "beacon_note": self.beacon_note,
            "estimate": est,
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
            "meetups_on": bool(RENDEZVOUS_INTERVAL),
            "height": self.world.h(self.pos),
            "relay_point": self.world.relay_point(self.pos),
            "relay_height": self.world.h(self.world.relay_point(self.pos)),
            "base_position": self.base,
            "resume": self.resume,
            "plans": {j: ({"own": [], "priority": []} if self.solo else
                          {"own": self.world.own_sectors[j], "priority": self.world.priority[j]})
                      for j in self.world.own_sectors},
            "event": event,
            "event_arg": event_arg,
        }

    def apply(self, action, arg, reason, source, tick, event=None):
        """Carry out a decision. Returns False if refused by the battery safety lock:
        once LOW_BATTERY has fired, nothing (a later event, a hub order) can send the drone
        anywhere but home."""
        if self.low_batt_done and action != "RETURN_HOME":
            return False
        if action == "GO_TO_RENDEZVOUS" and self.action != "GO_TO_RENDEZVOUS":
            self.rdv_target = (self.next_rendezvous(tick)
                               if not RENDEZVOUS_INTERVAL or tick % RENDEZVOUS_INTERVAL else tick)
            if self.handled_rdv is not None and self.handled_rdv > tick:
                self.rdv_target = self.handled_rdv
        if action == "FOLLOW_SIGNAL" and self.action != "FOLLOW_SIGNAL":
            self.follow_start = tick
        if action == "CLIMB_TO_RELAY":
            self.relay_ticks = 0
            self.relay_target = self.world.relay_point(self.pos)
        if action == "SEARCH_SECTION" and arg != self.arg:
            self.cur_row = None
            if self.resume and self.resume["action"] == "SEARCH_SECTION" and self.resume["arg"] == arg:
                self.cur_row = self.resume["row"]  # pick the lawnmower back up on the same row
        if event == "RELAUNCHED":
            self.resume = None
        if action != "FOLLOW_SIGNAL" and self.action == "FOLLOW_SIGNAL":
            self.signal_cooldown = tick
        self.action, self.arg, self.reason, self.source = action, arg, reason, source
        return True
