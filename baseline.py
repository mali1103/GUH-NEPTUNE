"""Naive central-hub baseline, for comparison.

- A base-station hub assigns every drone its next sector (each sector once).
- A drone that needs instructions but can't reach the hub hovers and waits.
- No drone-to-drone sharing, no rendezvous, no takeover of lost drones' sectors.
- Only battery safety is autonomous.
"""
from world import BASE_COMMS_RANGE, BASE_POSITION, SECTOR_LABELS, cheb, dist, sector_center


class Hub:
    def __init__(self, world):
        self.world = world
        self.pos = BASE_POSITION
        self.searched = set()
        self.assigned = {}
        self.found = False
        self.hiker_position = None

    def can_link(self, sim, d):
        return (sim.comms_enabled and d.alive
                and dist(d.pos, self.pos) <= BASE_COMMS_RANGE
                and self.world.line_of_sight(d.pos, self.pos, zb=self.world.mast_altitude(self.pos)))

    def tick(self, sim, t):
        sim.links = []
        for d in sim.drones:
            if not self.can_link(sim, d):
                continue
            up, down = sim.send_message(), sim.send_message()
            sim.links.append({"a": d.id, "b": "hub", "ok": up or down})
            if up:
                self.searched |= d.searched
                if d.hiker_found and not self.found:
                    self.found = True
                    self.hiker_position = d.hiker_position
                    sim.add_log(t, "hub", "hub", "REPORT", "", None,
                                f"Drone {d.id} reported the hiker at {d.hiker_position}.")
            if not down or d.landed or d.low_batt_done:  # low battery: already heading home
                continue
            if self.found and not d.hiker_found:
                d.hiker_found = True
                d.hiker_position = self.hiker_position
                d.found_event_done = True
                d.needs_orders = True
            if self.found and d.action != "RETURN_HOME":
                d.needs_orders = True
            if d.needs_orders:
                action, arg, reason = self.decide(d)
                d.needs_orders = False
                d.apply(action, arg, reason, "hub", t, getattr(d, "pending_event", None))
                sim.add_log(t, d.id, "hub", getattr(d, "pending_event", None), action, arg, reason)

    def decide(self, d):
        ev = getattr(d, "pending_event", None)
        if self.found or d.hiker_found:
            return "RETURN_HOME", None, "Hub: hiker found, return to base."
        if ev == "SIGNAL_DETECTED":
            return "FOLLOW_SIGNAL", None, "Hub: follow that signal."
        free = [s for s in SECTOR_LABELS if s not in self.assigned]
        last = getattr(d, "last_sector", None)
        if last and self.assigned.get(last) == d.id and d.progress(last) < 1:
            return "SEARCH_SECTION", last, f"Hub: resume sector {last}."
        if not free:
            return "RETURN_HOME", None, "Hub: no unassigned sectors left."
        s = min(free, key=lambda s: (cheb(d.pos, sector_center(s)), s))
        self.assigned[s] = d.id
        return "SEARCH_SECTION", s, f"Hub: search sector {s}."
