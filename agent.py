"""Per-drone decision agent.

    decide(state) -> (action, argument, reason)

Each drone runs its own copy and calls it on events (sector done, signal heard, teammate
lost...). It is a pure function of that one drone's `state`: it never sees the true world
or another drone's internals, only what this drone has sensed or heard over the radio.
Rules-based: instant, deterministic, explainable, and needs no network.
"""
from world import RENDEZVOUS_SKIP_IF_HEARD, SECTOR_LABELS, cheb, sector_center

ACTIONS = ["SEARCH_SECTION", "FOLLOW_SIGNAL", "SEARCH_LAST_SIGNAL", "GO_TO_RENDEZVOUS",
           "GO_TO_TEAMMATE", "CLIMB_TO_RELAY", "RETURN_HOME"]


# ====================================================================== rules
def _claims(state, exclude=()):
    """Sectors live teammates are searching now, or will reach next in their own plan."""
    prog = state["sector_progress"]
    now, plan = {}, {}
    for j, t in state["teammates"].items():
        if not t["alive_presumed"] or j in exclude:
            continue
        tgt = t["current_target"]
        if t["current_action"] == "SEARCH_SECTION" and tgt in prog and prog[tgt] < 1:
            now.setdefault(tgt, j)
        own = state["plans"][j]["own"]
        start = own.index(tgt) + 1 if tgt in own else 0
        for s in own[start:]:
            if prog[s] < 1:
                plan.setdefault(s, j)
    return now, plan


def _lost_owner(state, s):
    for j, t in state["teammates"].items():
        if s in state["plans"][j]["own"] and not t["alive_presumed"]:
            return j
    return None


def _sector_note(state, s, now, plan):
    pct = int(state["sector_progress"][s] * 100)
    me = state["id"]
    if s in now and now[s] != me:
        status = f"drone {now[s]} is searching it now"
    elif s in plan:
        status = f"next in drone {plan[s]}'s plan"
    elif s in state["my_sectors"]:
        status = "my own sector"
    else:
        lost = _lost_owner(state, s)
        status = f"unclaimed, owner drone {lost} presumed lost" if lost is not None else "unclaimed"
    return f"{pct}% searched, {status}"


def _rank_sectors(state):
    """All unfinished sectors, best first, by this drone's priority order and claims."""
    me, prog = state["id"], state["sector_progress"]
    now, plan = _claims(state)
    order = state["priority_order"]
    cur = state["current_target"] if state["current_action"] == "SEARCH_SECTION" else None
    scored = []
    for s in SECTOR_LABELS:
        if prog[s] >= 1:
            continue
        score = order.index(s) + cheb(state["position"], sector_center(s)) / 10
        if s in now:
            if s == cur and me < now[s]:
                pass  # tie: both on it, lower id keeps it
            else:
                score += 100
        elif s in plan:
            score += 40
        if s == cur and (s not in now or me < now[s]):
            score -= 50  # stick with my sector
        scored.append((score, s))
    scored.sort()
    return [s for _, s in scored], now, plan


def _agreed_split(state):
    """Deterministic re-split at a meetup: everyone in contact runs this same code on the
    same merged knowledge, in id order, so they all reach the same assignment."""
    me, prog = state["id"], state["sector_progress"]
    alive = [j for j, t in state["teammates"].items() if t["alive_presumed"]]
    group = sorted({me, *[j for j in state["in_contact_with"] if j in alive]})
    absent_now, absent_plan = _claims(state, exclude=group)
    taken = {}
    for j in group:
        order = state["plans"][j]["priority"]
        options = [s for s in SECTOR_LABELS if prog[s] < 1 and s not in taken]
        if not options:
            break
        best = min(options, key=lambda s: (order.index(s) + 100 * (s in absent_now)
                                           + 40 * (s in absent_plan), s))
        taken[best] = j
        if j == me:
            return best, group
    return None, group


def _keep(state, why):
    return state["current_action"], state["current_target"], why


def decide_default(state):
    ev = state["event"]
    me = state["id"]
    pos = state["position"]
    rp = tuple(state["rendezvous_point"])
    mates = state["teammates"]
    alive = [j for j, t in mates.items() if t["alive_presumed"]]
    contact = set(state["in_contact_with"])

    if ev == "LOW_BATTERY":
        return "RETURN_HOME", None, f"Battery at {int(state['battery'] * 100)}% only just covers the trip home."

    if state["hiker_found"]:
        unaware = [j for j in alive if not mates[j]["knows_hiker_found"] and j not in contact]
        if not unaware:
            return "RETURN_HOME", None, "Hiker found and every live teammate knows; returning to base."
        j = min(unaware, key=lambda k: (cheb(pos, tuple(mates[k]["position"])), k))
        tp = tuple(mates[j]["position"])
        if not state.get("meetups_on", True):
            # No scheduled meetups: base is where everyone comes back to swap batteries,
            # so landing there means every teammate hears the news on its next visit.
            home = tuple(state["base_position"])
            if ev == "ARRIVED" and state["current_action"] == "GO_TO_TEAMMATE":
                return "RETURN_HOME", None, f"Drone {j} wasn't at its last known spot; landing at base, where it will come to swap batteries."
            if cheb(pos, tp) < cheb(pos, home):
                return "GO_TO_TEAMMATE", j, f"Hiker found; drone {j} doesn't know and its last position {tp} is closer than base."
            return "RETURN_HOME", None, f"Hiker found; landing at base to tell drone(s) {unaware} when they come back to swap batteries."
        if state["current_action"] == "GO_TO_RENDEZVOUS" and ev not in ("ARRIVED",):
            return _keep(state, f"Waiting at the rendezvous to tell drone(s) {unaware} about the hiker.")
        if ev == "ARRIVED" and state["current_action"] == "GO_TO_TEAMMATE":
            return "GO_TO_RENDEZVOUS", None, f"Drone {j} wasn't at its last known spot; the rendezvous is where we'll meet."
        if cheb(pos, tp) <= cheb(pos, rp):
            return "GO_TO_TEAMMATE", j, f"Hiker found; drone {j} doesn't know and its last position {tp} is closer than the rendezvous."
        return "GO_TO_RENDEZVOUS", None, f"Hiker found; the rendezvous is closer than drone {j}'s last known position."

    if ev == "SIGNAL_DETECTED":
        bs = state["best_signal"] or {}
        return "FOLLOW_SIGNAL", None, f"Beacon reading {bs.get('strength', 0):.2f} near {tuple(bs.get('position', pos))}; leaving the pattern to search around it."

    est = state.get("estimate")
    if ev == "BEACON_SILENT" and est and est["probability_left"] >= 0.05:
        return ("SEARCH_LAST_SIGNAL", None,
                f"No beacon since tick {est['beacon_last_heard']}, but it was heard then: the estimate puts the hiker "
                f"within ~{max(1, round(est['radius']))} cells of {tuple(est['center'])}. Checking there by camera first.")
    if ev == "BEACON_LOST" and est:
        return ("SEARCH_LAST_SIGNAL", None,
                f"Beacon went silent after tick {est['beacon_last_heard']}; my readings put the hiker within "
                f"~{max(1, round(est['radius']))} cells of {tuple(est['center'])}. Checking the likeliest spots by camera.")
    if ev == "RELAUNCHED" and est and est["probability_left"] >= 0.05:
        return ("SEARCH_LAST_SIGNAL", None,
                f"Fresh battery; {int(est['probability_left'] * 100)}% of the estimate around "
                f"{tuple(est['center'])} is still unchecked.")
    if ev == "RELAUNCHED" and state.get("resume"):
        # Back from a battery swap: go back to exactly what I was doing, unless it's done
        # or (as far as I know) a teammate has taken it over meanwhile.
        r = state["resume"]
        left_at = tuple(r["pos"])
        if r["action"] == "SEARCH_SECTION":
            s = r["arg"]
            now, _ = _claims(state)
            taken = s in now and now[s] != me
            if state["sector_progress"][s] < 1 and not taken:
                return ("SEARCH_SECTION", s,
                        f"Fresh battery; resuming sector {s} where I left off at {left_at} "
                        f"({int(state['sector_progress'][s] * 100)}% searched).")
        if r["action"] == "FOLLOW_SIGNAL" and state["best_signal"]:
            return ("FOLLOW_SIGNAL", None,
                    f"Fresh battery; going back to the beacon signal I was following near {left_at}.")

    if ev == "RENDEZVOUS_DUE":
        nr = state["next_rendezvous_tick"]
        if state["current_action"] == "FOLLOW_SIGNAL" and state["recent_signal"]:
            return _keep(state, f"Skipping the tick-{nr} rendezvous: I'm on a live beacon signal.")
        if state["current_action"] == "SEARCH_LAST_SIGNAL":
            return _keep(state, f"Skipping the tick-{nr} rendezvous: I'm checking where the beacon was last heard.")
        if not alive:
            return _keep(state, "No teammates presumed alive, so no one to meet; continuing.")
        t = state["tick"]
        silent = [j for j in alive if t - mates[j]["last_heard"] > RENDEZVOUS_SKIP_IF_HEARD]
        if not silent:
            return _keep(state, f"Heard from drones {alive} in the last {RENDEZVOUS_SKIP_IF_HEARD} ticks; maps are synced, skipping the trip.")
        return "GO_TO_RENDEZVOUS", None, f"Rendezvous at tick {nr}; haven't heard from drone(s) {silent} recently."

    if ev == "LOST_CONTACT":
        if state["current_action"] in ("GO_TO_RENDEZVOUS", "FOLLOW_SIGNAL", "SEARCH_LAST_SIGNAL") or (
                state["current_action"] == "SEARCH_SECTION"
                and state["sector_progress"].get(state["current_target"], 1) < 1):
            j = state["event_arg"]
            return _keep(state, f"Drone {j} presumed lost; its sectors are now unclaimed. Finishing my current task first.")

    if ev == "MEETUP_COMPLETE":
        s, group = _agreed_split(state)
        if s:
            return "SEARCH_SECTION", s, f"Meetup with {[g for g in group if g != me]} done; agreed split gives me sector {s} ({int(state['sector_progress'][s] * 100)}% searched)."

    ranked, now, plan = _rank_sectors(state)
    if not ranked:
        return "RETURN_HOME", None, "Every sector is searched according to my map; returning to base."
    s = ranked[0]
    prefix = {"MISSED_MEETUP": f"Drone(s) {state['event_arg']} missed the meetup, so their sectors are open. ",
              "LOST_CONTACT": f"Drone {state['event_arg']} presumed lost. ",
              "SECTOR_COMPLETE": f"Sector {state['current_target']} done. ",
              "SIGNAL_LOST": "Signal trail went cold. ",
              "ESTIMATE_EXHAUSTED": "Checked the likely spots around the last signal; widening to a camera search. ",
              "BEACON_SILENT": "Whole mountain swept with no beacon: it must be off. Re-searching by camera. ",
              "BEACON_LOST": "Beacon went silent. ",
              "RELAUNCHED": "Fresh battery. "}.get(ev, "")
    return "SEARCH_SECTION", s, f"{prefix}Taking sector {s}: {_sector_note(state, s, now, plan)}."


def decide(state):
    """The drone's decision: (action, argument, one-line reason)."""
    action, arg, reason = decide_default(state)
    assert action in ACTIONS, action
    return action, arg, reason


decide_rules = decide  # old name
