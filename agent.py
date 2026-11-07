"""Per-drone decision agents.

The rules propose, the LLM chooses:
  build_candidates(state) -> (candidates, default_index)   legal options + one-line facts
  decide_rules(state)     -> (action, arg, reason)          the default, instant and deterministic
  submit_llm(...)         -> Future[(choice, reason, usage, latency)]

agent.py never sees the true world, only one drone's `state`.
"""
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor

from world import (GRID_SIZE, OWN_SECTORS, RENDEZVOUS_SKIP_IF_HEARD, SECTOR_LABELS, SECTOR_SIZE,
                   cheb, priority_order, sector_center)

ACTIONS = ["SEARCH_SECTION", "FOLLOW_SIGNAL", "GO_TO_RENDEZVOUS", "GO_TO_TEAMMATE",
           "CLIMB_TO_RELAY", "RETURN_HOME"]
MODEL = "claude-haiku-5-5"
MAX_CANDIDATES = 8


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
        own = OWN_SECTORS[j]
        start = own.index(tgt) + 1 if tgt in own else 0
        for s in own[start:]:
            if prog[s] < 1:
                plan.setdefault(s, j)
    return now, plan


def _lost_owner(state, s):
    for j, t in state["teammates"].items():
        if s in OWN_SECTORS[j] and not t["alive_presumed"]:
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
        order = priority_order(j)
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

    if ev == "RENDEZVOUS_DUE":
        nr = state["next_rendezvous_tick"]
        if state["current_action"] == "FOLLOW_SIGNAL" and state["recent_signal"]:
            return _keep(state, f"Skipping the tick-{nr} rendezvous: I'm on a live beacon signal.")
        if not alive:
            return _keep(state, "No teammates presumed alive, so no one to meet; continuing.")
        t = state["tick"]
        silent = [j for j in alive if t - mates[j]["last_heard"] > RENDEZVOUS_SKIP_IF_HEARD]
        if not silent:
            return _keep(state, f"Heard from drones {alive} in the last {RENDEZVOUS_SKIP_IF_HEARD} ticks; maps are synced, skipping the trip.")
        return "GO_TO_RENDEZVOUS", None, f"Rendezvous at tick {nr}; haven't heard from drone(s) {silent} recently."

    if ev == "LOST_CONTACT":
        if state["current_action"] in ("GO_TO_RENDEZVOUS", "FOLLOW_SIGNAL") or (
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
              "SIGNAL_LOST": "Signal trail went cold. "}.get(ev, "")
    return "SEARCH_SECTION", s, f"{prefix}Taking sector {s}: {_sector_note(state, s, now, plan)}."


def build_candidates(state):
    """Short list of legal options with one-line facts. Returns (candidates, default_index)."""
    a, arg, reason = decide_default(state)
    cands = [{"action": a, "arg": arg, "note": reason}]

    def add(action, argument, note):
        if len(cands) >= MAX_CANDIDATES:
            return
        if any(c["action"] == action and c["arg"] == argument for c in cands):
            return
        cands.append({"action": action, "arg": argument, "note": note})

    pos = state["position"]
    mates = state["teammates"]
    contact = set(state["in_contact_with"])
    alive = [j for j, t in mates.items() if t["alive_presumed"]]

    if state["current_action"] in ACTIONS and state["current_action"] != "RETURN_HOME":
        if not (state["current_action"] == "SEARCH_SECTION"
                and state["sector_progress"].get(state["current_target"], 1) >= 1):
            add(state["current_action"], state["current_target"], "Continue what I'm doing now.")

    if not state["hiker_found"]:
        ranked, now, plan = _rank_sectors(state)
        for s in ranked[:3]:
            add("SEARCH_SECTION", s, f"Sector {s}: {_sector_note(state, s, now, plan)}, "
                                     f"{cheb(pos, sector_center(s))} ticks away.")
        if state["best_signal"] and (state["recent_signal"] or state["event"] == "SIGNAL_DETECTED"):
            bs = state["best_signal"]
            add("FOLLOW_SIGNAL", None, f"Best beacon reading {bs['strength']:.2f} at {tuple(bs['position'])}, tick {bs['tick']}.")

    rp = tuple(state["rendezvous_point"])
    add("GO_TO_RENDEZVOUS", None, f"Rendezvous {rp} is {cheb(pos, rp)} ticks away; next meetup at tick {state['next_rendezvous_tick']}.")

    for j in alive:
        if j in contact:
            continue
        knows = mates[j]["knows_hiker_found"]
        if state["hiker_found"] and not knows:
            tp = tuple(mates[j]["position"])
            add("GO_TO_TEAMMATE", j, f"Drone {j} doesn't know about the hiker; last seen at {tp} (tick {mates[j]['last_heard']}), {cheb(pos, tp)} ticks away.")

    relay = state.get("relay_point")
    if relay and any(j not in contact for j in alive) \
            and state["relay_height"] - state["height"] >= 0.15:
        add("CLIMB_TO_RELAY", None,
            f"High ground at {tuple(relay)} (height {int(state['relay_height'] * 100)}%, I'm at "
            f"{int(state['height'] * 100)}%) is {cheb(pos, relay)} ticks away; altitude gives radio line of sight over the hills.")

    home = tuple(state["base_position"])
    add("RETURN_HOME", None, f"Base is {cheb(pos, home)} ticks away; battery {int(state['battery'] * 100)}%.")
    return cands, 0


def decide_rules(state):
    cands, i = build_candidates(state)
    c = cands[i]
    return c["action"], c["arg"], c["note"]


# ====================================================================== LLM
SYSTEM_PROMPT = f"""You are the onboard decision agent of one search-and-rescue drone in a team of three, searching a {GRID_SIZE}x{GRID_SIZE} mountain grid (9 sectors A-I, {SECTOR_SIZE}x{SECTOR_SIZE} each; A B C is the top row, G H I the bottom row next to base) for a lost hiker carrying a radio beacon. Drones sweep in wide lanes listening for the beacon, then close in with the camera when they hear it.

There is no central controller. You only know what your own drone has sensed or heard from teammates over a radio that drops out (range limit, mountains between drones block line of sight, packet loss). Drones fly just above the terrain, so a drone on high ground can talk over the hills. Teammates you can't hear are assumed to be following their last known plan until they are presumed lost.

Shared goal: find the hiker as fast as possible, cover the mountain with as little duplicated searching as possible, and make sure every live drone learns when the hiker is found.

Guidelines:
- Avoid sectors a live teammate is searching or is about to search; prefer unclaimed sectors, especially ones owned by teammates presumed lost.
- A fresh beacon reading is the strongest clue you have.
- Rendezvous trips cost search time; they are worth it when you have been out of contact.
- Never strand the drone: low battery means going home.
- The `default` option is a safe rules-based choice. Pick something else only if you can name a concrete reason from the data.

You will receive the drone's situation as JSON with a numbered `candidates` list. Pick exactly one candidate by index. `reason` must be one first-person sentence of at most 25 words that cites the concrete fact that decided it."""

OUTPUT_SCHEMA = {
    "type": "object",
    "properties": {
        "choice": {"type": "integer"},
        "reason": {"type": "string"},
    },
    "required": ["choice", "reason"],
    "additionalProperties": False,
}

_client = None
_executor = ThreadPoolExecutor(max_workers=3, thread_name_prefix="drone-agent")


def llm_available():
    return bool(os.getenv("ANTHROPIC_API_KEY") or os.getenv("ANTHROPIC_AUTH_TOKEN"))


def _get_client():
    global _client
    if _client is None:
        from anthropic import Anthropic
        _client = Anthropic(max_retries=0)
    return _client


def llm_payload(state, cands, default, memory):
    """Compact view of the state. The raw searched-cell set is never sent."""
    t = state["tick"]
    return {
        "drone": state["id"],
        "tick": t,
        "event": state["event"],
        "event_arg": state["event_arg"],
        "position": list(state["position"]),
        "battery_pct": int(state["battery"] * 100),
        "terrain_height_pct": int(state["height"] * 100),
        "doing_now": {"action": state["current_action"], "arg": state["current_target"]},
        "my_sectors": state["my_sectors"],
        "sector_progress_pct": {s: int(p * 100) for s, p in state["sector_progress"].items()},
        "best_signal": state["best_signal"] and {
            "strength": state["best_signal"]["strength"],
            "position": list(state["best_signal"]["position"]),
            "ticks_ago": t - state["best_signal"]["tick"]},
        "hiker_found": state["hiker_found"],
        "teammates": {
            str(j): {"last_known_position": list(m["position"]), "heard_ticks_ago": t - m["last_heard"],
                     "doing": f"{m['current_action']} {m['current_target'] or ''}".strip(),
                     "presumed_alive": m["alive_presumed"], "knows_hiker_found": m["knows_hiker_found"],
                     "in_radio_contact_now": j in state["in_contact_with"]}
            for j, m in state["teammates"].items()
        },
        "next_rendezvous_tick": state["next_rendezvous_tick"],
        "my_recent_decisions": memory,
        "candidates": [{"index": i, "action": c["action"], "arg": c["arg"], "fact": c["note"],
                        **({"default": True} if i == default else {})}
                       for i, c in enumerate(cands)],
    }


def _call_llm(payload, n_cands, timeout):
    t0 = time.monotonic()
    client = _get_client().with_options(timeout=timeout, max_retries=0)
    resp = client.messages.create(
        model=MODEL,
        max_tokens=1024,
        system=SYSTEM_PROMPT,
        output_config={"effort": "low", "format": {"type": "json_schema", "schema": OUTPUT_SCHEMA}},
        messages=[{"role": "user", "content": json.dumps(payload, separators=(",", ":"))}],
    )
    latency = time.monotonic() - t0
    if resp.stop_reason == "refusal":
        raise ValueError("refusal")
    text = next((b.text for b in resp.content if b.type == "text"), None)
    if text is None:
        raise ValueError(f"no text block (stop_reason={resp.stop_reason})")
    data = json.loads(text)
    choice = data["choice"]
    if not isinstance(choice, int) or not 0 <= choice < n_cands:
        raise ValueError(f"invalid choice {choice!r}")
    usage = {"in": resp.usage.input_tokens, "out": resp.usage.output_tokens}
    return choice, str(data["reason"])[:200], usage, latency


def submit_llm(state, cands, default, memory, timeout):
    payload = llm_payload(state, cands, default, memory)
    return _executor.submit(_call_llm, payload, len(cands), timeout)


def describe_error(exc):
    try:
        import anthropic
        if isinstance(exc, anthropic.APITimeoutError):
            return "timeout"
        if isinstance(exc, anthropic.RateLimitError):
            return "rate limited"
        if isinstance(exc, anthropic.AuthenticationError):
            return "bad API key"
        if isinstance(exc, anthropic.APIStatusError):
            return f"API error {exc.status_code}"
        if isinstance(exc, anthropic.APIConnectionError):
            return "connection error"
    except ImportError:
        pass
    return f"{type(exc).__name__}: {exc}"[:60]
