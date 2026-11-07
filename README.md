# Mountain Rescue Drones 🚁⛰️

**GreatUniHack 2026: Neptune North Challenge**

> Three autonomous drones, each with its own AI agent, search a mountain for a lost hiker carrying an RF beacon. They split the mountain before launch, keep searching on their own when the radio drops, sync their maps whenever they meet, and use a pre-agreed rendezvous so nothing stays lost for long.

---

## Running it

```bash
# 1. Create and activate a venv (once)
python -m venv .venv
# Windows:  .venv\Scripts\Activate.ps1
# Mac:      source .venv/bin/activate

# 2. Install dependencies
pip install -r requirements.txt

# 3. Add your API key to a .env file (NEVER commit this)
echo ANTHROPIC_API_KEY=sk-ant-... > .env

# 4. Run the server
uvicorn server:app --reload

# 5. Open http://localhost:8000
```

The sim also runs with no API key: agents fall back to rules-based decisions.

Headless benchmark (ours vs baseline, rules agent, several seeds):

```bash
python bench.py --seeds 5
```

---

## The challenge brief, and how we answer it

| Brief | Our answer |
|---|---|
| Multiple agents | 3 drones, each running its own independent AI agent (Claude Haiku), seeing only its own drone's knowledge |
| Shared goal | Find the hiker as fast as possible while covering the mountain with as little duplicated search as possible |
| Keep working when comms fail | Lost drones keep searching using a pre-agreed priority order, then sync at the next encounter or rendezvous |
| No central hub | No drone or base station gives orders. Every decision is made locally from the drone's own knowledge |

---

## Files

| File | What it does |
|---|---|
| `world.py` | Constants, grid, ridges, hidden hiker, beacon signal, line of sight, sector helpers |
| `drone.py` | Drone autopilot: position, battery, movement, local map, gossip merge, event detection |
| `agent.py` | Candidate builder, `decide_rules`, LLM agent (one per drone) with fallback |
| `baseline.py` | Central-hub baseline mode, for comparison |
| `sim.py` | Tick loop, comms, async agent decisions, metrics, headless runner |
| `server.py` | FastAPI: serves the page, `/state` and `/world` JSON, control endpoints |
| `static/index.html` | One file: canvas, controls, reasoning log, metrics (vanilla JS) |
| `bench.py` | Headless comparison across seeds, packet loss and failure scenarios |

---

## The world

- **Grid:** 30 × 30 cells. Each cell is `open` or `ridge`.
- **Sectors:** 9 sectors of 10 × 10 (3 × 3, labelled `A`–`I`, row-major: `A B C` is the top row, `G H I` the bottom row).
- **Base:** `BASE_POSITION = (15, 29)`, bottom centre. All drones launch from here and `RETURN_HOME` flies here.
- **Ridges** block line of sight (and therefore radio). Drones can fly over them. A drone **sitting on a ridge cell** is on high ground and has line of sight to everything (that is what `CLIMB_TO_RELAY` uses).
- **Hiker:** hidden at a random open cell at least 10 cells from base, unknown to the drones.
- **Beacon signal:** `strength = max(0, 1 - distance / BEACON_RANGE)`, halved if a ridge blocks line of sight, plus Gaussian noise. Readings below `SIGNAL_THRESHOLD` count as nothing. `SIGNAL_DETECTED` needs two readings within 5 ticks (debounces noise spikes), and `FOLLOW_SIGNAL` searches around the strength-weighted centre of recent readings, not a single one.
- **Found:** a drone finds the hiker when it's within `FIND_RADIUS` cells (Chebyshev distance).
- **Searching:** a drone marks every cell within `SENSOR_RADIUS` as searched as it flies. Inside a sector it flies a lawnmower pattern that skips rows already known to be searched.
- **Movement:** 1 cell per tick, 8 directions.
- **Time:** 1 tick = 1 simulated minute. Rendezvous happen at every multiple of `RENDEZVOUS_INTERVAL`.
- **Battery:** drains `BATTERY_DRAIN` per tick (flying or hovering). `LOW_BATTERY` fires when the battery only covers the trip home plus `BATTERY_RESERVE_TICKS`.

### Tunable constants (top of `world.py`)

Sized so a full search takes a few hundred ticks, so comms failures actually matter before the hiker is found.

```python
GRID_SIZE = 30
SECTOR_SIZE = 10
NUM_DRONES = 3
SENSOR_RADIUS = 1
COMMS_RANGE = 12          # cells, drone to drone
BASE_COMMS_RANGE = 22     # base station mast (baseline mode only)
PACKET_LOSS = 0.0         # 0.0 - 1.0, controlled by a slider
BEACON_RANGE = 5
BEACON_NOISE = 0.05
SIGNAL_THRESHOLD = 0.15
FIND_RADIUS = 1
RENDEZVOUS_INTERVAL = 45  # ticks
RENDEZVOUS_POINT = (15, 15)
LOST_TIMEOUT = 60         # ticks of silence before a teammate is presumed lost (> interval, so rendezvous detects it first)
MISSED_MEETUP_WAIT = 5    # ticks to wait at the rendezvous for a missing drone
BASE_POSITION = (15, 29)
BATTERY_DRAIN = 0.0025    # per tick, i.e. 400 ticks of flight
BATTERY_RESERVE_TICKS = 12
SIGNAL_FOLLOW_LIMIT = 40  # ticks before giving up on a signal
RELAY_HOVER_TICKS = 8     # how long CLIMB_TO_RELAY waits on the ridge
MAX_TICKS = 900
```

### Pre-launch plan

| Drone | Own sectors (searched in this order) | Full priority order |
|---|---|---|
| 0 | `G D A` (left column) | `G D A B C E F H I` |
| 1 | `H E B` (middle column) | `H E B A C D F G I` |
| 2 | `I F C` (right column) | `I F C B A E D H G` |

Own sectors bottom to top, then other drones' sectors **in reverse order** (nearest drone first), interleaved.

---

## Communication model

Two drones can exchange messages only if **all** of these hold:
1. both are alive and comms are enabled
2. distance ≤ `COMMS_RANGE`
3. line of sight (no ridge on the straight line between them, unless one of them is on a ridge)
4. the message isn't dropped (random chance `PACKET_LOSS`, per message, per direction)

When connected, drones **gossip** every tick: each one sends its whole local knowledge and the receiver merges it:
- `searched`: union of both sets
- `best_signal`: keep the stronger reading
- `teammates`: for each drone, keep whichever info has the newest `last_heard`; `missed_at` keeps the max
- `hiker_found`: `True` if either drone knows it

Merging is **order-independent and idempotent**: duplicate or late messages never break anything.

---

## Drone state (the shared contract)

The dict that `drone.py` builds and passes to `decide()`:

```python
state = {
    "id": 1,
    "tick": 142,
    "position": (4, 7),
    "battery": 0.63,
    "current_action": "SEARCH_SECTION",
    "current_target": "B",
    "my_sectors": ["H", "E", "B"],
    "priority_order": ["H", "E", "B", "A", "C", "D", "F", "G", "I"],
    "searched_cells": {(0, 0), ...},          # rules may use it; NEVER sent to the LLM
    "sector_progress": {"A": 1.0, "B": 0.4, ...},
    "best_signal": {"strength": 0.32, "position": (5, 9), "tick": 140},  # or None
    "hiker_found": False,
    "hiker_position": None,
    "teammates": {
        0: {"position": (2, 3), "last_heard": 130, "current_action": "SEARCH_SECTION",
            "current_target": "D", "alive_presumed": True, "knows_hiker_found": False},
        2: {...},
    },
    "in_contact_with": [0],
    "next_rendezvous_tick": 180,
    "rendezvous_point": (15, 15),
    "nearest_ridge": (6, 9),
    "base_position": (15, 29),
    "event": "LOST_CONTACT",
    "event_arg": 2,                            # e.g. the teammate id, or None
}
```

### Events that trigger a decision
`decide()` is **not** called every tick, only on:
- `SECTOR_COMPLETE`: finished current sector
- `LOST_CONTACT`: a teammate passed `LOST_TIMEOUT`
- `SIGNAL_DETECTED`: beacon reading above threshold
- `SIGNAL_LOST`: gave up following a signal (no unsearched cells near it, or `SIGNAL_FOLLOW_LIMIT`)
- `HIKER_FOUND`: found it, or learned about it through gossip
- `RENDEZVOUS_DUE`: time to leave for the rendezvous
- `MEETUP_COMPLETE`: everyone presumed alive is in contact at the rendezvous
- `MISSED_MEETUP`: a teammate didn't arrive within `MISSED_MEETUP_WAIT`
- `ARRIVED`: reached a teammate / finished relaying on a ridge
- `LOW_BATTERY`

---

## Actions

| Action | Argument | Meaning |
|---|---|---|
| `SEARCH_SECTION` | sector label | Fly to and lawnmower-search that sector |
| `FOLLOW_SIGNAL` | none | Leave the pattern, search outward from the strongest recent readings |
| `GO_TO_RENDEZVOUS` | none | Fly to the rendezvous point and wait for teammates |
| `GO_TO_TEAMMATE` | drone id | Fly to a teammate's last known position (e.g. to report a find) |
| `CLIMB_TO_RELAY` | none | Fly to the nearest ridge cell and hover there to get line of sight |
| `RETURN_HOME` | none | Fly back to base and land |
| `HOVER` | none | Baseline only: wait for instructions from the hub |

---

## The agents

```python
# agent.py
def build_candidates(state) -> (candidates, default_index)
def decide_rules(state) -> (action, argument, reason)
```

**The rules propose, the LLM chooses.**

1. `build_candidates` turns the drone's state into a short list (≤ 8) of **legal** options, each with a one-line fact, e.g. `SEARCH_SECTION F: 20% searched, unclaimed (drone 2 presumed lost)`. One of them is the rules' default.
2. **Rules agent** (`decide_rules`): returns the default. Always works, instant, deterministic.
3. **LLM agent:** each drone has its own Claude Haiku agent (`claude-haiku-5-5`) with its own short memory of its last 5 decisions. It gets a compact summary (sector progress, teammates, signal, event, candidates; never the raw cell set) and returns `{"choice": index, "reason": "..."}` via structured outputs, so it can only pick a legal option.
4. **Fallback:** timeout (2 s free-running), API error, rate limit (429) or invalid choice → the rules decision stands. The drone never stalls.

### Two run modes for the LLM
- **Wait for agents (demo):** the sim clock pauses while any drone's agent is thinking, so reasoning lines up with movement. Three agents think in parallel.
- **Free-running:** the drone applies the rules decision immediately and switches if the LLM answer arrives in time. Stale answers (a newer event already fired) are discarded by decision id.

Benchmarks always use the rules agent, so runs are exactly repeatable per seed. That is the floor the system guarantees even without the LLM.

---

## Resilience rules (what makes it work without comms)

1. **Pre-assigned sectors:** each drone owns 3 sectors before launch.
2. **Priority order:** own sectors first, then other drones' sectors in reverse order. Minimal overlap, zero comms needed.
3. **Assume teammates follow their plan:** a sector a live teammate is searching, or will reach next in its own plan, counts as claimed.
4. **Gossip on contact:** whenever two drones can talk, they merge everything they know.
5. **Scheduled rendezvous:** at every multiple of `RENDEZVOUS_INTERVAL`, drones fly to `RENDEZVOUS_POINT`. The meetup is complete as soon as everyone presumed alive is in radio contact (no need to be on the exact cell). A drone **skips** the trip if it is already in contact with every live teammate.
6. **Deterministic re-split:** at `MEETUP_COMPLETE` every drone has the same knowledge, so each one runs the same assignment (drones in id order, each keeps its unfinished sector, others take the best unclaimed sector by their own priority order). No negotiation needed.
7. **Missed meetup:** a drone not in contact within `MISSED_MEETUP_WAIT` ticks is marked `missed_at`, presumed lost, and its unsearched sectors become unclaimed. If it shows up again in gossip later, its newer `last_heard` brings it back.
8. **Lost-contact timeout:** a teammate silent for `LOST_TIMEOUT` ticks is presumed lost.
9. **Found it? Tell someone:** the finder goes to the nearest teammate who doesn't know yet, or the rendezvous, whichever is closer, instead of waiting.

---

## Baseline: central hub (for comparison)

`baseline.py` runs the same world with a **naive central controller** at the base (mast range `BASE_COMMS_RANGE`, still blocked by ridges):
- The hub assigns every drone its next sector (each sector is assigned once).
- A drone that needs instructions but can't reach the hub **hovers and waits**.
- No drone-to-drone map sharing, no rendezvous, no takeover of lost drones' sectors.
- Only battery safety is autonomous.

---

## Metrics (shown live on the page)

- **Time to find** (ticks) and **time until every live drone knows**
- **% of mountain searched** (ground truth)
- **% duplicated**: searched cells that were swept by 2 or more different drones
- **Messages sent / dropped**
- **LLM calls / fallbacks / average latency / tokens**

---

## Web UI

- **Canvas:** grid, ridges, searched cells (tinted per drone), drones, comms links (green = delivered, red = dropped), rendezvous point, base, hiker (hidden until found, or "reveal").
- **View as drone N:** show only what that drone knows, to show map merging.
- **Controls:** start/pause, step, speed, reset (seed), packet-loss slider, comms on/off, kill drone 0/1/2, mode (ours / baseline), agent (rules / LLM), wait-for-agents.
- **Reasoning log:** `[tick] Drone N (source): ACTION arg: reason`.
- **Benchmark button:** runs the headless comparison and shows the table.

### API
| Method | Path | Body / effect |
|---|---|---|
| GET | `/` | serves `static/index.html` |
| GET | `/world` | static grid: ridges, sectors, base, rendezvous |
| GET | `/state?view=-1` | full sim state (`view` = drone id to show only its knowledge) |
| POST | `/control/start` / `/control/pause` / `/control/step` | |
| POST | `/control/reset` | `{"seed": 42, "mode": "ours"}` |
| POST | `/control/speed` | `{"ticks_per_second": 10}` |
| POST | `/control/packet_loss` | `{"value": 0.5}` |
| POST | `/control/comms` | `{"enabled": false}` |
| POST | `/control/kill` | `{"drone_id": 2}` |
| POST | `/control/agent` | `{"mode": "llm", "wait": true}` |
| POST | `/bench` | `{"seeds": 3}` runs the comparison |

---

## Team rules

- **Pull before you start, push often.**
- **Never commit `.env`** (API key). It's in `.gitignore`.
- **Code freeze at 17:15.** After that, only bug fixes.

## Known limitations (be honest in the pitch)

- During a long blackout, two drones may both take over the same lost sector, which duplicates work until they reconnect.
- A lone drone following a noisy signal can head the wrong way for a while.
- Rendezvous trips cost search time. Too frequent is wasteful, too rare means stale maps (mitigated by skipping when already in contact).
- A drone that is merely out of range at meetup time is presumed lost until it is heard again; its sectors may be searched twice.
- The signal and terrain model is simplified (2D grid, ridges as walls, ridge cells as high ground).
- LLM decisions are not reproducible, so the benchmark uses the rules agent.
