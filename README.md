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

# 3. Run the server
uvicorn server:app --reload

# 4. Open http://localhost:8000
```

The sim also runs with no API key: agents fall back to rules-based decisions.

### Adding your Claude API key

1. Create a key at <https://console.anthropic.com> → **API Keys**.
2. Create a file called `.env` in the project folder (next to `server.py`) containing one line:
   ```
   ANTHROPIC_API_KEY=sk-ant-...your key...
   ```
   Easiest on Windows: `notepad .env`, paste the line, save. (The server also copes with the UTF-16 file that PowerShell's `echo ... > .env` writes.)
3. Restart the server (`.env` is only read at startup).
4. In the page, set **Agent** to **Claude Haiku (LLM)**. The status tag should say "LLM ready", and the log should start showing `llm` decisions.

`.env` is in `.gitignore`. Never paste the key into code, chat, or a commit.

Headless benchmark (ours vs baseline, rules agent, several seeds):

```bash
python bench.py --seeds 5
```

Independence test (no API key needed, ~12 s). Proves the drones only share information over the simulated radio and each LLM agent sees only its own drone's knowledge:

```bash
python tests/test_isolation.py
```

It checks that: with the radio off, each drone knows only what its own sensors saw; every delivered message (3,000+ per run of the test) had the radio on, both drones alive, range ≤ `COMMS_RANGE` and a clear line over the terrain; nobody hears from a drone after it is killed; line of sight is the same in both directions; and each LLM call carries only its own drone's memory and never the raw map.

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
| `world.py` | Constants, random heightmap terrain, hidden hiker, beacon signal, line of sight, sector helpers |
| `drone.py` | Drone autopilot: position, battery, movement, local map, gossip merge, event detection |
| `agent.py` | Candidate builder, `decide_rules`, LLM agent (one per drone) with fallback |
| `baseline.py` | Central-hub baseline mode, for comparison |
| `sim.py` | Tick loop, comms, async agent decisions, metrics, headless runner |
| `server.py` | FastAPI: serves the page, `/state` and `/world` JSON, control endpoints |
| `static/index.html` | One file: canvas, controls, reasoning log, metrics (vanilla JS) |
| `bench.py` | Headless comparison across seeds, packet loss and failure scenarios |
| `tests/test_isolation.py` | Proves the agents are independent and only talk over the radio |

---

## The world

- **Grid:** 63 × 63 cells, each with a terrain **height** from 0.0 (valley floor) to 1.0 (highest peak).
- **Terrain:** randomly generated per seed as a sum of Gaussian mountains, a few long ridgelines (the main radio walls) and some rough texture, all scaled with the map size. The base sits in a valley. The UI shows it as a height heatmap with hillshading and contour lines.
- **Sectors:** 9 sectors of 21 × 21 (3 × 3, labelled `A`–`I`, row-major: `A B C` is the top row, `G H I` the bottom row). 21 = exactly three 7-wide search lanes.
- **Launch point (base):** somewhere on the **edge of the map**, because rescuers know roughly which side the hiker went in from but not more. It is random per seed (any wall or corner), or pick one in the UI (`Launch from`). All drones launch from here, `RETURN_HOME` flies here, and the base sits in a valley.
- **Flying:** drones hug the terrain at `FLIGHT_CLEARANCE` above the ground, so a drone over a peak is high up and one in a valley is low.
- **Line of sight:** the straight line between two drones' altitudes must clear the terrain at every cell in between. Mountains between drones cut the radio link; climbing to high ground (`CLIMB_TO_RELAY`) restores it. Roughly 40% of drone pairs that are within range are blocked by terrain.
- **Hiker:** hidden at a random cell (not a summit) at least 21 cells from base, unknown to the drones.
- **Beacon signal:** `strength = max(0, 1 - distance / BEACON_RANGE)`, halved if terrain blocks line of sight to the hiker, plus Gaussian noise. Readings below `SIGNAL_THRESHOLD` count as nothing. `SIGNAL_DETECTED` needs two readings within 5 ticks (debounces noise spikes), and `FOLLOW_SIGNAL` searches around the strength-weighted centre of recent readings, not a single one.
- **Found:** a drone finds the hiker when its camera sees it: within `FIND_RADIUS` cells (Chebyshev distance).
- **Movement:** 1 cell per tick, 8 directions.
- **Time:** 1 tick = 1 simulated minute. Rendezvous happen at every multiple of `RENDEZVOUS_INTERVAL`.
- **Battery:** drains `BATTERY_DRAIN` per tick (flying or hovering). `LOW_BATTERY` fires when the battery only covers the trip home plus `BATTERY_RESERVE_TICKS`.

### How a drone searches (two sensors)

1. **Sweep for the beacon, not with the camera.** The beacon is heard from about 4 cells away (3.5 even through a hill), but the camera only sees 1 cell. So drones fly lawnmower lanes **7 cells apart** and mark every cell within `BEACON_CLEAR_RADIUS = 3` as *cleared*: if the hiker were there, the beacon would have been heard. Lanes that are already fully cleared (by this drone or, via gossip, a teammate) are skipped.
2. **Close in with the camera.** On `SIGNAL_DETECTED` the drone switches to `FOLLOW_SIGNAL` and searches around the readings at camera resolution (`SENSOR_RADIUS = 1`) until it sees the hiker.

Measured on the 42 × 42 map, this found the hiker **about 40% sooner** than camera-width lanes (56 vs 98 ticks with perfect radio, 74 vs 132 with a drone killed), with no missed hikers in 150 runs. The cost: more cells are swept twice, because a 7-wide footprint overlaps more when drones transit over each other's ground.

### Tunable constants (top of `world.py`)

Sized so a full search takes a few hundred ticks, so comms failures actually matter before the hiker is found.

```python
GRID_SIZE = 63
SECTOR_SIZE = 21
NUM_DRONES = 3
SENSOR_RADIUS = 1         # camera: close search when homing in on a signal
COMMS_RANGE = 20          # cells, drone to drone
BASE_COMMS_RANGE = 46     # base station mast (baseline mode only)
PACKET_LOSS = 0.0         # 0.0 - 1.0, controlled by a slider
BEACON_RANGE = 5
BEACON_NOISE = 0.05
SIGNAL_THRESHOLD = 0.15
BEACON_CLEAR_RADIUS = 3   # search lane half-width: a pass this close would have heard the beacon
FIND_RADIUS = 1
RENDEZVOUS_INTERVAL = 150 # ticks (fewer, longer gaps measured better than 90/120 on this map)
RENDEZVOUS_POINT = (31, 31)
RENDEZVOUS_SKIP_IF_HEARD = 15
LOST_TIMEOUT = 180        # ticks of silence before a teammate is presumed lost (> interval, so rendezvous detects it first)
MISSED_MEETUP_WAIT = 5    # ticks to wait at the rendezvous for a missing drone
START_OPTIONS = ["random", "S", "N", "W", "E", "NW", "NE", "SW", "SE"]  # launch point choices
BATTERY_DRAIN = 0.0014    # per tick, i.e. ~700 ticks of flight
BATTERY_RESERVE_TICKS = 20
SIGNAL_FOLLOW_LIMIT = 40  # ticks before giving up on a signal
RELAY_HOVER_TICKS = 8     # how long CLIMB_TO_RELAY waits on high ground
MAX_TICKS = 2000
MAP_SCALE = GRID_SIZE / 30  # terrain features scale with the map
FLIGHT_CLEARANCE = 0.04   # drones fly this far above the terrain
MAST_HEIGHT = 0.15        # base station mast (baseline mode)
NUM_PEAKS = (8, 12)       # round mountains
NUM_RIDGES = (3, 5)       # long ridgelines
RELAY_SEARCH_RADIUS = 14  # how far CLIMB_TO_RELAY looks for high ground
```

### Pre-launch plan

The plan is computed from the launch point before take-off (`make_plan` in `world.py`), so every drone knows everyone's plan with zero comms:

1. Sort the 9 sectors by **bearing from base** (measured from "straight into the map") and cut them into **three wedges** fanning out from base, one per drone.
2. Each drone searches its own wedge **nearest sector first**.
3. Then the other drones' sectors **in reverse order** (furthest first, where their owners are least likely to have reached yet), nearest drone first, interleaved.

Examples:

| Launch from | Drone 0 | Drone 1 | Drone 2 |
|---|---|---|---|
| Bottom wall | `G D A` (left column) | `H E B` (middle column) | `I F C` (right column) |
| Left wall | `A B C` (top row) | `D E F` (middle row) | `G H I` (bottom row) |
| Top-left corner | `B C F` (along the top) | `A E I` (the diagonal) | `D G H` (down the left) |

---

## Communication model

Two drones can exchange messages only if **all** of these hold:
1. both are alive and comms are enabled
2. distance ≤ `COMMS_RANGE`
3. line of sight (no terrain above the straight line between their flying altitudes)
4. the message isn't dropped (random chance `PACKET_LOSS`, per message, per direction)

When connected, drones **gossip** every tick: each one sends its whole local knowledge and the receiver merges it:
- `searched` (beacon-cleared cells): union of both sets
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
    "my_sectors": ["H", "E", "B"],                  # depends on the launch point
    "priority_order": ["H", "E", "B", "A", "C", "D", "F", "G", "I"],
    "searched_cells": {(0, 0), ...},          # beacon-cleared cells; rules may use it; NEVER sent to the LLM
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
    "rendezvous_point": (31, 31),
    "height": 0.31,                            # terrain height under the drone
    "relay_point": (6, 9),                     # best nearby high ground for CLIMB_TO_RELAY
    "relay_height": 0.74,
    "base_position": (31, 62),                 # launch point on the map edge
    "plans": {0: {"own": [...], "priority": [...]}, ...},  # everyone's pre-launch plan
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
| `SEARCH_SECTION` | sector label | Fly to that sector and sweep it in 7-wide beacon lanes |
| `FOLLOW_SIGNAL` | none | Leave the lanes, camera-search around the strongest recent readings |
| `GO_TO_RENDEZVOUS` | none | Fly to the rendezvous point and wait for teammates |
| `GO_TO_TEAMMATE` | drone id | Fly to a teammate's last known position (e.g. to report a find) |
| `CLIMB_TO_RELAY` | none | Fly to the best nearby high ground and hover there to get line of sight over the hills |
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
   - **If a drone dies,** nobody flies off to look for it: the survivors notice at the next meetup (or the timeout), spread the news by gossip, and fold its unsearched sectors into their own priority order. Until then they assume it is still searching its own sectors, so the takeover can lag by up to one `RENDEZVOUS_INTERVAL`.
9. **Found it? Tell someone:** the finder goes to the nearest teammate who doesn't know yet, or the rendezvous, whichever is closer, instead of waiting.

---

## Baseline: central hub (for comparison)

`baseline.py` runs the same world with a **naive central controller** at the base (mast range `BASE_COMMS_RANGE`, mast `MAST_HEIGHT` tall, still blocked by mountains):
- The hub assigns every drone its next sector (each sector is assigned once).
- A drone that needs instructions but can't reach the hub **hovers and waits**.
- No drone-to-drone map sharing, no rendezvous, no takeover of lost drones' sectors.
- Only battery safety is autonomous.

---

## Metrics (shown live on the page)

- **Time to find** (ticks) and **time until every live drone knows**
- **% of mountain searched** (ground truth, beacon-cleared)
- **% duplicated**: searched cells that were swept by 2 or more different drones
- **Messages sent / dropped**
- **LLM calls / fallbacks / average latency / tokens**

---

## Web UI

- **Demo seed 2** (the default, launch point random → near the top-left corner): drone 1 owns the diagonal `A E I` and the hiker is in the far corner, sector I. Kill drone 1 around tick 40: drones 0 and 2 take over its wedge and still find the hiker (tick ~250); the baseline never does.
- **Launch from:** pick a wall or corner (or random) to show the plan adapting; the plan for the current launch point is shown under the controls.
- **Canvas:** terrain height heatmap with contours, unsearched cells dimmed, a dot per searched cell in the colour of the drone that searched it, drones, comms links (green = delivered, red = dropped), rendezvous point, base, hiker (hidden until found, or "reveal").
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
| POST | `/control/reset` | `{"seed": 42, "mode": "ours", "start": "random"}` (`start`: `random`, `S`, `N`, `W`, `E`, `NW`, `NE`, `SW`, `SE`) |
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
- Wide beacon lanes trust the beacon: a cell marked *cleared* is never revisited, so a hiker whose beacon failed (or a run of unlucky noise) would be missed. It never happened in 150 benchmark runs, but a real system would do a second, camera-width pass afterwards.
- Rendezvous trips cost search time. Too frequent is wasteful, too rare means stale maps (mitigated by skipping when already in contact).
- A drone that is merely out of range at meetup time is presumed lost until it is heard again; its sectors may be searched twice.
- The signal and terrain model is simplified: line of sight over a heightmap, a hard radio range, and no battery cost for climbing.
- LLM decisions are not reproducible, so the benchmark uses the rules agent.
