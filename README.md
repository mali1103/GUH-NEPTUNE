# Mountain Rescue Drones 🚁⛰️

**GreatUniHack 2026: Neptune North Challenge**

> Three autonomous drones, each making its own decisions, search a mountain for a lost hiker carrying an RF beacon. They split the mountain before launch, keep searching on their own when the radio drops, sync their maps whenever they meet, and use a pre-agreed rendezvous so nothing stays lost for long.

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

# 4. Open http://localhost:8000          (one team, full detail)
#    or   http://localhost:8000/compare  (all three teams side by side)
```

No API keys, no internet, no extra services: the drones' decision-making is plain Python.

Headless benchmarks (repeatable per seed). Saves `bench_results.json`, which the page's **Evidence** panel shows instantly:

```bash
python bench.py --seeds 10
```

It prints three things: what each layer of teamwork is worth, found-rate vs packet loss for every mode, and every failure scenario (blackout, killed drone, beacon dead/off).

Independence test (~20 s). Proves the drones only share information over the simulated radio and each drone decides only from its own knowledge:

```bash
python tests/test_isolation.py
```

It checks that: with the radio off, each drone knows only what its own sensors saw; every delivered message (3,000+ per run of the test) had the radio on, both drones alive, range ≤ `COMMS_RANGE` and a clear line over the terrain; nobody hears from a drone after it is killed; line of sight is the same in both directions; and every decision is a pure function of that drone's own state: replayed later, after the world has completely changed, it gives the same answer (so no hidden reads of the true world or other drones), and the state never contains the hiker's position before it's found.

---

## The challenge brief, and how we answer it

| Brief | Our answer |
|---|---|
| Multiple agents | 3 drones, each running its own decision agent on its own knowledge only (proved by `tests/test_isolation.py`) |
| Shared goal | Find the hiker as fast as possible while covering the mountain with as little duplicated search as possible |
| Keep working when comms fail | Lost drones keep searching using a pre-agreed priority order, then sync at the next encounter or rendezvous |
| No central hub | No drone or base station gives orders. Every decision is made locally from the drone's own knowledge |

### Headline results (10 maps, realistic 40-minute batteries, from `bench_results.json`)

| Setup (radio working) | Found | Avg time to find | All drones know | Searched twice | Drone flight time |
|---|---|---|---|---|---|
| No coordination (each drone alone) | 10/10 | 56 min | 173 min | 70% | 329 min |
| Pre-launch plan only (radio dead) | 10/10 | 42 min | 145 min | 48% | 282 min |
| **Plan + radio sharing (ours)** | **10/10** | **39 min** | **46 min** | **23%** | **135 min** |
| Central hub | 4/10 | 36 min (when found) | 51 min | 31% | 413 min |

- **Common goal:** the pre-launch plan alone makes the team 25% faster than drones searching alone; the radio then halves double-searching (48% → 23%) and gets the news to every drone 3.2× sooner.
- **Resilience:** ours finds the hiker on every map at 0, 25, 50, 75, 90 and 100% packet loss, through a total blackout (41 min vs 56 alone), with a drone killed (63 min vs 78), and with the beacon dying (53 min vs 83) or off (102 min vs 135). The central hub falls to 0/10 in a blackout or with no radio.
- **Battery / energy:** ours used 135 minutes of drone flight time in total vs 329 with no coordination and 413 for the central hub (its drones hover waiting for orders). Ours needed ~2 battery swaps per search vs ~10 alone and ~13 for the hub. No drone ran flat in any mode or scenario (`python bench.py --show` prints battery use per scenario).

---|---|---|---|---|
| No coordination (each drone alone) | 10/10 | 188 | 602 | 70% |
| Pre-launch plan only (radio dead) | 10/10 | 156 | 531 | 49% |
| **Plan + radio sharing (ours)** | **10/10** | **149** | **173** | **24%** |
| Central hub | 3/10 | 281 | 720 | 24% |

- **Common goal:** the plan alone makes the team 17% faster than drones searching alone; the radio then halves double-searching and gets the news to everyone 3.1× sooner.
- **Resilience:** ours finds the hiker on every map at 0, 25, 50, 75, 90 and 100% packet loss, through a total blackout, a killed drone, and a beacon that dies or was never on. The central hub falls to 0% when the radio fails completely.
- **Beacon off (dead phone):** ours finds the hiker in 282 ticks on average vs 638 for drones searching alone.
- **Battery / energy:** with the radio working, ours used 648 drone-ticks of flight in total vs 1,291 with no coordination and 4,434 for the central hub (its drones hover waiting for orders, drain to the 3% reserve and swap batteries ~5 times a run). Ours never needed a swap; its lowest charge averaged 68%. No drone ran flat in any mode or scenario (`python bench.py --show` prints battery use per scenario).

---

## Files

| File | What it does |
|---|---|
| `world.py` | Constants, random heightmap terrain, hidden hiker, beacon signal, line of sight, sector helpers |
| `drone.py` | Drone autopilot: position, battery, movement, local map, gossip merge, event detection |
| `agent.py` | `decide(state)`: each drone's rules-based decision agent |
| `baseline.py` | Central-hub baseline mode, for comparison |
| `sim.py` | Tick loop, comms, decisions, metrics, the three modes, headless benchmarks |
| `server.py` | FastAPI: serves the page, `/state` and `/world` JSON, control endpoints |
| `static/index.html` | Single run in full detail: canvas, controls, reasoning log, metrics, evidence panel (vanilla JS) |
| `static/compare.html` | The three teams side by side on the same map, driven by one set of controls |
| `bench.py` | Headless comparison across seeds, packet loss and failure scenarios |
| `tests/test_isolation.py` | Proves the agents are independent and only talk over the radio |

---

## Real-world scale

Everything is calibrated to one consistent scale, so times, distances and batteries are realistic:

| Sim | Real world | Basis |
|---|---|---|
| 1 tick | **10 seconds** | |
| 1 cell | **50 m** | |
| map, 63 × 63 cells | **3.15 km × 3.15 km** (~10 km²) | a sensible search area for 3 drones |
| 1 cell per tick | **5 m/s** | typical search speed |
| camera strip, 3 cells | 150 m | roughly a thermal camera's footprint from ~100 m up (UK limit 120 m) |
| beacon heard from ~4 cells | ~200 m | between an avalanche transceiver (tens of m) and phone/locator signals (up to km) |
| radio range, 20 cells | 1 km | conservative; real links reach several km with line of sight, and terrain blocks them |
| **battery, 240 ticks** | **40 minutes** | a search quadcopter (e.g. DJI Matrice 30T ~41 min) |
| climbing | extra battery per metre climbed | terrain spans 600 m; climbing at 3 m/s |
| return-home trigger | the real cost of the flight home (climbs included) + a **15% reserve** | pilots keep a margin for wind and cold |
| battery swap | ~4 minutes at base | |

The UI shows minutes and "minutes of flight left" rather than ticks.

### Low battery: go home, swap, resume

When the battery only covers the trip home plus the reserve, the drone **remembers what it was doing and where** (sector, lawnmower row, position), flies home, swaps the battery (~4 min) and relaunches **with its map intact**, then goes back to exactly that task, unless it's finished or (as far as it knows) a teammate took it over meanwhile. Log line: *"Fresh battery; resuming sector E where I left off at (22, 32) (69% searched)."* The other modes get the same: the central hub sends a relaunched drone back to its unfinished sector, and "no coordination" drones resume their own search. Once low battery fires, nothing can redirect the drone except home (safety lock). No drone ran flat in any benchmark run.

### Why there are no scheduled meetups any more

The design originally had all drones fly to a central rendezvous every 25 minutes. With realistic 40-minute batteries that cost more than it gained, measured over 15 maps:

| Ours, 15 maps | Beacon working | Beacon off | Drone 1 killed | 90% packet loss |
|---|---|---|---|---|
| Meetup every 25 min | 45 min | 109 min | 57 min | 41 min |
| Meetup every 50 min | 42 min | 90 min | 55 min | 44 min |
| **No scheduled meetups** | **33 min** | **90 min** | **45 min** | **33 min** |

All variants found the hiker on 15/15 maps. Drones already come back to base every battery and sync with whoever they meet there or on the way, so meetups are now **off by default** (`RENDEZVOUS_INTERVAL = None`; set a number of ticks to turn them back on). A teammate silent for longer than a battery lasts (`LOST_TIMEOUT`, 50 min) is presumed lost. When the hiker is found, the finder flies to the nearest unaware teammate or back to base, whichever is closer: everyone comes back to base to swap batteries, so they all hear the news there.

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
- **Time:** 1 tick = 10 seconds (see *Real-world scale*). Scheduled rendezvous are off by default.
- **Battery:** 40 minutes of level flight; climbing costs extra. `LOW_BATTERY` fires when the battery only covers the real flight home (climbs included) plus a 15% reserve. The drone then swaps its battery at base and resumes where it left off (see above).

### How a drone searches (two sensors)

1. **Sweep for the beacon, not with the camera.** The beacon is heard from about 4 cells away (3.5 even through a hill), but the camera only sees 1 cell. So drones fly lawnmower lanes **7 cells apart** and mark every cell within `BEACON_CLEAR_RADIUS = 3` as *cleared*: if the hiker were there, the beacon would have been heard. Lanes that are already fully cleared (by this drone or, via gossip, a teammate) are skipped.
2. **Close in with the camera.** On `SIGNAL_DETECTED` the drone switches to `FOLLOW_SIGNAL` and searches around the readings at camera resolution (`SENSOR_RADIUS = 1`) until it sees the hiker.

Measured on the 42 × 42 map, this found the hiker **about 40% sooner** than camera-width lanes (56 vs 98 ticks with perfect radio, 74 vs 132 with a drone killed), with no missed hikers in 150 runs. The cost: more cells are swept twice, because a 7-wide footprint overlaps more when drones transit over each other's ground.

### When the beacon is off, or dies

Pick **Hiker's beacon** in the UI: *Working*, *Off the whole time* (dead phone), or *Dies right after first contact* (`BEACON_DIES_AFTER` ticks after a drone first hears it).

The wide beacon lanes only work if the beacon does, so drones keep two kinds of evidence: cells **cleared by the beacon** (with the tick they were cleared) and cells **seen by the camera**.

1. **Noticing it died.** A drone following a signal that goes quiet compares two explanations of its readings, each with its best-fitting hiker position: *"the beacon is alive and I'm somewhere quiet"* vs *"it died after my last reading"*. It declares `BEACON_LOST` only when the second is over e⁶ (~400×) more likely, so flying behind a hill doesn't trigger it. With the beacon working it never fired in 30 test runs.
2. **Estimating where the hiker is (Bayesian).** The drone knows how the signal behaves (fades over `BEACON_RANGE`, halves behind terrain, known noise), so for every nearby cell it computes how likely its actual readings would be if the hiker were there, including the readings where it heard *nothing*, which rule out cells that would have been loud. The result is a probability map (orange on the canvas, dashed circle = uncertainty). In testing the centre was **0–3 cells** from the real hiker.
3. **Searching the estimate.** `SEARCH_LAST_SIGNAL`: visit the most likely unseen cells first with the camera (probability ÷ distance), crossing them off as it looks, until less than `ESTIMATE_DONE_MASS` of the probability is left.
4. **Falling back to a camera sweep.** The team switches to **camera mode**: lanes 3 wide instead of 7. Cells the beacon cleared *while it was still working* (before the last time anyone heard it) stay cleared; everything else is searched again by sight. Camera mode spreads by gossip, so teammates switch too.
5. **Beacon never heard at all.** When the whole mountain has been swept with no signal (`BEACON_SILENT`), the beacon must be off: if anyone did hear it earlier, they search the estimate first; otherwise the whole mountain is re-searched by camera.

Only **confirmed** readings (two within 5 ticks) count as "the beacon was alive then"; otherwise a single noise spike could make the team trust old beacon clearances and skip the hiker.

| Beacon (30 maps each) | Found | Avg tick | Worst |
|---|---|---|---|
| Working | 30/30 | 151 | 268 |
| Dies right after first contact | 30/30 | 188 | 889 |
| Off the whole time | 30/30 | 360 | 889 |

The 889-tick cases are runs where the beacon died before a second reading confirmed it, which is indistinguishable from noise, so the team correctly treated it as "never heard" and searched everything by camera (with battery swaps). Demo: seed 3, *Dies right after first contact*: drone 2 hears it at tick 73, declares it dead at 79 with an estimate 2 cells from the hiker, and finds them at tick 82.

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
RENDEZVOUS_INTERVAL = None  # scheduled meetups off (measured: they cost more than they gain with 40-min batteries)
RENDEZVOUS_POINT = (31, 31)
RENDEZVOUS_SKIP_IF_HEARD = 15
LOST_TIMEOUT = 300        # 50 min of silence (longer than a battery lasts) = presumed lost
MISSED_MEETUP_WAIT = 5    # ticks to wait at the rendezvous for a missing drone
START_OPTIONS = ["random", "S", "N", "W", "E", "NW", "NE", "SW", "SE"]  # launch point choices
TICK_SECONDS = 10         # 1 tick = 10 s
CELL_METERS = 50          # 1 cell = 50 m
BATTERY_MINUTES = 40      # -> BATTERY_TICKS = 240, BATTERY_DRAIN = 1/240 per tick
BATTERY_RESERVE_TICKS = 36  # land with ~15% left
TERRAIN_RELIEF_M = 600    # height 0..1 = 0..600 m
CLIMB_RATE_MS = 3         # climbing costs extra battery
SIGNAL_FOLLOW_LIMIT = 40  # ticks before giving up on a signal
RELAY_HOVER_TICKS = 8     # how long CLIMB_TO_RELAY waits on high ground
RECHARGE_TICKS = 24       # battery swap at base (~4 min)
BEACON_MODES = ["on", "off", "dies"]
BEACON_DIES_AFTER = 2     # 'dies' mode: ticks after first contact
BEACON_MODEL_NOISE = 0.07 # noise the drones assume when estimating
ESTIMATE_DONE_MASS = 0.05 # stop the estimate search when <5% of the probability is unchecked
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
    "searched_cells": {(0, 0), ...},          # beacon-cleared cells
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
- `BEACON_LOST`: a signal it was following went silent and the evidence says the beacon died
- `BEACON_SILENT`: the whole mountain was swept with no beacon: it is off
- `ESTIMATE_EXHAUSTED`: the likely spots from the estimate were all checked
- `RELAUNCHED`: back in the air after a battery swap
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
| `SEARCH_LAST_SIGNAL` | none | Beacon died: camera-search the estimated position, most likely cells first |
| `GO_TO_RENDEZVOUS` | none | Fly to the rendezvous point and wait for teammates |
| `GO_TO_TEAMMATE` | drone id | Fly to a teammate's last known position (e.g. to report a find) |
| `CLIMB_TO_RELAY` | none | Fly to the best nearby high ground and hover there to get line of sight over the hills |
| `RETURN_HOME` | none | Fly back to base and land |
| `HOVER` | none | Baseline only: wait for instructions from the hub |

---

## The agents

```python
# agent.py
def decide(state) -> (action, argument, reason)
```

Each drone runs its own copy of `decide`, called only on events. It is **rules-based**: instant, deterministic, explainable (every decision comes with a one-line reason in the log), and it needs no network, which matters for drones in mountains with no signal. It is a pure function of that one drone's `state`, and `tests/test_isolation.py` checks this.

> An earlier version also had an optional Claude (LLM) layer choosing between the rules' options. Neptune North confirmed LLMs aren't required, so it was removed: the rules carried every result anyway, and removing it means no API key, no latency and fully repeatable runs.

---

## Resilience rules (what makes it work without comms)

1. **Pre-assigned sectors:** each drone owns 3 sectors before launch.
2. **Priority order:** own sectors first, then other drones' sectors in reverse order. Minimal overlap, zero comms needed.
3. **Assume teammates follow their plan:** a sector a live teammate is searching, or will reach next in its own plan, counts as claimed.
4. **Gossip on contact:** whenever two drones can talk, they merge everything they know.
5. **Scheduled rendezvous (optional, off by default; see *Why there are no scheduled meetups*):** at every multiple of `RENDEZVOUS_INTERVAL`, drones fly to `RENDEZVOUS_POINT`. The meetup is complete as soon as everyone presumed alive is in radio contact (no need to be on the exact cell). A drone **skips** the trip if it is already in contact with every live teammate.
6. **Deterministic re-split:** at `MEETUP_COMPLETE` every drone has the same knowledge, so each one runs the same assignment (drones in id order, each keeps its unfinished sector, others take the best unclaimed sector by their own priority order). No negotiation needed.
7. **Missed meetup:** a drone not in contact within `MISSED_MEETUP_WAIT` ticks is marked `missed_at`, presumed lost, and its unsearched sectors become unclaimed. If it shows up again in gossip later, its newer `last_heard` brings it back.
8. **Lost-contact timeout:** a teammate silent for `LOST_TIMEOUT` ticks is presumed lost.
   - **If a drone dies,** nobody flies off to look for it: the survivors notice at the next meetup (or the timeout), spread the news by gossip, and fold its unsearched sectors into their own priority order. Until then they assume it is still searching its own sectors, so the takeover can lag by up to one `RENDEZVOUS_INTERVAL`.
9. **Found it? Tell someone:** the finder goes to the nearest teammate who doesn't know yet, or the rendezvous, whichever is closer, instead of waiting.

---

## Comparison modes (Mode dropdown)

| Mode | What it is | Why it's there |
|---|---|---|
| **Ours** | Pre-launch plan + radio sharing + rendezvous | This project |
| **Central hub** | A naive controller at base assigns sectors; drones hover when they can't reach it | "What if there were a hub?" Breaks when the radio does |
| **No coordination** | No plan, no radio: each drone searches the whole mountain in its own random order | "Is teamwork worth anything?" Shows what the plan and the radio each add |

"Pre-launch plan only" = **Ours** with the radio fully dead (100% packet loss): coordination without talking.

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
- **Belief error**: how far (in cells) each drone's picture of where its teammates are is from reality, now and averaged over the run

---

## Web UI

### Side-by-side comparison (`/compare`)

Three maps at once: **Ours**, **Central hub** and **No coordination**, on the same seed, launch point and beacon setting. One set of controls drives all three in lockstep: Start/Step/Speed, packet loss, comms blackout and *Kill drone N* hit every team at the same tick, so it's a fair race. Each column shows a live status badge (searching / FOUND at tick X / not found), a **found first** highlight, key numbers (time to find, when all drones knew, drones alive, searched, searched twice, messages lost), a **battery bar per drone** (charge, colour by level, a tick marking the lowest it has been, and status: flying / heading home / swapping battery / landed / ran flat, plus swap count) and its latest decisions. On the maps, a ring around each drone shows its battery.

Default seed **23** is typical of the benchmark averages: ours finds the hiker at tick ~111, no coordination at ~287, the central hub not at all, and that ordering holds when drone 1 is killed at tick 40. Single runs vary (a team with no plan can get lucky and start next to the hiker), which is why the Evidence panel averages many maps.

### Single run (`/`)


- **Demo seed 2** (the default, launch point random → near the top-left corner): drone 1 owns the diagonal `A E I` and the hiker is in the far corner, sector I. Kill drone 1 around tick 40: drones 0 and 2 take over its wedge and still find the hiker (tick ~250); the baseline never does.
- **Launch from:** pick a wall or corner (or random) to show the plan adapting; the plan for the current launch point is shown under the controls.
- **Hiker's beacon:** working / dies right after first contact / off. Drone cards show *camera search* and the estimate's uncertainty; the canvas shows the estimate in orange.
- **Canvas:** terrain height heatmap with contours, unsearched cells dimmed, a dot per searched cell in the colour of the drone that searched it, drones, comms links (green = delivered, red = dropped), rendezvous point, base, hiker (hidden until found, or "reveal").
- **View as drone N (knows & believes):** shows only what that drone knows (map merging), plus **ghosts**: a dashed ring where it *believes* each teammate is, labelled with what it thinks they're doing and how old that information is, joined by a dotted line to where the teammate really is. When the radio drops the ghosts freeze and drift from reality; when contact returns they snap back. This is what each drone actually decides from.
- **Controls:** start/pause, step, speed, reset (seed), packet-loss slider, comms on/off, kill drone 0/1/2, mode (ours / central hub / no coordination), launch point, beacon, view.
- **Reasoning log:** `[tick] Drone N (source): ACTION arg: reason`.
- **Evidence panel:** loads the saved benchmark instantly: the *what teamwork is worth* table with a one-line takeaway, two resilience charts (hiker found % and time to find vs packet loss, for all three modes, with hover tooltips), and the full failure-scenario table. *Re-run benchmark* recomputes it (~1 min for 5 maps).

### API
| Method | Path | Body / effect |
|---|---|---|
| GET | `/` | serves `static/index.html` |
| GET | `/world` | static grid: ridges, sectors, base, rendezvous |
| GET | `/state?view=-1` | full sim state (`view` = drone id to show only its knowledge) |
| POST | `/control/start` / `/control/pause` / `/control/step` | |
| POST | `/control/reset` | `{"seed": 42, "mode": "ours", "start": "random", "beacon": "on"}` (`start`: `random`, `S`, `N`, `W`, `E`, `NW`, `NE`, `SW`, `SE`; `beacon`: `on`, `off`, `dies`) |
| POST | `/control/speed` | `{"ticks_per_second": 10}` |
| POST | `/control/packet_loss` | `{"value": 0.5}` |
| POST | `/control/comms` | `{"enabled": false}` |
| POST | `/control/kill` | `{"drone_id": 2}` |
| GET | `/bench` | saved benchmark results (`bench_results.json`) |
| POST | `/bench` | `{"seeds": 5}` re-runs all benchmarks and saves them |
| GET | `/compare` | the side-by-side page |
| GET | `/compare/world`, `/compare/state` | shared map; state of all three teams |
| POST | `/compare/control/{start,pause,step,reset,speed,packet_loss,comms,kill,reveal}` | same bodies as the single-run controls; applied to all three teams |

---

## Team rules

- **Pull before you start, push often.**
- **Code freeze at 17:15.** After that, only bug fixes.

## Known limitations (be honest in the pitch)

- During a long blackout, two drones may both take over the same lost sector, which duplicates work until they reconnect.
- A lone drone following a noisy signal can head the wrong way for a while.
- Without a beacon the search is slow: about 2.4× longer on average, because the camera sees 3-wide lanes instead of 7, and the team only realises the beacon is off after a full beacon sweep. If rescuers already know the phone is dead, starting straight in camera mode would save that first sweep.
- The estimate assumes the hiker didn't move after the beacon died.
- Without scheduled meetups, a dead drone is only presumed lost after `LOST_TIMEOUT` (50 min) of silence, so its sectors are taken over later.
- No wind or cold (both shorten real flight time), and the camera never misses a hiker it flies over.
- A drone that is merely out of range at meetup time is presumed lost until it is heard again; its sectors may be searched twice.
- The signal and terrain model is simplified: line of sight over a heightmap, a hard radio range, and no battery cost for climbing.
