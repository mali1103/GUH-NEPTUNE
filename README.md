# Mountain Rescue Drones 🚁⛰️

Three autonomous drones search a mountain for a lost hiker, with **no central controller**. They agree a plan before take-off, make their own decisions in the air, share what they know whenever they're in radio range, and keep searching when the radio fails, a drone is lost, or the hiker's phone dies.

Built at **GreatUniHack 2026** for the **Neptune North challenge**: *design a system where multiple agents keep working towards a shared goal, even when communication fails.*

---

## Quick start

```bash
python -m venv .venv
# Windows: .venv\Scripts\Activate.ps1    Mac/Linux: source .venv/bin/activate
pip install -r requirements.txt
uvicorn server:app --reload
```

Then open:
- **http://localhost:8000**: one team in full detail
- **http://localhost:8000/compare**: three designs racing side by side on the same map

No API keys, internet access or external services are needed.

---

## How it works

**The world**
- A randomly generated 3.15 km × 3.15 km mountain (63 × 63 cells of 50 m), shown as a height heatmap.
- The hiker is hidden somewhere on it, possibly carrying a radio beacon (working, dying, or off).
- The drones launch from a base on the edge of the map.

**Before launch: a plan that needs no radio**
- The mountain's 9 sectors are split into three wedges fanning out from base, one per drone.
- Every drone knows everyone's plan and search order, so they can split the work without talking.

**Searching**
- Drones fly "lawnmower" lanes spaced to the beacon's range (~200 m), skipping ground already searched.
- When a drone hears the beacon, it closes in with its camera to confirm the find.

**Talking, when they can**
- Two drones can talk only within **1 km**, with a **clear line over the terrain**, and if the message isn't lost.
- When they can, they share maps, signal readings, what they know about teammates, and whether the hiker has been found.
- Merges never conflict, so late or repeated messages can't break anything.

**When things go wrong**
- **Radio lost:** each drone carries on with the plan and its own map, and syncs when contact returns.
- **Drone lost:** after 50 minutes of silence (longer than a battery lasts), teammates presume it's gone and take over its sectors.
- **Low battery:** the drone remembers what it was doing, flies home with a 15% reserve, swaps its battery (~4 min), then goes back to where it left off.
- **Beacon dies:** a statistical test tells "beacon dead" apart from "hill in the way". The drone then builds a Bayesian probability map of the hiker's position from its readings, including where it heard nothing, and searches the most likely spots first.
- **Beacon off:** once the whole mountain has been swept with no signal, drones switch to narrower camera lanes to search by sight.

**Decisions** come from a small, rules-based agent on each drone. It's instant, works offline, and explains itself: every decision appears in the log with a reason, e.g. *"Fresh battery; resuming sector E where I left off (69% searched)."*

---

## Three designs compared

| Design | How it works |
|---|---|
| **Ours (decentralised)** | Pre-launch plan + independent decisions + map sharing when in range |
| **Central hub** | Base assigns sectors; a drone that can't reach base hovers and waits for orders |
| **No coordination** | No plan, no radio: each drone searches the whole mountain alone |

### Results (average of 10 maps, 40-minute batteries)

| Design (radio working) | Hiker found | Time to find | All drones know | Ground searched twice | Total flight time |
|---|---|---|---|---|---|
| No coordination | 10/10 | 56 min | 173 min | 70% | 329 min |
| Plan only, radio dead | 10/10 | 42 min | 145 min | 48% | 282 min |
| **Ours (plan + radio)** | **10/10** | **39 min** | **46 min** | **23%** | **135 min** |
| Central hub | 4/10 | 36 min (when found) | 51 min | 31% | 413 min |

- **Teamwork pays:** the plan alone makes the team 25% faster than drones searching alone. Sharing maps then halves duplicated searching and gets the news to every drone 3× sooner.
- **It doesn't stop when communication does.** Ours found the hiker on every map:
  - at every level of packet loss from 0 to 100%
  - in a total radio blackout
  - with a drone destroyed mid-search
  - when the hiker's beacon died or was never on

  The central hub found it on 0 of 10 maps once the radio was gone.
- **Efficient:** ours used less than half the flight time of the other designs. No drone ever ran out of battery.

---

## Realistic scale

| | Value |
|---|---|
| Time step | 10 seconds |
| Cell size | 50 m (map 3.15 km across) |
| Flight speed | 5 m/s |
| Battery | 40 minutes, with extra energy for climbing |
| Return-home trigger | the cost of the flight home plus a 15% reserve |
| Battery swap | ~4 minutes |
| Drone-to-drone radio | 1 km, needs clear line of sight over terrain |
| Beacon detection | ~200 m (halved behind terrain) |
| Camera footprint | ~150 m wide |

---

## What you can do in the app

- **Break things:** packet-loss slider, total radio blackout, destroy any drone, or choose whether the hiker's beacon works, dies or is off.
- **Change the setup:** pick the seed (map) and the launch point (any wall, any corner, or random).
- **See what a drone believes:** view the map as one drone sees it, with "ghosts" showing where it *thinks* its teammates are and how old that information is.
- **Compare designs:** run all three side by side under identical failures, with battery bars for every drone.
- **Evidence panel:** charts of how each design copes as the radio gets worse, plus a table of every failure scenario.

Try seed **23** on the compare page, press **Start**, then **Kill 1** and **Comms OFF**.

---

## Testing and benchmarks

```bash
python tests/test_isolation.py   # proves drones share nothing except over the radio
python bench.py --seeds 10       # benchmark all designs and failure scenarios
python bench.py --show           # print the saved results
```

The isolation test checks four things:
- with the radio off, each drone knows only what its own sensors saw
- every message delivered was physically possible (in range, clear line of sight, both drones alive)
- line of sight is the same in both directions
- every decision depends only on that drone's own knowledge

---

## Project structure

| File | Purpose |
|---|---|
| `world.py` | Terrain, hiker, beacon, line of sight, pre-launch plan, settings |
| `drone.py` | A drone: movement, battery, sensing, map sharing, events |
| `agent.py` | Each drone's rules-based decision-making |
| `baseline.py` | The central-hub design, for comparison |
| `sim.py` | Simulation loop, radio, metrics, benchmarks, side-by-side runner |
| `server.py` | Web server and API (FastAPI) |
| `static/` | The two web pages (vanilla JavaScript, HTML canvas) |
| `bench.py` | Benchmark runner |
| `tests/` | Independence test |

---

## Limitations

- No wind or cold, both of which shorten real flight time.
- The camera never misses a hiker it flies over, and the hiker doesn't move.
- Terrain is a generated heightmap rather than real elevation data.
- During a long blackout, two drones may both take over the same lost drone's sectors.
- Without a beacon the search takes over 2× longer, because the camera covers narrower lanes than the beacon.
