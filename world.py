"""World model: random heightmap terrain, hidden hiker, beacon signal, line of sight, sectors.

Everything the drones are allowed to know up front (terrain, sectors, base,
rendezvous plan) lives here as constants/helpers. The hiker position is only
exposed through `beacon_reading` and `hiker_within`.
"""
import math
import random
from functools import lru_cache

# ---------------------------------------------------------------- constants
# Real-world scale: 1 tick = 10 s, 1 cell = 50 m, so 1 cell per tick = 5 m/s (a typical
# search speed) and the 63 x 63 map is 3.15 km x 3.15 km (~10 km2).
TICK_SECONDS = 10
CELL_METERS = 50
GRID_SIZE = 63
SECTOR_SIZE = 21
SECTOR_LABELS = "ABCDEFGHI"
NUM_DRONES = 3
SENSOR_RADIUS = 1         # camera: close search when homing in on a signal
COMMS_RANGE = 20          # cells, drone to drone
BASE_COMMS_RANGE = 46     # base station mast (baseline mode only)
PACKET_LOSS = 0.0         # 0.0 - 1.0, controlled by a slider
BEACON_RANGE = 5
BEACON_NOISE = 0.05
SIGNAL_THRESHOLD = 0.15
BEACON_CLEAR_RADIUS = 3   # a pass within this distance would have heard the beacon (even through a hill)
FIND_RADIUS = 1
# Scheduled meetups: None = off. With real 40-minute batteries, drones return to base every
# battery anyway and sync with whoever they meet; dedicated meetup flights cost more than they
# gained in every tested scenario (e.g. 33 vs 45 min to find). Set e.g. 150 to turn them back on.
RENDEZVOUS_INTERVAL = None
RENDEZVOUS_POINT = (31, 31)
RENDEZVOUS_SKIP_IF_HEARD = 15  # skip the trip if every live teammate was heard this recently
LOST_TIMEOUT = 300        # 50 min of silence (longer than a battery lasts) = presumed lost
MISSED_MEETUP_WAIT = 5    # ticks to wait at the rendezvous for a missing drone
BATTERY_MINUTES = 40      # a search quadcopter (e.g. DJI Matrice 30T ~41 min)
BATTERY_TICKS = BATTERY_MINUTES * 60 // TICK_SECONDS   # 240 ticks of level flight
BATTERY_DRAIN = 1 / BATTERY_TICKS
BATTERY_RESERVE_TICKS = 36  # land with ~15% (6 min) left: real pilots keep a margin for wind and cold
TERRAIN_RELIEF_M = 600    # terrain height 0..1 spans 600 m from valley floor to highest peak
CLIMB_RATE_MS = 3         # climbing is slow: every metre climbed costs extra flight energy
SIGNAL_FOLLOW_LIMIT = 40
RELAY_HOVER_TICKS = 8
RECHARGE_TICKS = 24       # battery swap at base before relaunching (4 min)

# Beacon failure modes (pick in the UI): the hiker's beacon is on, was never on (dead phone),
# or dies BEACON_DIES_AFTER ticks after a drone first hears it.
BEACON_MODES = ["on", "off", "dies"]
BEACON_DIES_AFTER = 2       # ticks after first contact: just enough readings to estimate from
BEACON_MODEL_NOISE = 0.07  # noise the drones assume when estimating (true noise + slack)
ESTIMATE_DONE_MASS = 0.05  # stop the estimate search when <5% of the probability is left unseen
MAX_TICKS = 2000          # ~5.5 hours

# Terrain: heights are 0.0 (valley floor) .. 1.0 (highest peak).
MAP_SCALE = GRID_SIZE / 30  # terrain features scale with the map
FLIGHT_CLEARANCE = 0.04   # drones hug the terrain at this height above ground
MAST_HEIGHT = 0.15        # base station mast (baseline mode)
NUM_PEAKS = (8, 12)       # round mountains
NUM_RIDGES = (3, 5)       # long ridgelines: the main radio walls
RELAY_SEARCH_RADIUS = 14  # CLIMB_TO_RELAY looks this far for high ground

# Launch point: anywhere on the map edge (random per seed, or pick one in the UI).
# The pre-launch sector plan is derived from it: see make_plan().
START_OPTIONS = ["random", "S", "N", "W", "E", "NW", "NE", "SW", "SE"]


# ---------------------------------------------------------------- geometry
def in_bounds(c):
    return 0 <= c[0] < GRID_SIZE and 0 <= c[1] < GRID_SIZE


def cheb(a, b):
    """Chebyshev distance = ticks needed to fly from a to b (8-way moves)."""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def step_toward(pos, target):
    dx = (target[0] > pos[0]) - (target[0] < pos[0])
    dy = (target[1] > pos[1]) - (target[1] < pos[1])
    return (pos[0] + dx, pos[1] + dy)


@lru_cache(maxsize=None)
def cells_in_radius(c, r):
    return tuple(
        (x, y)
        for x in range(c[0] - r, c[0] + r + 1)
        for y in range(c[1] - r, c[1] + r + 1)
        if in_bounds((x, y))
    )


@lru_cache(maxsize=None)
def cells_in_disk(c, r):
    """Cells within Euclidean distance ~r (the beacon 'heard it if it was there' footprint)."""
    lim = r * r + 1
    return tuple(p for p in cells_in_radius(c, r)
                 if (p[0] - c[0]) ** 2 + (p[1] - c[1]) ** 2 <= lim)


@lru_cache(maxsize=None)
def bresenham(a, b):
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx + dy
    out = []
    while True:
        out.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return tuple(out)
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


# ---------------------------------------------------------------- sectors
def sector_of(c):
    return SECTOR_LABELS[(c[1] // SECTOR_SIZE) * 3 + c[0] // SECTOR_SIZE]


@lru_cache(maxsize=None)
def sector_bounds(label):
    i = SECTOR_LABELS.index(label)
    x0, y0 = (i % 3) * SECTOR_SIZE, (i // 3) * SECTOR_SIZE
    return x0, y0, x0 + SECTOR_SIZE, y0 + SECTOR_SIZE


@lru_cache(maxsize=None)
def sector_cells(label):
    x0, y0, x1, y1 = sector_bounds(label)
    return frozenset((x, y) for x in range(x0, x1) for y in range(y0, y1))


def sector_center(label):
    x0, y0, x1, y1 = sector_bounds(label)
    return ((x0 + x1 - 1) // 2, (y0 + y1 - 1) // 2)


@lru_cache(maxsize=None)
def sector_rows(label, R=BEACON_CLEAR_RADIUS):
    """Lawnmower rows: each row's band covers rows y-R..y+R. R = beacon lane half-width
    normally, or the camera radius when searching by sight."""
    _, y0, _, y1 = sector_bounds(label)
    rows, r = [], y0 + R
    while True:
        rr = min(r, y1 - 1 - R)
        rows.append(rr)
        if rr + R >= y1 - 1:
            return tuple(rows)
        r += 2 * R + 1


def perimeter_cells():
    g = GRID_SIZE - 1
    top = [(x, 0) for x in range(g)]
    right = [(g, y) for y in range(g)]
    bottom = [(x, g) for x in range(g, 0, -1)]
    left = [(0, y) for y in range(g, 0, -1)]
    return top + right + bottom + left


def launch_point(start, rng_cell):
    """Named launch points, or 'random' = the seed's random cell on the map edge."""
    g = GRID_SIZE - 1
    m = g // 2
    named = {"S": (m, g), "N": (m, 0), "W": (0, m), "E": (g, m),
             "NW": (0, 0), "NE": (g, 0), "SW": (0, g), "SE": (g, g)}
    return named.get(start, rng_cell)


def make_plan(base):
    """Pre-launch plan for any launch point on the edge.

    Sectors are sorted by bearing from base (relative to 'into the map') and cut into
    three wedges fanning out from base; each drone owns one wedge, nearest sector first.
    Priority order: own sectors, then the other drones' sectors in reverse (furthest first),
    nearest drone first, interleaved. Everyone can compute this before launch: zero comms.
    """
    c = (GRID_SIZE - 1) / 2
    nx, ny = c - base[0], c - base[1]           # direction into the map

    def bearing(s):
        vx, vy = sector_center(s)[0] - base[0], sector_center(s)[1] - base[1]
        return round(math.atan2(nx * vy - ny * vx, nx * vx + ny * vy), 6)

    order = sorted(SECTOR_LABELS, key=lambda s: (bearing(s), dist(base, sector_center(s)), s))
    k = len(SECTOR_LABELS) // NUM_DRONES
    own = {i: sorted(order[i * k:(i + 1) * k], key=lambda s: (dist(base, sector_center(s)), s))
           for i in range(NUM_DRONES)}
    prio = {}
    for i in own:
        others = sorted((j for j in own if j != i), key=lambda j: (abs(j - i), j))
        rev = [list(reversed(own[j])) for j in others]
        p = list(own[i])
        for n in range(k):
            for r in rev:
                p.append(r[n])
        prio[i] = p
    return own, prio


# ---------------------------------------------------------------- world
class World:
    def __init__(self, seed=42, start="random", beacon="on"):
        self.seed = seed
        rng = random.Random(seed)
        edge = perimeter_cells()[rng.randrange(len(perimeter_cells()))]
        self.base = launch_point(start, edge)      # launch point, always on the map edge
        self.own_sectors, self.priority = make_plan(self.base)
        self.height = self._make_terrain(rng, self.base)  # height[y][x] in 0..1
        self.hiker = self._place_hiker(rng)
        self.noise_rng = random.Random(seed * 7919 + 1)
        self._relay_cache = {}
        self._home_cache = {}
        self.beacon_mode = beacon
        self.beacon_alive = beacon != "off"
        self.beacon_first_heard = None       # tick a drone first got a real reading
        self.beacon_died_at = 0 if beacon == "off" else None

    # ------------------------------------------------------------ terrain
    @staticmethod
    def _make_terrain(rng, base):
        g, s = GRID_SIZE, MAP_SCALE
        h = [[0.0] * g for _ in range(g)]

        def blob(cx, cy, amp, sx, sy, theta):
            ct, st = math.cos(theta), math.sin(theta)
            for y in range(g):
                for x in range(g):
                    u = (x - cx) * ct + (y - cy) * st
                    v = -(x - cx) * st + (y - cy) * ct
                    h[y][x] += amp * math.exp(-0.5 * ((u / sx) ** 2 + (v / sy) ** 2))

        for _ in range(rng.randint(*NUM_PEAKS)):          # round mountains
            blob(rng.uniform(0, g), rng.uniform(0, g), rng.uniform(0.5, 1.0),
                 rng.uniform(2.5, 5) * s, rng.uniform(2.5, 5) * s, 0)
        for _ in range(rng.randint(*NUM_RIDGES)):         # long ridgelines, any orientation
            blob(rng.uniform(4 * s, g - 4 * s), rng.uniform(4 * s, g - 4 * s), rng.uniform(0.7, 1.0),
                 rng.uniform(6, 11) * s, rng.uniform(1.3, 2.2) * s, rng.uniform(0, math.pi))
        for _ in range(int(25 * s * s)):                  # rough texture
            blob(rng.uniform(0, g), rng.uniform(0, g), rng.uniform(0.05, 0.15),
                 rng.uniform(1, 2), rng.uniform(1, 2), 0)
        blob(base[0], base[1], -0.8, 5 * s, 5 * s, 0)    # base sits in a valley

        lo = min(min(r) for r in h)
        hi = max(max(r) for r in h)
        return tuple(tuple(round((v - lo) / (hi - lo), 3) for v in row) for row in h)

    def _place_hiker(self, rng):
        while True:
            c = (rng.randrange(GRID_SIZE), rng.randrange(GRID_SIZE))
            if self.h(c) < 0.85 and cheb(c, self.base) >= GRID_SIZE // 3:
                return c

    def h(self, c):
        return self.height[c[1]][c[0]]

    def altitude(self, c):
        """Flying altitude of a drone over cell c."""
        return self.h(c) + FLIGHT_CLEARANCE

    # ------------------------------------------------------------ line of sight
    def line_of_sight(self, a, b, za=None, zb=None):
        """True if the straight line from a (altitude za) to b (altitude zb) clears the terrain.
        Defaults to drone flying altitude at both ends."""
        za = self.altitude(a) if za is None else za
        zb = self.altitude(b) if zb is None else zb
        if b < a:  # same answer both ways: the raster line A->B can differ from B->A
            a, b, za, zb = b, a, zb, za
        pts = bresenham(a, b)
        n = len(pts) - 1
        if n <= 1:
            return True
        for i in range(1, n):
            if self.h(pts[i]) > za + (zb - za) * i / n:
                return False
        return True

    def mast_altitude(self, c):
        return self.h(c) + MAST_HEIGHT

    def beacon_model(self, pos, cell):
        """Expected (noise-free) strength at `pos` if the hiker were at `cell`.
        This is public physics + the terrain map: drones may use it to estimate."""
        s = max(0.0, 1.0 - dist(pos, cell) / BEACON_RANGE)
        if s > 0 and not self.line_of_sight(pos, cell, zb=self.h(cell) + 0.01):
            s *= 0.5  # mountain between drone and hiker
        return s

    def beacon_reading(self, pos, tick=0):
        if self.beacon_mode == "dies" and self.beacon_alive and self.beacon_first_heard is not None                 and tick >= self.beacon_first_heard + BEACON_DIES_AFTER:
            self.beacon_alive = False
            self.beacon_died_at = tick
        true = self.beacon_model(pos, self.hiker) if self.beacon_alive else 0.0
        s = true + self.noise_rng.gauss(0, BEACON_NOISE)
        if s < SIGNAL_THRESHOLD:
            return 0.0
        if true > 0 and self.beacon_first_heard is None:
            self.beacon_first_heard = tick
        return round(s, 3)

    def hiker_within(self, pos, r=FIND_RADIUS):
        return cheb(pos, self.hiker) <= r

    def climb_cost(self, a, b):
        """Extra battery (in ticks of level flight) for flying from cell a to b uphill."""
        rise = max(0.0, self.h(b) - self.h(a)) * TERRAIN_RELIEF_M
        return rise / CLIMB_RATE_MS / TICK_SECONDS

    def cost_home(self, pos):
        """Battery (in ticks of flight) to fly from pos straight back to base, climbs included."""
        if pos not in self._home_cache:
            cost, here = 0.0, pos
            while here != self.base:
                nxt = step_toward(here, self.base)
                cost += 1 + self.climb_cost(here, nxt)
                here = nxt
            self._home_cache[pos] = cost
        return self._home_cache[pos]

    def relay_point(self, pos):
        """Best nearby high ground: tall, but not too far to fly to."""
        if pos not in self._relay_cache:
            self._relay_cache[pos] = max(
                cells_in_radius(pos, RELAY_SEARCH_RADIUS),
                key=lambda c: (self.h(c) - 0.03 * cheb(pos, c), -c[0], -c[1]))
        return self._relay_cache[pos]
