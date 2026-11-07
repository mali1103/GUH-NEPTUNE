"""World model: random heightmap terrain, hidden hiker, beacon signal, line of sight, sectors.

Everything the drones are allowed to know up front (terrain, sectors, base,
rendezvous plan) lives here as constants/helpers. The hiker position is only
exposed through `beacon_reading` and `hiker_within`.
"""
import math
import random
from functools import lru_cache

# ---------------------------------------------------------------- constants
GRID_SIZE = 42
SECTOR_SIZE = 14
SECTOR_LABELS = "ABCDEFGHI"
NUM_DRONES = 3
SENSOR_RADIUS = 1         # camera: close search when homing in on a signal
COMMS_RANGE = 15          # cells, drone to drone
BASE_COMMS_RANGE = 31     # base station mast (baseline mode only)
PACKET_LOSS = 0.0         # 0.0 - 1.0, controlled by a slider
BEACON_RANGE = 5
BEACON_NOISE = 0.05
SIGNAL_THRESHOLD = 0.15
BEACON_CLEAR_RADIUS = 3   # a pass within this distance would have heard the beacon (even through a hill)
FIND_RADIUS = 1
RENDEZVOUS_INTERVAL = 60  # ticks
RENDEZVOUS_POINT = (21, 21)
RENDEZVOUS_SKIP_IF_HEARD = 15  # skip the trip if every live teammate was heard this recently
LOST_TIMEOUT = 80         # ticks of silence before a teammate is presumed lost (> interval)
MISSED_MEETUP_WAIT = 5    # ticks to wait at the rendezvous for a missing drone
BASE_POSITION = (21, 41)
BATTERY_DRAIN = 0.002     # per tick -> 500 ticks of flight
BATTERY_RESERVE_TICKS = 15
SIGNAL_FOLLOW_LIMIT = 40
RELAY_HOVER_TICKS = 8
MAX_TICKS = 1200

# Terrain: heights are 0.0 (valley floor) .. 1.0 (highest peak).
MAP_SCALE = GRID_SIZE / 30  # terrain features scale with the map
FLIGHT_CLEARANCE = 0.04   # drones hug the terrain at this height above ground
MAST_HEIGHT = 0.15        # base station mast (baseline mode)
NUM_PEAKS = (8, 12)       # round mountains
NUM_RIDGES = (3, 5)       # long ridgelines: the main radio walls
RELAY_SEARCH_RADIUS = 10  # CLIMB_TO_RELAY looks this far for high ground

# Pre-launch plan: each drone owns one column, searched bottom (near base) to top.
OWN_SECTORS = {0: ["G", "D", "A"], 1: ["H", "E", "B"], 2: ["I", "F", "C"]}


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
def sector_rows(label):
    """Lawnmower rows spaced at beacon width: each row's band covers rows y-R..y+R."""
    _, y0, _, y1 = sector_bounds(label)
    R = BEACON_CLEAR_RADIUS
    rows, r = [], y0 + R
    while True:
        rr = min(r, y1 - 1 - R)
        rows.append(rr)
        if rr + R >= y1 - 1:
            return tuple(rows)
        r += 2 * R + 1


def priority_order(drone_id):
    """Own sectors first, then the others' sectors in reverse, nearest drone first, interleaved."""
    own = OWN_SECTORS[drone_id]
    others = sorted((j for j in OWN_SECTORS if j != drone_id), key=lambda j: (abs(j - drone_id), j))
    rev = [list(reversed(OWN_SECTORS[j])) for j in others]
    order = list(own)
    for k in range(len(own)):
        for r in rev:
            order.append(r[k])
    return order


# ---------------------------------------------------------------- world
class World:
    def __init__(self, seed=42):
        self.seed = seed
        rng = random.Random(seed)
        self.height = self._make_terrain(rng)      # height[y][x] in 0..1
        self.hiker = self._place_hiker(rng)
        self.noise_rng = random.Random(seed * 7919 + 1)
        self._relay_cache = {}

    # ------------------------------------------------------------ terrain
    @staticmethod
    def _make_terrain(rng):
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
            blob(rng.uniform(0, g), rng.uniform(0, g - 6 * s), rng.uniform(0.5, 1.0),
                 rng.uniform(2.5, 5) * s, rng.uniform(2.5, 5) * s, 0)
        for _ in range(rng.randint(*NUM_RIDGES)):         # long ridgelines
            blob(rng.uniform(4 * s, g - 4 * s), rng.uniform(3 * s, g - 8 * s), rng.uniform(0.7, 1.0),
                 rng.uniform(6, 11) * s, rng.uniform(1.3, 2.2) * s, rng.uniform(-0.6, 0.6))
        for _ in range(int(25 * s * s)):                  # rough texture
            blob(rng.uniform(0, g), rng.uniform(0, g), rng.uniform(0.05, 0.15),
                 rng.uniform(1, 2), rng.uniform(1, 2), 0)
        blob(BASE_POSITION[0], BASE_POSITION[1], -0.8, 5 * s, 4 * s, 0)  # base sits in a valley

        lo = min(min(r) for r in h)
        hi = max(max(r) for r in h)
        return tuple(tuple(round((v - lo) / (hi - lo), 3) for v in row) for row in h)

    def _place_hiker(self, rng):
        while True:
            c = (rng.randrange(GRID_SIZE), rng.randrange(GRID_SIZE))
            if self.h(c) < 0.85 and cheb(c, BASE_POSITION) >= GRID_SIZE // 3:
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
        pts = bresenham(a, b)
        n = len(pts) - 1
        if n <= 1:
            return True
        za = self.altitude(a) if za is None else za
        zb = self.altitude(b) if zb is None else zb
        for i in range(1, n):
            if self.h(pts[i]) > za + (zb - za) * i / n:
                return False
        return True

    def mast_altitude(self, c):
        return self.h(c) + MAST_HEIGHT

    def beacon_reading(self, pos):
        s = max(0.0, 1.0 - dist(pos, self.hiker) / BEACON_RANGE)
        if s > 0 and not self.line_of_sight(pos, self.hiker, zb=self.h(self.hiker) + 0.01):
            s *= 0.5  # mountain between drone and hiker
        s += self.noise_rng.gauss(0, BEACON_NOISE)
        return round(s, 3) if s >= SIGNAL_THRESHOLD else 0.0

    def hiker_within(self, pos, r=FIND_RADIUS):
        return cheb(pos, self.hiker) <= r

    def relay_point(self, pos):
        """Best nearby high ground: tall, but not too far to fly to."""
        if pos not in self._relay_cache:
            self._relay_cache[pos] = max(
                cells_in_radius(pos, RELAY_SEARCH_RADIUS),
                key=lambda c: (self.h(c) - 0.03 * cheb(pos, c), -c[0], -c[1]))
        return self._relay_cache[pos]
