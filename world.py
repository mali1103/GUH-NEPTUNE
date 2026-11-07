"""World model: grid, ridges, hidden hiker, beacon signal, line of sight, sector helpers.

Everything the drones are allowed to know up front (terrain, sectors, base,
rendezvous plan) lives here as constants/helpers. The hiker position is only
exposed through `beacon_reading` and `hiker_within`.
"""
import math
import random
from functools import lru_cache

# ---------------------------------------------------------------- constants
GRID_SIZE = 30
SECTOR_SIZE = 10
SECTOR_LABELS = "ABCDEFGHI"
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
RENDEZVOUS_SKIP_IF_HEARD = 15  # skip the trip if every live teammate was heard this recently
LOST_TIMEOUT = 60        # ticks of silence before a teammate is presumed lost
MISSED_MEETUP_WAIT = 5    # ticks to wait at the rendezvous for a missing drone
BASE_POSITION = (15, 29)
BATTERY_DRAIN = 0.0025    # per tick -> 400 ticks of flight
BATTERY_RESERVE_TICKS = 12
SIGNAL_FOLLOW_LIMIT = 40
RELAY_HOVER_TICKS = 8
MAX_TICKS = 900
NUM_RIDGES = 7
RIDGE_LENGTH = (8, 16)

# Pre-launch plan: each drone owns one column, searched bottom (near base) to top.
OWN_SECTORS = {0: ["G", "D", "A"], 1: ["H", "E", "B"], 2: ["I", "F", "C"]}

DIRS8 = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]


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
    """Lawnmower rows: each row's sensor band covers rows y-R..y+R."""
    _, y0, _, y1 = sector_bounds(label)
    R = SENSOR_RADIUS
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
        self.ridges = self._make_ridges(rng)
        self.hiker = self._place_hiker(rng)
        self.noise_rng = random.Random(seed * 7919 + 1)

    def _make_ridges(self, rng):
        ridges = set()
        protected = set(cells_in_radius(BASE_POSITION, 1))
        for _ in range(NUM_RIDGES):
            x, y = rng.randrange(GRID_SIZE), rng.randrange(3, GRID_SIZE - 4)
            d = rng.choice([0, 1, 7, 0, 2])  # mostly east-west ridges
            for _ in range(rng.randint(*RIDGE_LENGTH)):
                if rng.random() < 0.3:
                    d = (d + rng.choice([-1, 1])) % 8
                dx, dy = DIRS8[d]
                for c in ((x, y), (x + dx, y)):  # 4-connected walls: no diagonal gaps
                    if in_bounds(c) and c not in protected:
                        ridges.add(c)
                x, y = x + dx, y + dy
                if not in_bounds((x, y)):
                    break
        return ridges

    def _place_hiker(self, rng):
        while True:
            c = (rng.randrange(GRID_SIZE), rng.randrange(GRID_SIZE))
            if c not in self.ridges and cheb(c, BASE_POSITION) >= 10:
                return c

    def is_ridge(self, c):
        return c in self.ridges

    def line_of_sight(self, a, b):
        if a in self.ridges or b in self.ridges:  # on high ground
            return True
        return not any(c in self.ridges for c in bresenham(a, b)[1:-1])

    def beacon_reading(self, pos):
        s = max(0.0, 1.0 - dist(pos, self.hiker) / BEACON_RANGE)
        if s > 0 and not self.line_of_sight(pos, self.hiker):
            s *= 0.5
        s += self.noise_rng.gauss(0, BEACON_NOISE)
        return round(s, 3) if s >= SIGNAL_THRESHOLD else 0.0

    def hiker_within(self, pos, r=FIND_RADIUS):
        return cheb(pos, self.hiker) <= r

    def nearest_ridge(self, pos):
        if not self.ridges:
            return None
        return min(self.ridges, key=lambda c: (cheb(pos, c), c))
