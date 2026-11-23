"""Arena maps for the simulator. Choose one with  --map NAME  (view.py / batch.py).

    lab        your lab arena, read from the overhead photo (default)
    lab_old    the earlier reading of the same arena (side photos)
    rooms4     4 rooms, simple doors
    corridor   a corridor with rooms on both sides
    house:N    a random house (rooms, doors, a free-standing box), N = any number

All sizes in FEET. Origin = near-left corner, x to the right, y away from you.
"""
import math, random

FT = 304.8
TILE_FT = 1.25               # one floor tile = 15 in (measured); arena = 7 x 7 tiles

PAD_FT = TILE_FT / 2       # black pad: "the size of half a floor tile" (project sheet)


def _lab():
    # the lab arena is 7 x 7 floor tiles; every wall from the overhead photo lies on a tile
    # line (tile units below, origin = near-left corner, x right, y away from you)
    T = TILE_FT
    t = lambda *v: tuple(round(a * T, 3) for a in v)
    W = D = 7 * T
    walls = [
        t(3, 0, 3, 1), t(3, 1, 4, 1),                      # bottom room (1 x 1 tile, open right)
        t(4, 1, 4, 3),                                     # long wall
        t(4, 3, 5, 3),                                     # T arm
        t(3, 3, 4, 3), t(3, 4, 4, 4), t(3, 3, 3, 4), t(4, 3, 4, 4),   # closed square (1 x 1)
        t(0, 5, 2, 5),                                     # balcony front wall
        t(2, 6, 2, 7),                                     # stub from the far wall
        t(4, 5, 5, 5), t(4, 6, 5, 6), t(4, 5, 4, 6), t(5, 5, 5, 6),   # free-standing square
        t(5, 4, 7, 4),                                     # top-right room wall
    ]
    pads = {'bottom': t(3.5, 0.5), 'balcony': t(1.0, 6.0), 'topright': t(6.0, 5.5),
            'leftopen': t(0.8, 2.8), 'middle': t(2.0, 3.8), 'right': t(6.0, 2.0)}
    starts = {'right': t(5.6, 1.0) + (90,), 'left': t(1.0, 1.0) + (0,),
              'center': t(2.2, 4.2) + (270,)}
    nogo = [t(3, 3, 4, 4), t(4, 5, 5, 6)]
    return dict(w=W, d=D, walls=walls, pads=pads, starts=starts, nogo=nogo)


def _lab_old():
    W, D = 9.0, 8.6
    walls = [(3.6, 0, 3.6, 2.1), (3.6, 2.1, 5.4, 2.1), (5.4, 2.1, 5.4, 4.2), (4.0, 4.2, 5.9, 4.2),
             (4.0, 4.2, 4.0, 5.2), (4.0, 5.2, 5.0, 5.2), (5.0, 4.7, 5.0, 5.2), (0, 7.0, 3.5, 7.0),
             (5.6, 6.4, 6.8, 6.4), (5.6, 6.4, 5.6, D), (6.8, 6.4, 6.8, D), (6.7, 5.0, W, 5.0)]
    pads = {'bottom': (4.5, 0.75), 'topright': (7.6, 6.4), 'balcony': (2.0, 7.8),
            'leftopen': (1.0, 3.5), 'middle': (2.6, 5.0)}
    starts = {'right': (7.5, 2.5, 90), 'left': (1.5, 1.5, 0), 'center': (2.5, 5.8, 270)}
    nogo = [(5.6, 6.4, 6.8, D), (4.0, 4.2, 5.0, 5.2)]
    return dict(w=W, d=D, walls=walls, pads=pads, starts=starts, nogo=nogo)


def _hline(y, x0, x1, gaps=()):
    s, x = [], x0
    for g0, g1 in sorted(gaps):
        if g0 > x: s.append((x, y, g0, y))
        x = g1
    if x < x1: s.append((x, y, x1, y))
    return s


def _vline(x, y0, y1, gaps=()):
    s, y = [], y0
    for g0, g1 in sorted(gaps):
        if g0 > y: s.append((x, y, x, g0))
        y = g1
    if y < y1: s.append((x, y, x, y1))
    return s


def _rooms4():
    W = D = 8.0
    walls = (_vline(4.0, 0, D, [(1.3, 2.8), (5.6, 6.9)]) + _hline(4.0, 0, 4.0, [(1.0, 2.3)]) +
             _hline(4.0, 4.0, W, [(5.9, 7.1)]))
    return dict(w=W, d=D, walls=walls, pads={'a': (2, 6), 'b': (6, 6), 'c': (6, 2)},
                starts={'s': (1, 1, 0)}, nogo=[])


def _corridor():
    W = D = 8.0
    walls = (_hline(3.3, 0, W, [(1.6, 2.9), (5.6, 6.7)]) + _hline(4.7, 0, W, [(1.0, 2.3), (5.9, 7.2)]) +
             _vline(4.0, 0, 3.3) + _vline(4.9, 4.7, D))
    return dict(w=W, d=D, walls=walls, pads={'a': (2, 1.5), 'b': (6, 1.5), 'c': (2, 6.5), 'd': (6.5, 6.5)},
                starts={'s': (0.7, 4.0, 0)}, nogo=[])


def _house(seed):
    """Random house: recursive room division with doors, sometimes a free-standing box."""
    rng = random.Random(seed)
    W = round(rng.uniform(8, 10), 1)
    D = round(rng.uniform(8, 10), 1)
    walls, nogo = [], []

    def split(x0, y0, x1, y1, depth):
        w, h = x1 - x0, y1 - y0
        if depth >= 3 or (w < 4.5 and h < 4.5) or (depth > 0 and rng.random() < 0.25):
            return
        vertical = w >= h if abs(w - h) > 1 else rng.random() < 0.5
        if vertical and w < 4.5: vertical = False
        if not vertical and h < 4.5: vertical = True
        if vertical:
            x = round(rng.uniform(x0 + 2, x1 - 2), 1)
            g = rng.uniform(1.3, 2.0); p = rng.uniform(y0, y1 - g)
            gaps = [(p, p + g)]
            if h > 6 and rng.random() < 0.4:
                q = y1 - g if p < (y0 + y1) / 2 else y0
                gaps.append((q, q + g))
            walls.extend(_vline(x, y0, y1, gaps))
            split(x0, y0, x, y1, depth + 1); split(x, y0, x1, y1, depth + 1)
        else:
            y = round(rng.uniform(y0 + 2, y1 - 2), 1)
            g = rng.uniform(1.3, 2.0); p = rng.uniform(x0, x1 - g)
            gaps = [(p, p + g)]
            if w > 6 and rng.random() < 0.4:
                q = x1 - g if p < (x0 + x1) / 2 else x0
                gaps.append((q, q + g))
            walls.extend(_hline(y, x0, x1, gaps))
            split(x0, y0, x1, y, depth + 1); split(x0, y, x1, y1, depth + 1)

    split(0, 0, W, D, 0)
    if rng.random() < 0.6:                       # a free-standing box
        s = rng.uniform(1.0, 1.4)
        bx, by = rng.uniform(1.5, W - 1.5 - s), rng.uniform(1.5, D - 1.5 - s)
        walls += [(bx, by, bx + s, by), (bx, by + s, bx + s, by + s), (bx, by, bx, by + s), (bx + s, by, bx + s, by + s)]
        nogo.append((bx, by, bx + s, by + s))
    return dict(w=W, d=D, walls=walls, pads={}, starts={}, nogo=nogo)


MAPS = {'lab': _lab, 'lab_old': _lab_old, 'rooms4': _rooms4, 'corridor': _corridor}
NAMES = list(MAPS) + ['house:N']


class Cur:
    name = 'lab'
    m = _lab()


def use(name):
    Cur.name = name
    Cur.m = _house(int(name.split(':')[1])) if name.startswith('house:') else MAPS[name]()
    return Cur.m


def mm(v):
    return v * FT


def size_mm():
    return mm(Cur.m['w']), mm(Cur.m['d'])


def walls_mm():
    W, D = Cur.m['w'], Cur.m['d']
    outer = [(0, 0, W, 0), (0, D, W, D), (0, 0, 0, D), (W, 0, W, D)]
    return [tuple(mm(v) for v in w) for w in outer + Cur.m['walls']]


def pad_rect_mm(cx, cy):
    h = PAD_FT / 2
    return (mm(cx - h), mm(cy - h), mm(cx + h), mm(cy + h))


def pad_mm(name):
    return pad_rect_mm(*Cur.m['pads'][name])


def start_mm(name):
    x, y, h = Cur.m['starts'][name]
    return (mm(x), mm(y), math.radians(h))


# ---------------- reachability (where can the robot's centre go?) ----------------
def free_grid(clear=115, step=50):
    """Set of grid cells (step mm) where the robot centre fits (clear mm from walls)."""
    W, D = size_mm()
    ws = walls_mm()
    nogo = [tuple(mm(v) for v in r) for r in Cur.m['nogo']]

    def dist(px, py, w):
        x1, y1, x2, y2 = w
        cx = min(max(px, min(x1, x2)), max(x1, x2)); cy = min(max(py, min(y1, y2)), max(y1, y2))
        return math.hypot(px - cx, py - cy)
    free = set()
    for i in range(int(W // step) + 1):
        for j in range(int(D // step) + 1):
            x, y = i * step, j * step
            if any(a <= x <= c and b <= y <= d for a, b, c, d in nogo):
                continue
            if all(dist(x, y, w) > clear for w in ws):
                free.add((i, j))
    return free, step


def reachable(start_xy, free, step):
    from collections import deque
    s = (int(round(start_xy[0] / step)), int(round(start_xy[1] / step)))
    if s not in free:
        return set()
    seen = {s}; q = deque([s])
    while q:
        i, j = q.popleft()
        for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if n in free and n not in seen:
                seen.add(n); q.append(n)
    return seen
