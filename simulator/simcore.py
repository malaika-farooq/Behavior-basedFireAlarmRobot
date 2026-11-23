"""Runs the real main.py once in a simulated arena."""
import sys, os, io, re, math, random, contextlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pybricks.world import world
import arena

MAIN = os.path.join(HERE, '..', 'main.py')


class _Out(io.TextIOBase):
    """Captures main.py's print() lines: the behavior names show in the viewer."""
    def write(self, s):
        s = s.strip()
        if s and s.isupper() and ' ' not in s:
            world.behavior = s
        return len(s)


_cache = {}


def _grids():
    k = arena.Cur.name
    if k not in _cache:
        _cache[k] = (arena.free_grid(115), arena.free_grid(200))
    return _cache[k]


def random_spots(n):
    """Layout number n -> (pad rect mm, start pose, text). Pad reachable from the start,
    not touching walls, at least 60 cm away."""
    rng = random.Random(n * 7919 + sum(ord(c) * (i + 1) for i, c in enumerate(arena.Cur.name)))
    (free, step), (roomy, _) = _grids()
    W, D = arena.size_mm()
    half = arena.mm(arena.PAD_FT) / 2
    cells = sorted(roomy)
    world.reset(arena.walls_mm(), [], (0, 0, 0), 1, 1000)
    for _ in range(500):
        i, j = rng.choice(cells)
        x, y = i * step, j * step
        th = math.radians(rng.choice([0, 90, 180, 270]))
        if world.collides(x, y, th):
            continue
        reach = arena.reachable((x, y), free, step)
        for _ in range(300):
            cx, cy = rng.uniform(half, W - half), rng.uniform(half, D - half)
            if math.hypot(cx - x, cy - y) < 600:
                continue
            # pad must not overlap a wall / box
            pr = (cx - half, cy - half, cx + half, cy + half)
            if any(min(w[0], w[2]) - 8 < pr[2] and max(w[0], w[2]) + 8 > pr[0] and
                   min(w[1], w[3]) - 8 < pr[3] and max(w[1], w[3]) + 8 > pr[1] for w in world.walls):
                continue
            # robot centre must be able to get within 6 cm of the pad centre
            if any((round((cx + dx) / step), round((cy + dy) / step)) in reach
                   for dx in (-60, 0, 60) for dy in (-60, 0, 60)):
                text = 'pad at (%.1f, %.1f) ft, start at (%.1f, %.1f) ft facing %d deg' % (
                    cx / arena.FT, cy / arena.FT, x / arena.FT, y / arena.FT, round(math.degrees(th)))
                return pr, (x, y, th), text
    raise RuntimeError('no reachable layout found')


def run(pad, start, seed=1, limit_s=600, src=None, overrides=None, viewer=None,
        logfile='log.csv', map_name=None):
    """pad: name / list of names / list of (x1,y1,x2,y2) mm.  start: name or (x, y, th)."""
    if map_name and map_name != arena.Cur.name:
        arena.use(map_name)
    pads = pad if isinstance(pad, list) else [pad]
    pads = [arena.pad_mm(p) if isinstance(p, str) else p for p in pads]
    st = arena.start_mm(start) if isinstance(start, str) else start
    world.reset(arena.walls_mm(), pads, st, seed, limit_s * 1000)
    world.viewer = viewer
    code = src if src is not None else open(MAIN).read()
    overrides = dict(overrides or {})
    overrides.setdefault('LOG_FILE', repr(logfile if logfile else os.devnull))
    for k, v in overrides.items():
        code, n = re.subn(r'^%s = .*$' % k, '%s = %s' % (k, v), code, count=1, flags=re.M)
        if n != 1:
            raise KeyError('setting not found in main.py: ' + k)
    cwd = os.getcwd()
    os.chdir(HERE)                      # main.py writes log.csv here (simulator/log.csv)
    try:
        with contextlib.redirect_stdout(_Out()):
            exec(compile(code, MAIN, 'exec'), {'__name__': '__main__'})
    except SystemExit:
        pass
    finally:
        os.chdir(cwd)
    ft, ct = world.found_t, world.candle_hit_t
    return dict(found_s=None if ft is None else ft / 1000,
                dist_m=world.dist_mm / 1000, cells=len(world.visited),
                # candle touched: 'before' = while finding the pad, 'after' = back-up / fan sweep
                candle=None if ct is None else ('before' if ft is None or ct <= ft else 'after'))
