"""ASCII picture of one run: where the robot spent its time (1 char = 0.5 ft).
    python heat.py MAP LAYOUT [LIMIT] [--set K=V ...] [--src FILE]
    '#' wall, 'P' pad, 'S' start, ' ' never visited, '.' <3 s, 'o' <10 s, 'O' <30 s, '@' more
"""
import sys
import simcore, arena
from pybricks.world import world
import pybricks.world as W

args = [a for a in sys.argv[1:] if not a.startswith('--')]
ov = dict(a[6:].split('=', 1) for a in sys.argv[1:] if a.startswith('--set='))
srcf = [a[6:] for a in sys.argv[1:] if a.startswith('--src=')]
mp, n = args[0], int(args[1])
limit = int(args[2]) if len(args) > 2 else 300
arena.use(mp)
pad, st, text = simcore.random_spots(n)
H = {}
orig = W.W.step


def step(self, ms):
    orig(self, ms)
    k = (int(self.x // (arena.FT / 2)), int(self.y // (arena.FT / 2)))
    H[k] = H.get(k, 0) + ms


W.W.step = step
r = simcore.run([pad], st, n, limit, overrides=ov, src=open(srcf[0]).read() if srcf else None,
                logfile='log_heat.csv')
print(mp, text, r)
wall = set()
for x1, y1, x2, y2 in arena.walls_mm():
    L = max(abs(x2 - x1), abs(y2 - y1)) / arena.FT
    k = int(L * 4) + 1
    for i in range(k + 1):
        wall.add((int((x1 + (x2 - x1) * i / k) // (arena.FT / 2)), int((y1 + (y2 - y1) * i / k) // (arena.FT / 2))))
Wd, Dd = arena.size_mm()
pc = ((pad[0] + pad[2]) / 2, (pad[1] + pad[3]) / 2)
for j in range(int(Dd // (arena.FT / 2)), -1, -1):
    row = ''
    for i in range(int(Wd // (arena.FT / 2)) + 1):
        cx, cy = (i + .5) * arena.FT / 2, (j + .5) * arena.FT / 2
        if (i, j) in wall: row += '#'
        elif abs(cx - pc[0]) < 170 and abs(cy - pc[1]) < 170: row += 'P'
        elif abs(cx - st[0]) < 80 and abs(cy - st[1]) < 80: row += 'S'
        else:
            s = H.get((i, j), 0) / 1000
            row += ' ' if s == 0 else '.' if s < 3 else 'o' if s < 10 else 'O' if s < 30 else '@'
    print(row)
