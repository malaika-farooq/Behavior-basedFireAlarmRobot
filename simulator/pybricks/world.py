"""2-D world for running the REAL main.py on a PC (fake pybricks).
Differential-drive robot in a map of axis-aligned walls. Units: mm, ms, degrees.
Robot frame: x forward, y to the left. World: x right, y up (away from you)."""
import math, random

# ---- robot (measured) ----
WHEEL_DIA = 54.0
TRACK = 150.0                  # wheel to wheel
FRONT, REAR, HALFW = 75.0, 160.0, 85.0   # axle to bumper / to back, half width
SONAR_LOCAL = (-102.0, 85.0)   # sonar: 4 in (10.2 cm) behind the axle, left side, looking left
COLOR_X = 42.0                 # color sensor 1.65 in (4.2 cm) ahead of the axle
# ---- motors / sheet ----
K_DEG_PER_MS = 0.0066          # wheel deg per ms per % power (35 % -> ~230 deg/s, from logs)
DEADBAND = 16                  # below this power the wheels do not move the robot
HALF_T = 8.0                   # wall half thickness
CANDLE_R = 40.0                # glass jar radius


def _perimeter():
    pts = []
    for i in range(5):
        y = -HALFW + i * 2 * HALFW / 4
        pts.append((FRONT, y)); pts.append((-REAR, y))
    for i in range(1, 5):
        x = -REAR + i * (FRONT + REAR) / 5
        pts.append((x, HALFW)); pts.append((x, -HALFW))
    return pts
PERIM = _perimeter()
BUMPER = [(FRONT, -HALFW + i * 2 * HALFW / 6) for i in range(7)]


class W:
    viewer = None
    after_fire_ms = 8000       # keep simulating this long after FIRE (back-up, first fan sweeps)

    def reset(self, walls, pads, start, seed, limit, cs_x=COLOR_X, effL=1.02, effR=1.0):
        self.walls = walls                    # (x1,y1,x2,y2) axis aligned, mm
        self.pads = pads                      # black pads (x1,y1,x2,y2)
        self.candles = [((p[0] + p[2]) / 2, (p[1] + p[3]) / 2) for p in pads]
        self.x, self.y, self.th = start       # th: rad, counter-clockwise, 0 = +x
        self.t = seed * 37 % 1000
        self.t0 = self.t
        self.limit = limit
        self.rng = random.Random(seed)
        self.motors = {}
        self.omega = 0.0
        self.found_t = None
        self.found_xy = None
        self.cs_x = cs_x
        self.eff = {'A': effL, 'B': effR}
        self.visited = set()
        self.stuck_ms = 0
        self.bias = self.rng.uniform(-1.5, 1.5)
        self.dist_mm = 0.0
        self.behavior = ''
        self.trail = []
        self.candle_hit_t = None              # first time the body touched the candle jar

    def touching_candle(self, x, y, th, gap=3.0):
        """True if the robot body (rectangle) is within `gap` mm of a candle jar."""
        c, s = math.cos(th), math.sin(th)
        for cx, cy in self.candles:
            lx = (cx - x) * c + (cy - y) * s
            ly = -(cx - x) * s + (cy - y) * c
            dx = max(-REAR - lx, 0, lx - FRONT)
            dy = max(abs(ly) - HALFW, 0)
            if math.hypot(dx, dy) < CANDLE_R + gap:
                return True
        return False

    # ---------- geometry ----------
    def to_world(self, px, py, x=None, y=None, th=None):
        x = self.x if x is None else x; y = self.y if y is None else y
        th = self.th if th is None else th
        c, s = math.cos(th), math.sin(th)
        return x + px * c - py * s, y + px * s + py * c

    def near_walls(self, x, y, r=320.0):
        return [w for w in self.walls if min(w[0], w[2]) - r < x < max(w[0], w[2]) + r
                and min(w[1], w[3]) - r < y < max(w[1], w[3]) + r]

    @staticmethod
    def seg_dist(px, py, w):
        x1, y1, x2, y2 = w
        cx = min(max(px, min(x1, x2)), max(x1, x2))
        cy = min(max(py, min(y1, y2)), max(y1, y2))
        return math.hypot(px - cx, py - cy)

    def collides(self, x, y, th, pts=PERIM, margin=0.0):
        ws = self.near_walls(x, y)
        c, s = math.cos(th), math.sin(th)
        if pts is PERIM:      # wall ends / candles inside the body
            ends = [(w[0], w[1], HALF_T) for w in ws] + [(w[2], w[3], HALF_T) for w in ws]
            ends += [(cx, cy, CANDLE_R) for cx, cy in self.candles]
            for ex, ey, r in ends:
                lx = (ex - x) * c + (ey - y) * s
                ly = -(ex - x) * s + (ey - y) * c
                if -REAR - r < lx < FRONT + r and abs(ly) < HALFW + r:
                    return True
        for px, py in pts:
            wx, wy = x + px * c - py * s, y + px * s + py * c
            for w in ws:
                if self.seg_dist(wx, wy, w) < HALF_T + margin:
                    return True
            for cx, cy in self.candles:
                if math.hypot(wx - cx, wy - cy) < CANDLE_R + margin:
                    return True
        return False

    # ---------- physics ----------
    def step(self, ms):
        mA = self.motors.get('A'); mB = self.motors.get('B')
        if mA is None or mB is None:
            self.t += ms; return
        def wheel(m):
            d = m.duty
            if abs(d) < DEADBAND: return 0.0
            return d * K_DEG_PER_MS * self.eff[m.port] * ms * self.rng.uniform(0.97, 1.03)
        aL, aR = wheel(mA), wheel(mB)
        k = math.pi * WHEEL_DIA / 360.0
        def pose(aL, aR):
            dl, dr = aL * k, aR * k
            v = (dl + dr) / 2; dth = (dr - dl) / TRACK
            th = self.th + dth + self.rng.gauss(0, 0.0004) * (1 if (aL or aR) else 0)
            return self.x + v * math.cos(self.th + dth / 2), self.y + v * math.sin(self.th + dth / 2), th
        moved = None
        for cand in ((aL, aR), (0.0, aR), (aL, 0.0)):
            if cand != (aL, aR) and cand[0] == 0 and cand[1] == 0:
                continue
            x, y, th = pose(*cand)
            if not self.collides(x, y, th):
                moved = (x, y, th, cand); break
        if moved is None and (abs(aL) > 0.01 or abs(aR) > 0.01):
            # blocked: on the slick sheet the body gets shoved a little, the wheels slip
            x, y, th = pose(aL, aR)
            for r in (2.0, 4.0):
                for i in range(8):
                    a = i * math.pi / 4
                    nx, ny = self.x + r * math.cos(a), self.y + r * math.sin(a)
                    if not self.collides(nx, ny, th):
                        moved = (nx, ny, th, (aL * 0.6, aR * 0.6)); break
                if moved: break
        if moved is None:
            self.omega = 0.0
            if abs(aL) > 0.01 or abs(aR) > 0.01:
                self.stuck_ms += ms
        else:
            x, y, th, (uL, uR) = moved
            self.omega = math.degrees(th - self.th) * 1000.0 / ms
            self.dist_mm += math.hypot(x - self.x, y - self.y)
            self.x, self.y, self.th = x, y, th
            mA.angle_f += uL; mB.angle_f += uR
        cx, cy = self.to_world(self.cs_x, 0)
        self.visited.add((int(cx // 100), int(cy // 100)))
        if self.candle_hit_t is None and self.touching_candle(self.x, self.y, self.th):
            self.candle_hit_t = self.t - self.t0      # a real jar would be pushed / tipped
        self.t += ms

    # ---------- sensors ----------
    def bumper(self):
        c, s = math.cos(self.th), math.sin(self.th)
        for w in self.near_walls(self.x, self.y):
            for ex, ey in ((w[0], w[1]), (w[2], w[3])):
                lx = (ex - self.x) * c + (ey - self.y) * s
                ly = -(ex - self.x) * s + (ey - self.y) * c
                if FRONT - 2 < lx < FRONT + HALF_T + 6 and abs(ly) < HALFW:
                    return True
        return self.collides(self.x + 5 * c, self.y + 5 * s, self.th, BUMPER, 0.5)

    def color(self):
        cx, cy = self.to_world(self.cs_x, 0)
        for p in self.pads:
            if p[0] <= cx <= p[2] and p[1] <= cy <= p[3]:
                return self.rng.choice([0, 1, 1, 2])
        return self.rng.randint(15, 22)

    def sonar_hit(self):
        """(distance mm, hit point) of the left sonar. Glancing echoes (> 45 deg) are lost."""
        ox, oy = self.to_world(*SONAR_LOCAL)
        best, pt = 2550, None
        for da in (-0.2, 0.0, 0.2):
            a = self.th + math.pi / 2 + da
            dx, dy = math.cos(a), math.sin(a)
            for w in self.walls:
                x1, y1, x2, y2 = w
                if x1 == x2:
                    if abs(dx) < 1e-9: continue
                    tt = (x1 - ox) / dx
                    if tt <= 0: continue
                    yy = oy + tt * dy
                    if not (min(y1, y2) - HALF_T <= yy <= max(y1, y2) + HALF_T): continue
                    inc = math.degrees(math.acos(min(1, abs(dx))))
                else:
                    if abs(dy) < 1e-9: continue
                    tt = (y1 - oy) / dy
                    if tt <= 0: continue
                    xx = ox + tt * dx
                    if not (min(x1, x2) - HALF_T <= xx <= max(x1, x2) + HALF_T): continue
                    inc = math.degrees(math.acos(min(1, abs(dy))))
                if inc > 45: continue
                d = tt - HALF_T
                if d < best:
                    best, pt = d, (ox + tt * dx, oy + tt * dy)
        return max(30, best), pt

    def sonar(self):
        return int(self.sonar_hit()[0] + self.rng.randint(-3, 3))

    def gyro_rate(self):
        return int(round(-self.omega + self.bias))   # right turn = positive


world = W()
