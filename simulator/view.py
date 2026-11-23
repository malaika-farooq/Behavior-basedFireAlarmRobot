"""LIVE simulator window: watch the real main.py drive in the arena.

    python simulator/view.py                         (lab map, pad 'bottom', start 'right')
    python simulator/view.py --pad topright --start left --speed 8
    python simulator/view.py --random                (lab map, random pad + start)
    python simulator/view.py --map house:random      (a new random house each time)
    python simulator/view.py --map house:12 --random 5   (repeat a house + layout)
    python simulator/view.py --set "EXPLORE='map'"   (try another strategy)
Maps: lab, lab_old, rooms4, corridor, house:N  (edit them in simulator/arena.py)

Keys: SPACE pause/resume, + / - faster/slower, Esc quit.
Green = robot, red line = bumper pressed, blue line = left sonar beam,
light blue trail = where the color sensor went, black squares = pads, circles = candles.
"""
import argparse, math, sys, time, tkinter as tk
import simcore, arena
from pybricks.world import world, FRONT, REAR, HALFW

PX = 720


class Viewer:
    def __init__(self, speed):
        self.speed = speed
        self.paused = False
        self.root = tk.Tk()
        self.root.title('Fire robot simulator')
        self.s = PX / max(arena.size_mm())
        self.c = tk.Canvas(self.root, width=PX + 20, height=PX + 20, bg='white')
        self.c.pack()
        self.info = tk.Label(self.root, font=('Consolas', 12), anchor='w', justify='left')
        self.info.pack(fill='x')
        self.root.bind('<space>', lambda e: self.toggle())
        self.root.bind('<plus>', lambda e: self.set_speed(2))
        self.root.bind('<equal>', lambda e: self.set_speed(2))
        self.root.bind('<minus>', lambda e: self.set_speed(0.5))
        self.root.bind('<Escape>', lambda e: self.close())
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.closed = False
        self.next_draw = 0
        self.last_trail = None
        self.dyn = []
        self.wall0 = time.time()
        self.sim0 = None

    def p(self, x, y):
        return 10 + x * self.s, 10 + PX - y * self.s

    def static(self):
        for x1, y1, x2, y2 in world.walls:
            self.c.create_line(*self.p(x1, y1), *self.p(x2, y2), width=4, fill='#8b5a2b')
        for (x1, y1, x2, y2), (cx, cy) in zip(world.pads, world.candles):
            self.c.create_rectangle(*self.p(x1, y1), *self.p(x2, y2), fill='black')
            r = 40 * self.s
            X, Y = self.p(cx, cy)
            self.c.create_oval(X - r, Y - r, X + r, Y + r, outline='orange', width=2)
        ft = arena.FT
        W, D = arena.size_mm()
        for i in range(1, int(W / ft) + 1):
            self.c.create_line(*self.p(i * ft, 0), *self.p(i * ft, D), fill='#eeeeee')
        for i in range(1, int(D / ft) + 1):
            self.c.create_line(*self.p(0, i * ft), *self.p(W, i * ft), fill='#eeeeee')
        for x1, y1, x2, y2 in world.walls:            # walls on top of the grid
            self.c.create_line(*self.p(x1, y1), *self.p(x2, y2), width=4, fill='#8b5a2b')

    def toggle(self):
        self.paused = not self.paused

    def set_speed(self, f):
        self.speed = max(0.25, min(64, self.speed * f))
        self.wall0 = time.time(); self.sim0 = world.t

    def close(self):
        self.closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def tick(self):
        if self.closed:
            raise SystemExit('window closed')
        if world.t < self.next_draw:
            return
        self.next_draw = world.t + 40
        if self.sim0 is None:
            self.sim0 = world.t; self.wall0 = time.time()
            self.static()
        self.draw()
        while True:                       # pace: sim time = speed x real time
            self.root.update()
            if self.closed:
                raise SystemExit('window closed')
            if self.paused:
                time.sleep(0.03); self.wall0 = time.time(); self.sim0 = world.t; continue
            ahead = (world.t - self.sim0) / 1000 / self.speed - (time.time() - self.wall0)
            if ahead <= 0:
                break
            time.sleep(min(ahead, 0.03))

    def draw(self):
        for i in self.dyn:
            self.c.delete(i)
        self.dyn = []
        cx, cy = world.to_world(world.cs_x, 0)
        if self.last_trail:
            self.c.create_line(*self.p(*self.last_trail), *self.p(cx, cy), fill='#7ec8ff', width=2)
        self.last_trail = (cx, cy)
        corners = [world.to_world(FRONT, HALFW), world.to_world(FRONT, -HALFW),
                   world.to_world(-REAR, -HALFW), world.to_world(-REAR, HALFW)]
        pts = [v for c in corners for v in self.p(*c)]
        self.dyn.append(self.c.create_polygon(*pts, fill='#9be29b', outline='darkgreen', width=2))
        if world.bumper():
            a, b = world.to_world(FRONT, HALFW), world.to_world(FRONT, -HALFW)
            self.dyn.append(self.c.create_line(*self.p(*a), *self.p(*b), fill='red', width=4))
        d, hit = world.sonar_hit()
        o = world.to_world(-120, 85)
        if hit:
            self.dyn.append(self.c.create_line(*self.p(*o), *self.p(*hit), fill='blue', dash=(3, 3)))
        X, Y = self.p(cx, cy)
        self.dyn.append(self.c.create_oval(X - 3, Y - 3, X + 3, Y + 3, fill='blue'))
        t = (world.t - world.t0) / 1000
        found = '' if world.found_t is None else '   FIRE FOUND at %.1f s' % (world.found_t / 1000)
        if world.candle_hit_t is not None:
            found += '   CANDLE HIT at %.1f s' % (world.candle_hit_t / 1000)
        self.info.config(text='t = %5.1f s  (%s)   x%g speed   behavior: %-10s  sonar %4d mm  driven %.1f m%s' % (
            t, '%d:%02d' % (t // 60, t % 60), self.speed, world.behavior, d, world.dist_mm / 1000, found))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='lab', help=', '.join(arena.NAMES) + "  (house:random = new house)")
    ap.add_argument('--pad', default=None, help='pad name of the map (comma list = several)')
    ap.add_argument('--start', default=None, help='start name of the map')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--speed', type=float, default=5, help='x real time')
    ap.add_argument('--limit', type=int, default=600, help='seconds')
    ap.add_argument('--set', action='append', default=[], help="override a main.py setting, e.g. BASE=45")
    ap.add_argument('--random', type=int, nargs='?', const=-1, default=None,
                    help='random pad + start (optionally give a number to repeat the same layout)')
    a = ap.parse_args()
    import random
    if a.map == 'house:random':
        a.map = 'house:%d' % random.randrange(10000)
    m = arena.use(a.map)
    ov = dict(s.split('=', 1) for s in a.set)
    title = 'map %s' % a.map
    if a.random is None and (a.pad or m['pads']) and (a.start or m['starts']):
        pad = (a.pad or list(m['pads'])[0]).split(',')
        start = a.start or list(m['starts'])[0]
        title += ', pad %s, start %s' % (','.join(pad), start)
    else:                                    # random layout (always for house maps)
        n = a.random if a.random is not None and a.random >= 0 else random.randrange(10000)
        p, start, text = simcore.random_spots(n)
        pad = [p]
        title += ', random layout #%d' % n
        print('%s: %s   (repeat: --map %s --random %d)' % (a.map, text, a.map, n))
    v = Viewer(a.speed)
    v.root.title('Fire robot simulator - ' + title)
    r = simcore.run(pad, start, a.seed, a.limit, overrides=ov, viewer=v)
    print(r)
    if not v.closed:
        v.draw()
        v.info.config(text=v.info.cget('text') + ('' if r['found_s'] else '   NOT FOUND (time limit)'))
        v.root.mainloop()


if __name__ == '__main__':
    main()
