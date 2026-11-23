from pybricks.world import world


def wait(ms):
    ms = int(ms)
    while ms > 0:
        d = 5 if ms >= 5 else ms
        world.step(d)
        ms -= d
        if world.viewer is not None:
            world.viewer.tick()
    if world.t - world.t0 > world.limit:
        raise SystemExit('time limit')
    if world.found_t is not None and world.t - world.t0 - world.found_t > world.after_fire_ms:
        raise SystemExit('found')


class StopWatch:
    def time(self):
        return int(world.t)
