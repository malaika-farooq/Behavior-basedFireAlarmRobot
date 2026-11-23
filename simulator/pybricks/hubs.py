from pybricks.world import world


class _S:
    def beep(self, f=500, d=100):
        from pybricks.tools import wait
        wait(d)


class _L:
    def on(self, c): pass
    def off(self): pass


class _Sc:
    def clear(self): pass
    def print(self, *a):
        if a and a[0] == 'FIRE!' and world.found_t is None:
            world.found_t = world.t - world.t0    # keep running a few s: back-up + fan sweep
            world.found_xy = (world.x, world.y)


class EV3Brick:
    speaker = _S(); light = _L(); screen = _Sc()
