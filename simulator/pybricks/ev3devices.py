from pybricks.world import world


class Motor:
    def __init__(self, port, direction=1):
        self.port = port; self.duty = 0; self.angle_f = 0.0
        world.motors[port] = self
    def dc(self, d): self.duty = d
    def stop(self, *a): self.duty = 0
    def angle(self): return int(self.angle_f)
    def reset_angle(self, a=0): self.angle_f = float(a)
    def speed(self): return int(self.duty * 6.6)


class TouchSensor:
    def __init__(self, p): pass
    def pressed(self): return world.bumper()


class UltrasonicSensor:
    def __init__(self, p): pass
    def distance(self): return world.sonar()


class ColorSensor:
    def __init__(self, p): pass
    def reflection(self): return world.color()


class GyroSensor:
    def __init__(self, p): pass
    def speed(self): return world.gyro_rate()
    def angle(self): return 0
