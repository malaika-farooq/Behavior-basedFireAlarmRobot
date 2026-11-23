#!/usr/bin/env pybricks-micropython
"""FAN CHECK (motor on port D): full power for 5 s, prints how fast it really turned.
Hold the robot so nothing is near the fan."""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

print('battery: %.2f V (full ~8.0+, weak below ~7.4)' % (EV3Brick().battery.voltage() / 1000))
fan = Motor(Port.D)
fan.reset_angle(0)
fan.dc(100)
for i in range(5):
    wait(1000)
    print('second %d: %d deg/s' % (i + 1, fan.speed()))
fan.stop()
print('turned %d deg in 5 s' % abs(fan.angle()))
print('Near 0 = jammed gears or bad cable. A medium motor at full power does ~1000+ deg/s free.')
