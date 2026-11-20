#!/usr/bin/env python3
"""Fan-only test: medium motor on port D. Wheels are not touched."""
import time
from ev3dev2.motor import MediumMotor, OUTPUT_D, SpeedPercent

fan = MediumMotor(OUTPUT_D)

print("Fan forward 100% for 4s")
fan.on(SpeedPercent(100))
time.sleep(4)
fan.off()
time.sleep(1)

print("Fan reverse 100% for 4s (use whichever direction blows forward)")
fan.on(SpeedPercent(-100))
time.sleep(4)
fan.off()
print("done")
