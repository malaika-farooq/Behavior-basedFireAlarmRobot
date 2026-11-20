#!/usr/bin/env python3
"""Run this FIRST to verify ports, motor directions, and sensor readings.
Same port constants as fire_robot.py - keep them identical."""
import time
from ev3dev2.motor import (MoveTank, MediumMotor, OUTPUT_A, OUTPUT_B,
                           OUTPUT_D, SpeedPercent)
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor, ColorSensor

drive = MoveTank(OUTPUT_B, OUTPUT_A)
fan = MediumMotor(OUTPUT_D)
touch = TouchSensor(INPUT_4)
sonar = UltrasonicSensor(INPUT_3)
color = ColorSensor(INPUT_1)
color.mode = ColorSensor.MODE_RGB_RAW

print("Drive forward 1s (robot must go FORWARD, fan side = front)")
drive.on_for_seconds(SpeedPercent(30), SpeedPercent(30), 1)
print("Fan 3s")
fan.on_for_seconds(SpeedPercent(100), 3)
print("Sensors for 15s: press touch, wave hand on left, move over colors")
t0 = time.time()
while time.time() - t0 < 15:
    print("touch=%s sonar=%.1fcm rgb=%s" % (
        touch.is_pressed, sonar.distance_centimeters, color.rgb))
    time.sleep(0.3)
