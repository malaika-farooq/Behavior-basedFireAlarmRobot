#!/usr/bin/env python3
"""Step 1: check ports + read sensors. Run this BEFORE fire_robot.py.
Move the robot by hand: put the color sensor over floor, then over the fire pad,
and note the RGB numbers. Press the bumper, put a wall near the left sensor.
Ctrl+C to stop. Ports must match fire_robot.py."""
import time
from ev3dev2.motor import MoveTank, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor, UltrasonicSensor

COLOR_PORT = INPUT_1   # change if the color sensor is on another port

touch = TouchSensor(INPUT_4)
us = UltrasonicSensor(INPUT_3)
try:
    color = ColorSensor(COLOR_PORT)
    color.mode = 'RGB-RAW'
except Exception as e:
    color = None
    print('WARNING: color sensor not found:', e)
tank = MoveTank(OUTPUT_A, OUTPUT_B)
fan = MediumMotor(OUTPUT_D)

# motor check: both wheels should go FORWARD, fan should spin
print('Motors: wheels forward 1s, fan 2s')
tank.on_for_seconds(SpeedPercent(20), SpeedPercent(20), 1)
fan.on_for_seconds(SpeedPercent(100), 2)

print('Sensors (Ctrl+C to quit)')
while True:
    rgb = color.raw if color else 'n/a'
    print('rgb=%s  left_wall=%.1fcm  bumper=%s' %
          (rgb, us.distance_centimeters, touch.is_pressed))
    time.sleep(0.3)
