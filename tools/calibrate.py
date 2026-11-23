#!/usr/bin/env pybricks-micropython
"""SONAR CHECK only (left ultrasonic, port 2). Takes about 1 minute.
Hold a flat box / book square to the sonar at each distance the screen asks for
(measured from the FRONT of the sonar), then press the TOUCH sensor (port 1).
It averages 20 readings at each distance and prints the error."""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import TouchSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait

ev3 = EV3Brick()
touch = TouchSensor(Port.S1)
sonar = UltrasonicSensor(Port.S2)

DISTANCES_CM = (10, 15, 20, 30)     # 10 cm = the wall-following distance in main.py


def wait_press(cm):
    """Show the live reading until the touch sensor is pressed."""
    while touch.pressed():
        wait(20)
    while not touch.pressed():
        ev3.screen.clear()
        ev3.screen.print('Box at %d cm' % cm)
        ev3.screen.print('now %d mm' % sonar.distance())
        ev3.screen.print('press touch')
        wait(150)
    ev3.speaker.beep(800, 80)
    wait(500)                       # let the hand leave the robot


print('=== SONAR CHECK ===')
results = []
for cm in DISTANCES_CM:
    wait_press(cm)
    vals = []
    for _ in range(20):
        vals.append(sonar.distance())
        wait(30)
    avg = sum(vals) // len(vals)
    results.append((cm, avg, min(vals), max(vals)))
    print('%2d cm: avg %4d mm (min %d, max %d), error %+d mm' % (
        cm, avg, min(vals), max(vals), avg - cm * 10))

ev3.screen.clear()
for cm, avg, lo, hi in results:
    ev3.screen.print('%2dcm: %d mm' % (cm, avg))
ev3.speaker.beep(1200, 300)
print('Within about 20 mm everywhere = fine, nothing to change.')
print('Send me the printout if any error is bigger.')
wait(10000)                         # keep the results on the screen
