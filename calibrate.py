#!/usr/bin/env pybricks-micropython
"""STRAIGHT-DRIVE test: finds out if one wheel motor is stronger than the other.
Press the TOUCH sensor (port 1), step back: the robot drives straight for 3 s at the
same power as main.py. The wheel angle counts show which side is stronger, and the
gyro shows which way it curved. Ports/directions must match main.py."""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch

ev3 = EV3Brick()
left = Motor(Port.A, Direction.CLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
touch = TouchSensor(Port.S1)
gyro = GyroSensor(Port.S3)
sw = StopWatch()

BASE = 35                # keep equal to main.py
LEFT_GAIN = 90           # keep equal to main.py (re-test: should now drive straight)
RIGHT_GAIN = 100
RUNS = 3
DRIVE_MS = 3000


def step(msg):
    print(msg)
    ev3.screen.print(msg[:20])
    while touch.pressed():
        wait(20)
    while not touch.pressed():
        wait(20)
    wait(1500)                       # let the robot stop wobbling from the press


def gyro_bias(ms):
    t = sw.time()
    n = 0
    total = 0
    while sw.time() - t < ms:
        total += gyro.speed()
        n += 1
        wait(10)
    return total / n


print('=== STRAIGHT TEST: robot on the floor, 1 m of clear space ===')
sum_l = 0
sum_r = 0
for n in range(RUNS):
    step('Press touch, step back: drive straight')
    left.reset_angle(0)
    right.reset_angle(0)
    wait(150)
    bias = gyro_bias(250)
    ang = 0.0
    last = sw.time()
    t0 = last
    p = 0
    while sw.time() - t0 < DRIVE_MS:
        if p < BASE:
            p += 1                   # gentle ramp, like main.py
        left.dc(p * LEFT_GAIN // 100)
        right.dc(p * RIGHT_GAIN // 100)
        now = sw.time()
        ang += (gyro.speed() - bias) * (now - last) / 1000.0
        last = now
        wait(20)
    left.stop()
    right.stop()
    wait(300)
    l = left.angle()
    r = right.angle()
    sum_l += l
    sum_r += r
    print('run %d: left %d  right %d  (stronger: %s)  gyro heading change %.1f deg' % (
        n + 1, l, r, 'LEFT' if l > r else 'RIGHT', ang))
    print('        REAL: did it curve LEFT or RIGHT, and by how much?')

if sum_l > 0 and sum_r > 0:
    if sum_l > sum_r:
        print('Suggested: LEFT_GAIN = %d, RIGHT_GAIN = 100' % (100 * sum_r // sum_l))
    else:
        print('Suggested: LEFT_GAIN = 100, RIGHT_GAIN = %d' % (100 * sum_l // sum_r))
print('Done. Send me the printout.')
