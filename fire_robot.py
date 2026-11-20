#!/usr/bin/env python3
"""
Behavior-based Fire Alarm Robot (CSE 4360 Project 2) - ev3dev2 / Python 3

Coordination: SUBSUMPTION (fixed priority, highest first)
    1. Extinguish   - fire pad found: alarm + fan
    2. Escape       - front bumper hit: back up, turn away from the wall
    3. WallFollow   - wall seen on the left: keep a constant distance (P control)
    4. Wander       - nothing sensed: drive forward / random drift to find a wall

Goal finding (fire detection) is done by the color sensor looking at the floor.
It is a sensor "perception" check that feeds the Extinguish behavior.

Ports (change to match your robot):
    Motors : outB = left wheel, outC = right wheel, outA = fan (medium)
    Sensors: in1 = color (down), in2 = touch (front), in3 = ultrasonic (left)
"""
import random
import time

from ev3dev2.motor import (MoveTank, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D,
                           SpeedPercent)
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, TouchSensor, UltrasonicSensor
from ev3dev2.sound import Sound

# ---------------- configuration ----------------
LEFT_PORT, RIGHT_PORT, FAN_PORT = OUTPUT_A, OUTPUT_B, OUTPUT_D
COLOR_PORT, TOUCH_PORT, US_PORT = INPUT_1, INPUT_4, INPUT_3

BASE_SPEED = 30          # % cruising speed
WALL_TARGET_CM = 12      # desired distance to wall on the left
WALL_SEE_CM = 30         # closer than this = "a wall is there"
WALL_LOST_CM = 45        # farther than this while following = wall ended (corner)
KP = 1.6                 # wall-following proportional gain
MAX_STEER = 25

FIRE_THRESHOLD = 60      # RGB distance from the floor color that means "fire pad"
FIRE_CONFIRM = 3         # consecutive readings needed (rejects noise)
FAN_SECONDS = 10
CREEP_ROTATIONS = 0.4    # roll forward so the fan is on top of the candle
TICK = 0.05


# ---------------- hardware ----------------
tank = MoveTank(LEFT_PORT, RIGHT_PORT)
fan = MediumMotor(FAN_PORT)
color = ColorSensor(COLOR_PORT)
touch = TouchSensor(TOUCH_PORT)
us = UltrasonicSensor(US_PORT)
snd = Sound()
color.mode = 'RGB-RAW'

floor_rgb = (0, 0, 0)


def rgb():
    return color.raw  # (r, g, b)


def calibrate_floor():
    """Average the floor color at the start (robot must be on plain floor)."""
    global floor_rgb
    n, acc = 20, [0, 0, 0]
    for _ in range(n):
        v = rgb()
        for i in range(3):
            acc[i] += v[i]
        time.sleep(0.02)
    floor_rgb = tuple(a / n for a in acc)


def on_fire_pad():
    v = rgb()
    d = sum(abs(v[i] - floor_rgb[i]) for i in range(3))
    return d > FIRE_THRESHOLD


# ---------------- behaviors ----------------
# Each behavior has: active() -> bool, and act() which drives the motors.

class Extinguish:
    name = 'EXTINGUISH'

    def __init__(self):
        self.hits = 0

    def active(self):
        self.hits = self.hits + 1 if on_fire_pad() else 0
        return self.hits >= FIRE_CONFIRM

    def act(self):
        tank.off()
        snd.beep()                                   # alarm start
        snd.speak('Fire detected', play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)
        tank.on_for_rotations(SpeedPercent(15), SpeedPercent(15), CREEP_ROTATIONS)
        fan.on(SpeedPercent(100))
        end = time.time() + FAN_SECONDS
        while time.time() < end:                     # alarm while fan runs
            snd.beep(args='-f 1000 -l 200')
            time.sleep(0.3)
        fan.off()
        tank.off()
        raise SystemExit  # mission complete


class Escape:
    """Front bumper hit: back up, then turn right (wall is on our left)."""
    name = 'ESCAPE'

    def active(self):
        return touch.is_pressed

    def act(self):
        tank.on_for_rotations(SpeedPercent(-25), SpeedPercent(-25), 0.5)
        tank.on_for_rotations(SpeedPercent(30), SpeedPercent(-30), 0.4)  # ~90 deg right


class WallFollow:
    """Keep the wall on the LEFT at WALL_TARGET_CM with P control."""
    name = 'WALLFOLLOW'

    def __init__(self):
        self.following = False

    def active(self):
        d = us.distance_centimeters
        if d < WALL_SEE_CM:
            self.following = True
        elif d > WALL_LOST_CM:
            # lost the wall: if we were following it is an outside corner
            pass
        return self.following

    def act(self):
        d = us.distance_centimeters
        if d > WALL_LOST_CM:
            # outside corner / doorway: arc left to find the wall again
            tank.on(SpeedPercent(BASE_SPEED * 0.3), SpeedPercent(BASE_SPEED))
            return
        err = d - WALL_TARGET_CM            # + = too far from wall
        steer = max(-MAX_STEER, min(MAX_STEER, KP * err))
        # too far -> steer left (left wheel slower); too close -> steer right
        tank.on(SpeedPercent(BASE_SPEED - steer), SpeedPercent(BASE_SPEED + steer))


class Wander:
    """Free space: drive forward with a random drift until a wall appears."""
    name = 'WANDER'

    def __init__(self):
        self.next_change = 0
        self.turn = 0

    def active(self):
        return True  # lowest priority, always available

    def act(self):
        if time.time() > self.next_change:
            self.turn = random.randint(-8, 8)
            self.next_change = time.time() + random.uniform(1.0, 3.0)
        tank.on(SpeedPercent(BASE_SPEED + self.turn), SpeedPercent(BASE_SPEED - self.turn))


# ---------------- arbitration ----------------
def main():
    calibrate_floor()
    snd.speak('Searching', play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)
    behaviors = [Extinguish(), Escape(), WallFollow(), Wander()]  # priority order
    last = None
    try:
        while True:
            for b in behaviors:
                if b.active():
                    if b.name != last:
                        print(b.name)
                        last = b.name
                    b.act()
                    break
            time.sleep(TICK)
    except SystemExit:
        pass
    finally:
        tank.off()
        fan.off()


if __name__ == '__main__':
    main()
