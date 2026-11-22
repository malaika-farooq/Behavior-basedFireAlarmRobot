# Behavior-Based Fire Alarm Robot

CSE 4360/5364 Project 2. A LEGO EV3 robot that explores an unknown maze, finds the fire (a candle on a black pad), raises an alarm, and blows the candle out with a fan.

Written in Pybricks MicroPython, using only basic motor and sensor functions.

## Hardware

| Part | Port |
|---|---|
| Left drive motor | A |
| Right drive motor | B |
| Fan (medium motor) | D |
| Touch sensor (front bumper) | 1 |
| Ultrasonic sensor (left side) | 2 |
| Gyro sensor | 3 |
| Color sensor (facing down) | 4 |

## Behaviors (subsumption, highest priority first)

1. **Extinguish:** moves onto the pad and runs the fan while the alarm sounds.
2. **Goal finding:** a dark floor reading, confirmed while stopped, means the fire is found and the alarm starts.
3. **Escape:** after a bumper hit, backs up and turns exactly 90° using the gyro. The direction comes from the left sonar and a short memory of directions found blocked.
4. **Depart:** at times turns away from the wall to cross the room and reach other areas.
5. **Wall following:** keeps the wall on the left. Where the wall ends, it turns left into the next room.
6. **Wander:** drives straight, holding its gyro heading, until it hits something.

## Files

- `main.py`: the robot program.
- `calibrate.py`: a straight-drive test that balances the left and right motors.

## Run

In VS Code with the ev3dev extension, choose **"2. Fire Robot main.py (download and run)"**. Place the robot and keep it still for the first 3 seconds.

Each run writes `log.csv` on the brick for debugging.
