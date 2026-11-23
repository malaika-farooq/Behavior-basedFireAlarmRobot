# Behavior-Based Fire Alarm Robot

CSE 4360/5364 Project 2. A LEGO EV3 robot that explores an unknown maze, finds the fire (a candle on a black pad), raises an alarm, and blows the candle out with a fan.

Written in Pyb
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
2. **Tipped:** if the floor disappears for 3 s, the robot has fallen over or been lifted. It stops and waits to be set back up.
3. **Goal finding:** a dark floor reading, confirmed while stopped, means the fire is found and the alarm starts.
4. **Escape:** after a hit, backs up and turns using the gyro by the left-hand rule: left if the left is open, otherwise right, and a 180° turn if it is hit again right away.
5. **Wall following:** keeps the wall on the left while staying on the 90° grid. Where the wall ends, it turns left into the next room. After a full loop it turns away once.
6. **Wander:** drives straight on the grid until it hits something.

The robot follows the **left-hand rule** for exploration. The start heading defines a 90° grid, so place the robot square to the walls.

## Files

- `main.py`: the robot program.
- `calibrate.py`: a straight-drive test that balances the left and right motors.

## Run

In VS Code with the ev3dev extension, choose **"2. Fire Robot main.py (download and run)"**. Place the robot and keep it still for the first 3 seconds.

Each run writes `log.csv` on the brick for debugging.
