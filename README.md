# Behavior-Based Fire Alarm Robot

CSE 4360/5364 Project 2. A LEGO EV3 robot explores an unknown house (rooms and balconies), finds the fire (a candle on a black pad), raises an alarm, and blows the candle out with a fan.

It's written in Pybricks MicroPython and uses only basic motor and sensor calls (no DriveBase or robotics module).

## Where the code is

| Path | What it is | Runs on |
|---|---|---|
| **`main.py`** | **The robot program: all behaviors and the subsumption control.** This is the project code. | EV3 |
| `tools/calibrate.py` | Sonar check: reads the left sonar at 10, 15, 20 and 30 cm and prints the error. | EV3 |
| `tools/stop.py` | Stops all motors if a cancelled run left one running. | EV3 |
| `tools/fan_test.py` | Runs the fan 5 s at full power and prints its real speed. | EV3 |
| `simulator/` | PC simulator that runs the real `main.py` in a 2-D arena. It isn't copied to the EV3. See [simulator/README.md](simulator/README.md). | PC |
| `Project2_Fire_Alarm_Robot_Report.docx` | The report. | |

## Hardware

| Part | Port | Placement |
|---|---|---|
| Left drive motor | A | left wheel |
| Right drive motor | B | right wheel |
| Fan (medium motor, geared up) | D | front |
| Touch sensor (front bumper) | 1 | front, bumper covers the front and both corners |
| Ultrasonic sensor | 2 | left side, about 10 cm (4 in) behind the axle |
| Gyro sensor | 3 | on the body |
| Color sensor (facing down) | 4 | about 4.2 cm (1.65 in) ahead of the axle |

The drive wheels are at the front and the caster is at the back.

## Behaviors (subsumption, highest priority first)

1. **Extinguish:** backs up 3.5 cm, then runs the fan for 60 s while sweeping 20° left and right, with the alarm beeping.
2. **Tipped:** if the floor disappears for 3 s, the robot has fallen over or been lifted. It stops and waits to be set back up.
3. **Goal finding:** stops on the first dark floor reading, then re-checks while standing still. If the reading is still dark, the fire is confirmed and the alarm starts.
4. **Escape:** after a bump or a stuck condition, backs up 4.5 cm and makes a gyro turn on the 90° grid.
5. **Wall following:** keeps the wall on the left at about 10 cm. Where the wall ends, it turns left into the opening.
6. **Wander:** drives straight on the grid until it hits something.

**Exploration** is set by `EXPLORE` in `main.py`:

- `'wall'` (default): first a tour that follows the left wall all the way round the house. Then it switches to `'map'` to cover the middle of the rooms.
- `'map'`: wall following plus an odometry visit map. The robot heads for the least-visited direction.

## Run

1. In VS Code with the ev3dev extension, choose **"2. Fire Robot main.py (download and run)"**.
2. Set the robot down square to the walls. The start direction defines the 90° grid.
3. Keep the robot still for the first 3 seconds.

Each run writes `log.csv` on the brick for debugging.
