# PC simulator

This runs the **real `main.py`** on your PC, in a 2-D model of an arena. It's not copied
to the EV3.

Use it from a terminal in the project folder (Python 3, nothing to install).

## Watch a run

```
python simulator/view.py --map lab --random          your lab, random pad + start
python simulator/view.py --map lab --random 7        repeat layout #7
python simulator/view.py --map house:random          a new random house each time
python simulator/view.py --map house:12 --random 3   repeat house 12, layout 3
python simulator/view.py --map lab --pad balcony --start left
python simulator/view.py --set "EXPLORE='map'"       try another strategy
```

- **Maps:** `lab`, `lab_old`, `rooms4`, `corridor`, and `house:N` (random houses).
- **Keys:** SPACE pauses, `+` and `-` change the speed, Esc quits.
- **The log:** each run writes `simulator/log.csv`, in the same format as the robot's log.

## Score many runs (found within 5 minutes)

```
python simulator/batch.py --map lab                  30 random layouts in the lab
python simulator/batch.py --map lab --named          the lab's named pad spots x starts
python simulator/batch.py --map houses:20            20 random houses, 3 layouts each
python simulator/batch.py --map lab,houses:10 --set "EXPLORE='map'"
```

Add `-v` to list every run, then replay any miss with `view.py`.

**Candle hit** counts the runs where the robot's body touched the candle jar, either before the
fire was confirmed or during the back-up and the first 8 s of fan sweeping.

## Where it went: `python simulator/heat.py lab 7`

This prints an ASCII picture of one run (map `lab`, layout #7).

## The lab map: `simulator/arena.py`

- **The number to measure:** `TILE_FT`, one floor tile in feet. The arena is 7 x 7 tiles
  and every wall sits on a tile line, so the whole map scales from this one number.
- **The pad** is half a tile square (`PAD_FT`), as the project sheet says, with the candle jar
  in its middle.
- **Check the narrow places:** the balcony opening, the bottom room, and the gap between
  the T-wall and the top-right room's wall. They decide the results.
