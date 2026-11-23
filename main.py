#!/usr/bin/env pybricks-micropython
"""
Behavior-based Fire Alarm Robot - CSE 4360 Project 2 (Pybricks EV3 MicroPython)

Only basic motor / sensor functions are used (no robotics / DriveBase module).
No map, no localization: reactive behaviors plus a gyro heading.

EXPLORATION (EXPLORE setting, default 'wall'):
  'wall' : a LEFT-WALL-FOLLOWER TOUR first. The walls of a house are connected, so following
           them enters every room and balcony. Then it switches to 'map' for the room middles.
  'map'  : an odometry VISIT MAP (gyro heading + wheel distance, 25 cm cells): at every bump,
           wall end and every 2 new cells it heads for the least visited direction.
All straight paths are on the grid lined up with the start heading (gyro), so errors from
slip or a bad turn never carry into the next move.

BEHAVIORS (priority order = subsumption, highest first)
    Extinguish  fire identified   -> back up 3.5 cm, fan on 60 s sweeping +-20 deg, alarm
    Tipped      no floor for 3 s  -> robot fell over / lifted: stop, beep, wait to be set up
    GoalFind    dark pad          -> stop AT ONCE, re-check standing still, start the alarm
    Escape      bumper / stuck    -> back up 4.5 cm, then a gyro turn on the 90-degree grid
                                     ('wall': always right, so the wall ends up on the left;
                                      'map': toward the least visited side)
    WallFollow  wall on the left  -> stay on the grid line, lean at most 10 deg to keep the
                                     wall at ~10 cm. Wall ENDS (doorway / corner) ->
                                     drive on until the body clears it, turn LEFT into it.
    Wander      always            -> drive straight on the grid until something is hit

Ports: motors A = left wheel, B = right wheel, D = fan;
       sensors 1 = touch (bumper), 2 = ultrasonic (LEFT side), 3 = gyro, 4 = color (down)
"""
import gc
import sys
try:
    import math
except ImportError:
    import umath as math

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch

# ======================= configuration =======================
LEFT_PORT, RIGHT_PORT, FAN_PORT = Port.A, Port.B, Port.D
COLOR_PORT, SONAR_PORT, TOUCH_PORT, GYRO_PORT = Port.S4, Port.S2, Port.S1, Port.S3
LEFT_DIR = Direction.CLOCKWISE          # flip if a wheel runs backward
RIGHT_DIR = Direction.CLOCKWISE
FAN_DIR = Direction.CLOCKWISE           # flip if the fan blows backward

# --- robot geometry (measured, cm from the AXLE CENTER = midway between the drive wheels) ---
WHEEL_DIA_MM = 54        # 2.125 in wheels
# front 7.5 cm, rear 16 cm, width 17 cm, wheel to wheel 15 cm; sonar ~10 cm (4 in) behind the axle,
# color sensor ~4.2 cm (1.65 in) ahead of the axle.
# Turning in place the corners swing out to 11.3 cm (front) and 18.1 cm (rear): keep ~10 cm
# from the left wall, and back up 6 cm so the front corners clear a wall before turning.


def mm_to_deg(mm):
    """Wheel degrees for a straight distance in mm (pi ~ 3.14)."""
    return mm * 36000 // (314 * WHEEL_DIA_MM)


# --- driving (percent motor power) ---
BASE = 50                # cruising power (35 = old, slower; 50 found the fire ~2x sooner in the simulator)
LEFT_GAIN = 975        # 97.5% of full power (matched to the right wheel)
RIGHT_GAIN = 1000       # 100% of full power
SLEW = 10                # max power change per tick: gentle starts on the slick sheet
MIN_DUTY = 22            # a wheel below this power stalls on the sheet
BACK_PWR = 30
BACKUP_DEG = mm_to_deg(45)      # back up 4.5 cm before every bump turn (front corners swing
                                # out 11.3 cm when turning: bumper at 7.5 cm needs >= 3.8 cm)

# --- heading (gyro) ---
HOLD_KP_X10 = 15         # straight driving: 1.5 power per degree off the target heading
HOLD_KI_X1000 = 30       # slowly removes a constant offset (one motor stronger)
HOLD_MAX = 10            # max power difference used to hold the heading
SIGN_CHECK_DEG = 10      # startup: small right turn to learn which way the gyro counts

# --- turns (gyro) ---
TURN_PWR = 45            # fast part of a turn (the last 30 deg is always slow = accurate)
TURN_SLOW_PWR = 22       # last 30 deg and trims: slow, so it does not coast past
SLOW_ZONE_DEG = 30
GYRO_LEAD = 1
TRIM_TOL_DEG = 1.5
TRIM_MAX_MS = 1200
TURN_MAX_MS = 3500
CHECK_MS = 200           # stuck check inside a turn
STUCK_MDEG = 4000        # turned less than 4 deg in CHECK_MS = held by the sheet / a wall
BOOST_STEP = 10
MAX_BOOST = 30           # kept low: pushing a tall robot hard into a wall tips it
ROCK_MS = 250
STUCK_TOL_DEG = 15       # a turn that ends further off than this = stuck
RECOVER_DEG = mm_to_deg(60)     # stuck turn: back up 6 cm, try once more

# --- exploration mode ---
EXPLORE = 'wall'        # (default) see 'wall' below. Others: 'map', 'random', 'left'
                        # 'wall'  : LEFT-WALL FOLLOWER TOUR: bump -> turn right, wall ends ->
                        #           turn left (wrap round wall ends). The walls of the house are
                        #           all connected, so one tour enters every room / balcony. After
                        #           the tour it switches to 'map' for the middle of the rooms.
WRAP_MM = 450           # after a wall-end left turn: no wall seen within this -> left again
TOUR_MIN_MM = 6000      # tour done = back within TOUR_BACK_MM of where it started after
TOUR_BACK_MM = 600      #   driving at least TOUR_MIN_MM ...
TOUR_MAX_MS = 240000    #   ... or after this long anyway
                        # 'map'     : wall following + an odometry visit map: always head for
                        #           the least visited cells (turns, openings, crossing rooms)
                        # 'random': wall following + random choices (previous version)
                        # 'left'  : strict left-hand rule (first version)
CELL_MM = 250           # map cell (about one robot length)
MAP_LOOK = 4            # cells looked at ahead when scoring a direction (1 m)
BUMP_MARK = 3           # a bump adds this to the cell in front (a wall is there)
STEER_EVERY = 2         # open floor: re-check the map every 2 new cells
STEER_MARGIN = 4        # ...and turn only if a side scores this much lower than straight
SIDE_OPEN_MM = 450      # turn LEFT in the open only if the sonar sees this much free space
OPENING_PCT = 70        # random mode: % chance to turn LEFT into an opening when a wall ends
TURN180_PCT = 25        # random mode: % chance a hit turn is 180 instead of 90 (on the grid)

# --- left sonar (mm) ---
SONAR_CAP = 1000
LEFT_OPEN_MM = 150       # bump decision: left is open only if no wall within 15 cm (more than
                         # the 10 cm following distance, so the followed wall counts as a wall)
WALL_SEE = 300           # a wall closer than this on the left = start following it
WALL_TARGET = 100        # distance to keep from the left wall (10 cm: a right turn swings the
                         # rear corner ~9 cm out to the left, so 7 cm would scrape)
CLOSE_MM = 60           # closer than 6 cm: lean away at full strength
WALL_LOST = 380          # farther than this while following = the wall ended (door/corner)
LOST_CONFIRM_MS = 500
MIN_FOLLOW_MS = 1500     # only a wall followed this long counts as "ended"
CORNER_ADVANCE_DEG = mm_to_deg(30)   # sonar sits 10 cm behind the axle, so the front is
                                     # already past the opening: drive on only ~3 cm
LEAN_MDEG_PER_MM = 300   # wall following: 0.3 deg of lean per mm of distance error...
LEAN_MAX_DEG = 10        # ...at most 10 deg off the grid line
LOOP_TURNS = 4           # 4 turns the same way (360 deg) = going round in a loop
LOOP_BREAK_MS = 2000     # leave the loop after following the next wall this long
DETACH_MS = 20000        # follow a wall this long, then leave it to cross the room
CROSS_MS = 8000         # time spent crossing the room straight after leaving a wall

# --- treated as a bump (the bumper did not catch it) ---
REPEAT_MS = 3000         # BUMPER hit again this soon after a turn...
REPEAT_DEG = mm_to_deg(150)     # ...and less than 15 cm driven since = boxed in
STALL_MS = 1000          # a wheel powered forward but turning less than...
STALL_DEG = 15           # ...this in STALL_MS = stuck
WEDGE_MM = 45            # sonar this close...
WEDGE_TICKS = 55         # ...for ~2 s = wedged against the wall
SIDEHIT_DEG = 15         # knocked this far off the heading while driving...
SIDEHIT_TICKS = 10       # ...for ~0.35 s = a corner hit the side
NOPROG_MS = 4000         # driving forward, but the side sonar AND the floor reading did not
NOPROG_MM = 10           # change (sonar within 1 cm, floor within 3) for 4 s = wheels spin
NOPROG_FLOOR = 3         # but the robot does not move (only when the sonar sees something)
NOPROG_SLIP_DEG = 45     # ...AND the wheels say it turned this much more than the gyro saw
                         # (a slipping wheel). Driving down a lane also freezes the sonar and
                         # floor, but the wheels and gyro agree there, so it is not stuck.
WATCHDOG_MS = 25000      # driving forward 25 s without any turn or hit = stuck (backstop)
PINNED_MS = 2500         # steering at its limit this long while the gyro does not move...
PINNED_DEG = 3           # ...more than this = pushing into something (wheels slipping)

# --- fire pad (BLACK) and tip-over ---
PAD_MAX = 3              # pad = reading below this (measured: pad 0-1, floor 13-25, shadows 2-5)
MIN_FLOOR = 8            # floor must read at least this at startup or detection is off
FIRE_CONFIRM = 1         # dark ticks in a row before GoalFind STOPS to check. Was 8: the
                         # logs (runs 5-7) show the robot rolled 4-7 cm onto the pad before it
                         # stopped and the front hit the candle. Now it stops on the first dark
                         # reading (~1 cm); the re-check standing still rejects false alarms.
PAD_WATCH_N = 4          # during turns / back-ups: this many dark reads in a row = pad
TIP_MS = 3000            # dark (no floor) this long without a fire = tipped over / lifted
UPRIGHT_MS = 1500        # floor seen again this long = standing again
FAN_MS = 60000          # fan runs this long while sweeping
SWEEP_DEG = 20          # sweep this far left and right of the fire direction
FIRE_BACK_MM = 35       # back up this far from the pad before the fan starts

START_DELAY_MS = 3000
TICK_MS = 30

LOG_FILE = 'log.csv'
LOG_EVERY_MS = 200
LOG_FLUSH_EVERY = 5
LOG_MAX_LINES = 5000


# ======================= hardware =======================
ev3 = EV3Brick()
left = Motor(LEFT_PORT, LEFT_DIR)
right = Motor(RIGHT_PORT, RIGHT_DIR)
fan = Motor(FAN_PORT, FAN_DIR)
color = ColorSensor(COLOR_PORT)
sonar = UltrasonicSensor(SONAR_PORT)
touch = TouchSensor(TOUCH_PORT)
try:
    gyro = GyroSensor(GYRO_PORT)
except OSError:
    gyro = None
clock = StopWatch()


# ======================= state shared by the behaviors =======================
class Sense:
    dist = SONAR_CAP     # left sonar, mm (smoothed)
    bump = False
    bright = 0           # floor reflection
    floor = 15
    thresh = PAD_MAX
    detect_ok = True
    fire = False
    hit = False          # something was hit (bumper, stall, wedge, side hit): Escape takes it
    hit_bump = False     # the hit was the front bumper (not a stall / wedge / side hit)
    turn_epoch = 0       # counts turns: wall following restarts after every turn
    np_t = 0             # no-progress check: window start and the readings then
    np_d = 0
    np_b = 0
    np_l = 0
    np_r = 0
    np_y = 0
    wd_t = 0             # watchdog: start of the current forward drive
    busy = False         # Escape is backing up (color sensor not trusted)
    wedge_ticks = 0
    side_ticks = 0
    stall_t = 0
    stall_l = 0
    stall_r = 0
    dark_since = 0       # time the floor went dark (0 = not dark)
    turn_sum = 0         # net 90-deg turns, left = +1, right = -1 (loop detection)
    follow_off = False   # no wall following (crossing the room) until off_until
    mode = EXPLORE       # current exploration mode (the wall tour switches to 'map')
    odo_mm = 0.0         # distance driven (odometry)
    wrap_n = 0           # wall tour: left turns after a wall end without a wall yet
    wrap_x = 0.0
    wrap_y = 0.0
    tour_t = 0           # wall tour start time (first wall), 0 = not started
    tour_x = 0.0
    tour_y = 0.0
    x = 0.0              # odometry position (mm) from the start, x = start heading
    y = 0.0              # y = to the left of the start heading
    oL = 0               # wheel angles at the last odometry update
    oR = 0
    cell = (0, 0)
    new_cells = 0        # cells entered since the last turn / map check
    sat = False          # heading hold at its steering limit
    pin_t = 0
    pin_y = 0
    pad_seen = False     # pad seen during a turn / back-up (GoalFind checks it)
    dark_n = 0
    off_until = 0        # time (ms) the crossing ends


S = Sense()


class Cmd:
    l = 0
    r = 0


C = Cmd()


# ======================= log =======================
class Log:
    f = None
    n = 0
    next = 0


def log_open():
    try:
        Log.f = open(LOG_FILE, 'w')
        Log.f.write('t_ms,tag,behavior,dist_mm,bump,bright,floor,thresh,'
                    'dutyL,dutyR,angL,angR,note\n')
    except OSError:
        Log.f = None


def log(tag, beh, note=''):
    f = Log.f
    if f is None or Log.n >= LOG_MAX_LINES:
        return
    f.write('%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n' % (
        clock.time(), tag, beh, S.dist, int(S.bump), S.bright, S.floor,
        S.thresh, C.l, C.r, left.angle(), right.angle(), note))
    Log.n += 1
    if tag != 'S' or Log.n % LOG_FLUSH_EVERY == 0:
        f.flush()


def log_close():
    if Log.f is not None:
        try:
            Log.f.close()
        except OSError:
            pass
        Log.f = None


# ======================= motors =======================
def ramp(cur, target):
    if target > cur + SLEW:
        return cur + SLEW
    if target < cur - SLEW:
        return cur - SLEW
    return target


def min_duty(p):
    if 0 < p < MIN_DUTY:
        return MIN_DUTY
    if -MIN_DUTY < p < 0:
        return -MIN_DUTY
    return p


def drive(l, r):
    C.l = ramp(C.l, min_duty(l * LEFT_GAIN // 1000))
    C.r = ramp(C.r, min_duty(r * RIGHT_GAIN // 1000))
    left.dc(C.l)
    right.dc(C.r)


def stop_drive():
    C.l = 0
    C.r = 0
    left.stop(Stop.BRAKE)
    right.stop(Stop.BRAKE)


def spin(d, pwr):
    """Turn in place: d=+1 left, d=-1 right."""
    drive(-pwr * d, pwr * d)


# ======================= gyro heading =======================
class Heading:
    """One heading for the whole run, integrated from the gyro RATE (milli-degrees,
    RIGHT turn = positive). `target` is where the robot should point."""
    yaw = 0
    target = 0
    bias = 0
    last = 0
    gsign = 1
    iacc = 0


def wrap(m):
    """Angle difference into -180..+180 deg (milli-deg): never turn the long way round."""
    m = (m + 180000) % 360000 - 180000
    return m


def snap90(m):
    """Nearest grid direction (multiple of 90 deg), milli-deg."""
    return (m + 45000) // 90000 * 90000


def yaw_update():
    if gyro is None:
        return
    now = clock.time()
    Heading.yaw += Heading.gsign * (gyro.speed() * 1000 - Heading.bias) * (now - Heading.last) // 1000
    Heading.last = now


def gyro_zero():
    """Robot STOPPED: re-measure the gyro zero offset (it drifts). ~0.25 s."""
    if gyro is None:
        return
    yaw_update()
    wait(150)
    total = 0
    for _ in range(10):
        total += gyro.speed()
        wait(10)
    Heading.bias = total * 100
    Heading.last = clock.time()


def heading_error():
    return wrap(Heading.target - Heading.yaw)


def drive_straight(base):
    """Forward, holding the target heading (corrects slip and motor differences)."""
    yaw_update()
    if gyro is None:
        drive(base, base)
        return
    err = heading_error() // 1000                  # degrees, + = target is to the right
    lim = HOLD_MAX * 1000 // HOLD_KI_X1000
    Heading.iacc = max(-lim, min(lim, Heading.iacc + err))
    steer = err * HOLD_KP_X10 // 10 + Heading.iacc * HOLD_KI_X1000 // 1000
    steer = max(-HOLD_MAX, min(HOLD_MAX, steer))
    S.sat = abs(steer) >= HOLD_MAX
    drive(base + steer, base - steer)


def turn_to(goal):
    """Rotate in place to the heading `goal` (milli-deg), the SHORT way. Fast, slow near the
    end, settle, trim. If the robot does not rotate (held by the sheet / a wall) add power
    in steps, rock back if needed. Returns the final error (milli-deg)."""
    stop_drive()
    reset_wheels()
    gyro_zero()
    Heading.iacc = 0
    t0 = clock.time()
    boost = 0
    peak = 0
    rocks = 0
    chk_t = t0
    chk_e = abs(wrap(goal - Heading.yaw))
    while clock.time() - t0 < TURN_MAX_MS:
        yaw_update()
        err = wrap(goal - Heading.yaw)
        if abs(err) <= GYRO_LEAD * 1000:
            break
        if pad_watch():
            stop_drive()
            return err
        d = -1 if err > 0 else 1                   # goal to the right -> turn right
        now = clock.time()
        if now - chk_t >= CHECK_MS:
            if chk_e - abs(err) < STUCK_MDEG:
                if boost >= MAX_BOOST:
                    t_r = clock.time()
                    while clock.time() - t_r < ROCK_MS:
                        yaw_update()
                        spin(-d, TURN_PWR + MAX_BOOST)
                        wait(10)
                    rocks += 1
                    boost = 0
                else:
                    boost = min(boost + BOOST_STEP, MAX_BOOST)
            else:
                boost = max(boost - BOOST_STEP, 0)
            peak = max(peak, boost)
            chk_t = clock.time()
            chk_e = abs(wrap(goal - Heading.yaw))
        spin(d, (TURN_PWR if abs(err) > SLOW_ZONE_DEG * 1000 else TURN_SLOW_PWR) + boost)
        wait(10)
    stop_drive()
    t1 = clock.time()
    while clock.time() - t1 < 250:                 # settle: integrate the coast
        yaw_update()
        wait(10)
    t2 = clock.time()
    trims = 0
    boost = 0
    chk_t = t2
    chk_e = abs(wrap(goal - Heading.yaw))
    while clock.time() - t2 < TRIM_MAX_MS:
        yaw_update()
        err = wrap(goal - Heading.yaw)
        if abs(err) <= TRIM_TOL_DEG * 1000:
            break
        if pad_watch():
            stop_drive()
            return err
        now = clock.time()
        if now - chk_t >= CHECK_MS:
            if chk_e - abs(err) < STUCK_MDEG // 4:
                boost = min(boost + BOOST_STEP, MAX_BOOST)
            chk_t = now
            chk_e = abs(err)
        trims += 1
        spin(-1 if err > 0 else 1, TURN_SLOW_PWR + boost)
        wait(10)
    stop_drive()
    reset_wheels()
    t3 = clock.time()
    while clock.time() - t3 < 150:
        yaw_update()
        wait(10)
    err = wrap(goal - Heading.yaw)
    log('E', 'TURN', 'to %d: now %d, error %d, trims %d boost %d rocks %d%s' % (
        goal // 1000, Heading.yaw // 1000, err // 1000, trims, peak, rocks,
        ' STUCK' if abs(err) > STUCK_TOL_DEG * 1000 else ''))
    return err


class Map:
    cells = {}           # (ix, iy) -> visits (+ BUMP_MARK where a wall was hit)


def odo():
    """Dead reckoning: gyro heading + the SMALLER wheel distance (a slipping wheel spins
    more, so the smaller one is closer to the truth). Turning in place adds nothing."""
    la = left.angle()
    ra = right.angle()
    dl = la - S.oL
    dr = ra - S.oR
    S.oL = la
    S.oR = ra
    if dl == 0 or dr == 0 or (dl > 0) != (dr > 0):
        return
    d = min(abs(dl), abs(dr)) * (1 if dl > 0 else -1)
    mm = d * 314 * WHEEL_DIA_MM / 36000
    a = math.radians(-Heading.yaw / 1000)
    S.x += mm * math.cos(a)
    S.y += mm * math.sin(a)
    S.odo_mm += abs(mm)
    c = (int(S.x // CELL_MM), int(S.y // CELL_MM))
    if c != S.cell:
        S.cell = c
        Map.cells[c] = Map.cells.get(c, 0) + 1
        S.new_cells += 1


def reset_wheels():
    odo()
    left.reset_angle(0)
    right.reset_angle(0)
    S.oL = 0
    S.oR = 0


def grid_step(h):
    """Cell step (dx, dy) for a grid heading h (milli-deg, right turn = +)."""
    k = (snap90(h) // 90000) % 4
    return ((1, 0), (0, -1), (-1, 0), (0, 1))[k]


def score(h):
    """How much the robot has already been in the cells ahead along heading h (lower =
    newer ground). Near cells count more."""
    dx, dy = grid_step(h)
    cx, cy = S.cell
    t = 0
    for i in range(1, MAP_LOOK + 1):
        t += (MAP_LOOK + 1 - i) * Map.cells.get((cx + dx * i, cy + dy * i), 0)
    return t


def mark_bump():
    """A bump: the cell just in front holds a wall, make it less attractive."""
    a = math.radians(-Heading.yaw / 1000)
    c = (int((S.x + 150 * math.cos(a)) // CELL_MM), int((S.y + 150 * math.sin(a)) // CELL_MM))
    Map.cells[c] = Map.cells.get(c, 0) + BUMP_MARK


def pad_watch():
    """Used inside turns and back-ups (the main loop is not running there)."""
    if S.fire or not S.detect_ok:
        return False
    if color.reflection() < S.thresh:
        S.dark_n += 1
        if S.dark_n >= PAD_WATCH_N:
            S.pad_seen = True
            return True
    else:
        S.dark_n = 0
    return False


def back_up(deg):
    reset_wheels()
    t = clock.time()
    while clock.time() - t < 1500 and -(left.angle() + right.angle()) // 2 < deg:
        yaw_update()
        if pad_watch():
            break
        drive(-BACK_PWR, -BACK_PWR)
        wait(10)
    stop_drive()
    reset_wheels()


class Rng:
    seed = 12345


def rand(lo, hi):
    """Small integer random number generator (no floats)."""
    Rng.seed = (Rng.seed * 1103515245 + 12345) & 0x7FFFFFFF
    return lo + (Rng.seed >> 8) % (hi - lo + 1)


def turn_rel(d, robot_deg):
    """Random-mode turn: d=+1 left, -1 right, by robot_deg from the current heading
    (no 90-degree grid). Same gyro turn and stuck recovery as turn_by."""
    S.turn_epoch += 1
    if gyro is None:
        turn_by(d, robot_deg)
        return
    Heading.target = Heading.target - d * robot_deg * 1000
    err = turn_to(Heading.target)
    if S.pad_seen:
        return
    if abs(err) > STUCK_TOL_DEG * 1000:
        log('E', 'TURN', 'stuck %d deg short: back up and retry' % (abs(err) // 1000))
        back_up(RECOVER_DEG)
        err = turn_to(Heading.target)
    if abs(err) > STUCK_TOL_DEG * 1000:
        Heading.target = Heading.yaw               # carry on from where it points
        log('E', 'TURN', 'gave up: continue from the current heading')


def turn_by(d, robot_deg):
    """Turn 90 or 180 deg (d=+1 left, -1 right) from the GRID line we are on, so any error
    from before is taken out by this turn. Counts the turn for loop detection."""
    S.turn_sum += d * (robot_deg // 90)
    S.turn_epoch += 1
    if gyro is None:                               # fallback: wheel encoders
        stop_drive()
        reset_wheels()
        target = robot_deg * 440 // 100
        t0 = clock.time()
        while clock.time() - t0 < TURN_MAX_MS and (right.angle() - left.angle()) * d // 2 < target:
            spin(d, TURN_PWR)
            wait(10)
        stop_drive()
        return
    Heading.target = snap90(Heading.target) - d * robot_deg * 1000
    S.new_cells = 0
    err = turn_to(Heading.target)
    if S.pad_seen:
        return
    if abs(err) > STUCK_TOL_DEG * 1000:
        log('E', 'TURN', 'stuck %d deg short: back up and retry' % (abs(err) // 1000))
        back_up(RECOVER_DEG)
        if S.pad_seen:
            return
        err = turn_to(Heading.target)
    if abs(err) > STUCK_TOL_DEG * 1000:            # could not get there: drive on the grid
        Heading.target = snap90(Heading.yaw)       # line we actually face instead
        log('E', 'TURN', 'gave up: continue on the grid line at %d' % (Heading.target // 1000))


def advance(deg):
    """Straight on the grid for a fixed distance (wheel deg). False if something was hit."""
    reset_wheels()
    t = clock.time()
    while clock.time() - t < 2500 and (left.angle() + right.angle()) // 2 < deg:
        if pad_watch():
            stop_drive()
            return False
        if touch.pressed():
            stop_drive()
            S.hit = True
            S.hit_bump = True
            return False
        drive_straight(BASE)
        wait(10)
    stop_drive()
    reset_wheels()
    return True


def gyro_startup():
    """Standing still: measure the gyro zero, make a small right turn to learn which way
    the gyro counts (depends on the mount), turn back. The start heading is grid line 0."""
    if gyro is None:
        return
    Heading.last = clock.time()
    gyro_zero()
    raw = 0
    last = clock.time()
    t0 = last
    while clock.time() - t0 < 1500 and abs(raw) < SIGN_CHECK_DEG * 1000:
        now = clock.time()
        raw += (gyro.speed() * 1000 - Heading.bias) * (now - last) // 1000
        last = now
        spin(-1, TURN_PWR)
        wait(10)
    stop_drive()
    if abs(raw) < 3000:
        log('E', 'GYRO', 'sign check: almost no rotation seen (%d mdeg) - check the gyro' % raw)
        print('GYRO: no rotation seen in the sign check - check mount/cable')
    Heading.gsign = 1 if raw >= 0 else -1
    Heading.yaw = abs(raw)
    Heading.last = clock.time()
    Heading.target = 0
    log('E', 'GYRO', 'sign %d, bias %d mdeg/s' % (Heading.gsign, Heading.bias))
    turn_to(0)


# ======================= sensing (once per tick) =======================
def average_reflection(n):
    total = 0
    for _ in range(n):
        total += color.reflection()
        wait(20)
    return total // n


def sense():
    d = sonar.distance()
    if d > SONAR_CAP:
        d = SONAR_CAP
    S.dist = (S.dist + d) // 2
    pressed = touch.pressed()
    if pressed and not S.bump:                     # a NEW press only
        S.hit = True
        S.hit_bump = True
    S.bump = pressed
    S.bright = color.reflection()
    yaw_update()
    odo()
    now = clock.time()

    # floor dark: the pad, or the robot tipped over / lifted (sensor off the floor)
    if S.bright < S.thresh:
        if S.dark_since == 0:
            S.dark_since = now
    else:
        S.dark_since = 0

    forward = C.l >= MIN_DUTY and C.r >= MIN_DUTY

    # stuck: a wheel powered forward but (almost) not turning
    la = left.angle()
    ra = right.angle()
    if not forward or la < S.stall_l or ra < S.stall_r:
        S.stall_t = now
        S.stall_l = la
        S.stall_r = ra
    elif now - S.stall_t >= STALL_MS:
        if la - S.stall_l < STALL_DEG or ra - S.stall_r < STALL_DEG:
            S.hit = True
            log('E', 'STALL', 'wheel stuck (L %d R %d deg in %d ms)' % (
                la - S.stall_l, ra - S.stall_r, now - S.stall_t))
        S.stall_t = now
        S.stall_l = la
        S.stall_r = ra

    # wedged: the left side is pressed against a wall for ~2 s
    if S.dist < WEDGE_MM:
        S.wedge_ticks += 1
        if S.wedge_ticks >= WEDGE_TICKS:
            S.wedge_ticks = 0
            S.hit = True
            log('E', 'WEDGE', 'left side against the wall')
    else:
        S.wedge_ticks = 0

    # side hit: knocked far off the heading while driving forward (a corner hit the side)
    if gyro is not None and forward and abs(heading_error()) > SIDEHIT_DEG * 1000:
        S.side_ticks += 1
        if S.side_ticks >= SIDEHIT_TICKS:
            S.side_ticks = 0
            S.hit = True
            log('E', 'SIDEHIT', 'knocked %d deg off course' % (heading_error() // 1000))
    else:
        S.side_ticks = 0

    # no progress: wheels turning forward but nothing the robot senses changes
    if (not forward or S.dist >= SONAR_CAP - 100 or abs(S.dist - S.np_d) > NOPROG_MM or
            abs(S.bright - S.np_b) > NOPROG_FLOOR or la < S.np_l or ra < S.np_r):
        S.np_t = now
        S.np_d = S.dist
        S.np_b = S.bright
        S.np_l = la
        S.np_r = ra
        S.np_y = Heading.yaw
    elif now - S.np_t >= NOPROG_MS:
        # wheel-implied rotation (deg) = wheel deg difference * wheel dia / (2 * track 150 mm)
        wheel_turn = abs((la - S.np_l) - (ra - S.np_r)) * WHEEL_DIA_MM // 300
        gyro_turn = abs(Heading.yaw - S.np_y) // 1000
        if wheel_turn - gyro_turn > NOPROG_SLIP_DEG:
            S.hit = True
            log('E', 'NOPROGRESS', 'sonar+floor frozen %d ms, wheels %d deg vs gyro %d deg: slipping' % (
                NOPROG_MS, wheel_turn, gyro_turn))
        S.np_t = now                               # lane driving: start a new window
        S.np_d = S.dist
        S.np_b = S.bright
        S.np_l = la
        S.np_r = ra
        S.np_y = Heading.yaw

    # pinned: steering at its limit but the robot does not turn (front corner on a wall)
    if forward and S.sat:
        if S.pin_t == 0:
            S.pin_t = now
            S.pin_y = Heading.yaw
        elif now - S.pin_t >= PINNED_MS:
            if abs(Heading.yaw - S.pin_y) < PINNED_DEG * 1000:
                S.hit = True
                log('E', 'PINNED', 'steering at the limit %d ms, gyro still' % PINNED_MS)
            S.pin_t = 0
    else:
        S.pin_t = 0

    # watchdog: in a closed maze a moving robot hits something long before this
    if not forward:
        S.wd_t = now
    elif now - S.wd_t >= WATCHDOG_MS:
        S.wd_t = now
        S.hit = True
        log('E', 'WATCHDOG', 'driving %d ms without a turn or hit: assume stuck' % WATCHDOG_MS)


# ======================= behaviors =======================
class Wander:
    """No wall beside us: drive straight on the grid until something is hit."""
    name = 'WANDER'

    def active(self):
        return True

    def act(self):
        if S.mode == 'wall' and 0 < S.wrap_n < 3:
            if (S.x - S.wrap_x) ** 2 + (S.y - S.wrap_y) ** 2 > WRAP_MM * WRAP_MM:
                log('E', self.name, 'no wall after the corner: wrap left again')
                turn_by(1, 90)
                S.wrap_n += 1
                S.wrap_x, S.wrap_y = S.x, S.y
                return
        if S.mode == 'map' and S.new_cells >= STEER_EVERY:
            S.new_cells = 0
            h = snap90(Heading.target)
            s0 = score(h)
            sl = score(h - 90000) if S.dist >= SIDE_OPEN_MM else 9999
            sr = score(h + 90000) + 2                  # right side is not seen: small penalty
            if min(sl, sr) <= s0 - STEER_MARGIN:
                d = 1 if sl <= sr else -1
                log('E', self.name, 'map: ahead %d, left %d, right %d -> turn %s' % (
                    s0, sl, sr, 'L' if d > 0 else 'R'))
                turn_by(d, 90)
                return
        drive_straight(BASE)


class WallFollow:
    """Keep the wall on the LEFT at ~15 cm WITHOUT leaving the grid: lean at most 10 deg.
    When the wall ends, drive on until the body clears it and turn LEFT into the opening
    (left-hand rule). After 4 turns the same way (a full loop) turn right once to leave."""
    name = 'WALLFOLLOW'

    def __init__(self):
        self.following = False
        self.seen_since = 0
        self.lost_since = 0
        self.epoch = 0

    def active(self):
        if self.epoch != S.turn_epoch:             # we turned: the old wall is not beside us
            self.epoch = S.turn_epoch
            self.following = False
        if S.follow_off:
            if clock.time() < S.off_until:
                self.following = False
                return False
            S.follow_off = False                   # crossing done: wall following may restart
        if S.dist < WALL_SEE and not self.following:
            S.wrap_n = 0
            self.following = True
            self.seen_since = clock.time()
            self.lost_since = 0
        return self.following

    def act(self):
        now = clock.time()
        d = S.dist
        grid = snap90(Heading.target)              # the grid line we are on (lean < 45 deg)
        if d > WALL_LOST:                          # wall gone?
            if self.lost_since == 0:
                self.lost_since = now
            Heading.target = grid
            if now - self.lost_since < LOST_CONFIRM_MS:
                drive_straight(BASE)               # maybe an echo dropout: keep straight
                return
            followed = now - self.seen_since
            self.following = False
            if followed >= MIN_FOLLOW_MS:          # a real wall ended: doorway / corner
                if S.mode == 'random' and rand(1, 100) > OPENING_PCT:
                    log('E', self.name, 'wall ended after %d ms: random choice, go straight' % followed)
                    return
                if S.mode == 'map':
                    h = snap90(Heading.target)
                    if score(h - 90000) > score(h) + 1:
                        log('E', self.name, 'wall ended: map says straight is newer ground')
                        return
                log('E', self.name, 'wall ended after %d ms: go round it to the left' % followed)
                if advance(CORNER_ADVANCE_DEG):
                    turn_by(1, 90)
                    S.wrap_n = 1
                    S.wrap_x, S.wrap_y = S.x, S.y
            return
        self.lost_since = 0
        if S.mode == 'wall' and S.tour_t:
            back = (S.x - S.tour_x) ** 2 + (S.y - S.tour_y) ** 2 < TOUR_BACK_MM * TOUR_BACK_MM
            if (back and S.odo_mm > TOUR_MIN_MM) or now - S.tour_t > TOUR_MAX_MS:
                log('E', 'TOUR', 'wall tour done (%d mm): explore the middle by map' % S.odo_mm)
                S.mode = 'map'
                S.turn_sum = 0
        if S.mode == 'wall' and S.turn_sum >= LOOP_TURNS:   # circling a free-standing block
            log('E', 'LOOP', 'circling an island: leave it to the right')
            S.turn_sum = 0
            S.follow_off = True
            S.off_until = now + CROSS_MS
            self.following = False
            turn_by(-1, 90)
            return
        if S.mode != 'wall' and abs(S.turn_sum) >= LOOP_TURNS and now - self.seen_since > LOOP_BREAK_MS:
            log('E', 'LOOP', 'turned 360 deg the same way: leave the loop to the right')
            S.turn_sum = 0
            S.follow_off = True                    # cross the room
            S.off_until = now + CROSS_MS
            self.following = False
            turn_by(-1, 90)
            return
        if S.mode == 'map' and S.new_cells >= STEER_EVERY:
            S.new_cells = 0
            h = snap90(Heading.target)
            if score(h + 90000) + 2 <= score(h) - STEER_MARGIN:
                log('E', self.name, 'map: right side is newer ground: leave the wall')
                S.turn_sum = 0
                S.follow_off = True
                S.off_until = now + CROSS_MS
                self.following = False
                turn_by(-1, 90)
                return
        if S.mode != 'wall' and now - self.seen_since > DETACH_MS:   # followed long enough: leave
            log('E', self.name, 'detach: leave the wall (turn right), cross the room')
            S.turn_sum = 0
            S.follow_off = True
            S.off_until = now + CROSS_MS
            self.following = False
            turn_by(-1, 90)
            return
        lean = (WALL_TARGET - d) * LEAN_MDEG_PER_MM    # too close -> lean right (+)
        if d < CLOSE_MM:                               # under 6 cm: full lean away
            lean = LEAN_MAX_DEG * 1000
        lean = max(-LEAN_MAX_DEG * 1000, min(LEAN_MAX_DEG * 1000, lean))
        Heading.target = grid + lean
        drive_straight(BASE)


class Escape:
    """Something was hit: back up 6 cm, then turn by the left-hand rule:
    left open -> LEFT 90, left blocked -> RIGHT 90, hit again right away -> turn around 180."""
    name = 'ESCAPE'
    IDLE, BACK = 0, 1

    def __init__(self):
        self.phase = Escape.IDLE
        self.t0 = 0
        self.last_end = -100000
        self.streak = 0
        self.dir = -1
        self.deg = 90

    def active(self):
        return self.phase != Escape.IDLE or S.hit

    def act(self):
        now = clock.time()
        if self.phase == Escape.IDLE:
            trav = (left.angle() + right.angle()) // 2   # driven since the last turn
            bumper = S.hit_bump
            S.hit_bump = False
            S.hit = False
            S.follow_off = False                   # a bump ends a loop-break crossing
            stop_drive()
            reset_wheels()
            if bumper:
                mark_bump()
            self.t0 = now
            self.phase = Escape.BACK
            if bumper and now - self.last_end < REPEAT_MS and trav < REPEAT_DEG:
                self.streak += 1                   # front blocked again right after a turn
            else:
                self.streak = 1
            left_open = S.dist >= LEFT_OPEN_MM
            if S.mode == 'wall':                   # front wall: turn right, wall ends up on the left
                self.dir, self.deg = -1, 90
                S.wrap_n = 0
                if S.tour_t == 0:
                    S.tour_t = now
                    S.tour_x, S.tour_y = S.x, S.y
                    S.odo_mm = 0
            elif self.streak == 2:                 # hit again right after a turn: boxed in
                self.dir = 1 if left_open else -1
                self.deg = 180
            elif self.streak >= 3:                 # still boxed in: try the right side
                self.dir = -1
                self.deg = 90
                self.streak = 0
            elif S.mode == 'map':                 # least visited of left / right / back
                h = snap90(Heading.target)
                opts = [(-1, 90, score(h + 90000) + rand(0, 1))]
                if left_open:
                    opts.append((1, 90, score(h - 90000) + rand(0, 1)))
                opts.append((1 if left_open else -1, 180, score(h + 180000) + 2 + rand(0, 1)))
                best = min(opts, key=lambda o: o[2])
                self.dir, self.deg = best[0], best[1]
            elif S.mode == 'random':              # random 90/180 on the grid, never left into a wall
                self.dir = (1 if rand(0, 1) else -1) if left_open else -1
                self.deg = 180 if rand(1, 100) <= TURN180_PCT else 90
            else:
                self.dir = 1 if left_open else -1
                self.deg = 90
            log('E', self.name, 'hit: back up, turn %s %d (left sonar %d mm, hit #%d)' % (
                'L' if self.dir > 0 else 'R', self.deg, S.dist, self.streak))
        if self.phase == Escape.BACK:
            drive(-BACK_PWR, -BACK_PWR)
            back = -(left.angle() + right.angle()) // 2
            if back >= BACKUP_DEG or now - self.t0 > 1500:
                turn_by(self.dir, self.deg)        # always on the 90-degree grid
                self.phase = Escape.IDLE
                self.last_end = clock.time()
                S.hit = False                      # ignore presses during the turn
                S.hit_bump = False
                S.wedge_ticks = 0
                S.side_ticks = 0


class GoalFind:
    """Dark pad for several ticks while driving level and on course -> stop and re-check
    (identification). Confirmed -> alarm starts."""
    name = 'GOALFIND'

    def __init__(self):
        self.hits = 0

    def active(self):
        if S.pad_seen:
            return True
        if not S.detect_ok or S.bright >= S.thresh:
            self.hits = 0
        else:
            self.hits += 1
        return self.hits >= FIRE_CONFIRM

    def act(self):
        stop_drive()
        avg = average_reflection(8)
        log('E', self.name, 'recheck avg=%d thresh=%d' % (avg, S.thresh))
        S.pad_seen = False
        S.dark_n = 0
        if avg < S.thresh:
            S.fire = True
            Heading.target = Heading.yaw           # sweep around where it really points
            ev3.light.on(Color.RED)
            ev3.screen.clear()
            ev3.screen.print('FIRE!')
            print('FIRE found: reading', S.bright, '< threshold', S.thresh)
            ev3.speaker.beep(1000, 300)
        else:
            self.hits = 0
            log('E', self.name, 'false alarm rejected')


class Tipped:
    """No floor under the color sensor for 3 s and no fire: the robot fell over or was
    lifted. Stop, beep, wait until it stands on the floor again, then restart the gyro
    grid from the direction it now faces."""
    name = 'TIPPED'

    def active(self):
        return (not S.fire and S.dark_since != 0 and
                clock.time() - S.dark_since > TIP_MS)

    def act(self):
        stop_drive()
        log('E', self.name, 'no floor for 3 s: fell over / lifted - waiting to be set up')
        ev3.light.on(Color.YELLOW)
        ok_since = 0
        while True:
            b = color.reflection()
            now = clock.time()
            if b >= S.thresh + 2:
                if ok_since == 0:
                    ok_since = now
                elif now - ok_since > UPRIGHT_MS:
                    break
            else:
                ok_since = 0
                ev3.speaker.beep(400, 100)
            wait(300)
        ev3.light.off()
        gyro_zero()
        Heading.yaw = 0                            # new start: grid lined up with how the
        Heading.target = 0                         # robot was set down
        Heading.iacc = 0
        S.dark_since = 0
        S.turn_sum = 0
        S.follow_off = False
        S.hit = False
        log('E', self.name, 'standing again: continue')


class Extinguish:
    """Fire identified (first dark reading on the pad): back up a little so the fan is not
    on top of the candle, fan on for FAN_MS while sweeping SWEEP_DEG left and right of the
    fire direction, alarm beeping, then finish."""
    name = 'EXTINGUISH'

    def active(self):
        return S.fire

    def act(self):
        stop_drive()
        log('E', self.name, 'black pad: back up %d mm' % FIRE_BACK_MM)
        back_up(mm_to_deg(FIRE_BACK_MM))
        stop_drive()
        base = Heading.target
        fan.dc(100)
        log('E', self.name, 'fan on, sweep +-%d deg' % SWEEP_DEG)
        t = clock.time()
        side = 1
        while clock.time() - t < FAN_MS:
            turn_to(base + side * SWEEP_DEG * 1000)
            log('E', self.name, 'fan %d deg/s' % fan.speed())   # proves the fan really turns
            side = -side
            ev3.speaker.beep(1000, 200)
        turn_to(base)
        fan.stop()
        log('E', self.name, 'fan off, mission complete')
        ev3.light.on(Color.GREEN)
        ev3.speaker.beep(1500, 600)
        return True


def show_light(name):
    """Status light: red = fire, green = wander, orange = wall following.
    Bumps and escapes do not change the light."""
    if name in ('GOALFIND', 'EXTINGUISH'):
        ev3.light.on(Color.RED)
    elif name == 'WANDER':
        ev3.light.on(Color.GREEN)
    elif name == 'WALLFOLLOW':
        ev3.light.on(Color.ORANGE)


# ======================= subsumption arbitration =======================
def main():
    log_open()
    ev3.screen.clear()
    ev3.screen.print('Set robot down')
    wait(START_DELAY_MS)
    S.floor = average_reflection(30)
    S.thresh = PAD_MAX
    S.detect_ok = S.floor >= MIN_FLOOR
    print('floor =', S.floor, ' pad threshold =', S.thresh)
    log('E', 'START', 'floor=%d thresh=%d BASE=%d WALL_TARGET=%d LEFT_OPEN=%d mode=%s battery=%dmV' % (
        S.floor, S.thresh, BASE, WALL_TARGET, LEFT_OPEN_MM, EXPLORE, ev3.battery.voltage()))
    print('gyro:', 'connected' if gyro is not None else 'NOT found - turns use encoders')
    if not S.detect_ok:
        print('FIRE DETECTION OFF: floor reads', S.floor)
        log('E', 'START', 'detection disabled: floor too dark')
        ev3.speaker.beep(200, 800)
    gyro_startup()
    Rng.seed = (Rng.seed + clock.time() * 7919) & 0x7FFFFFFF   # different path each run
    ev3.speaker.beep(600, 100)

    behaviors = [Extinguish(), Tipped(), GoalFind(), Escape(), WallFollow(), Wander()]
    escape = behaviors[3]
    last = ''
    gc.collect()
    try:
        while True:
            sense()
            S.busy = escape.phase != Escape.IDLE
            for b in behaviors:
                if b.active():
                    if b.name != last:
                        last = b.name
                        print(last)
                        show_light(last)
                        log('E', last, 'behavior start')
                    if b.act():
                        return
                    break
            now = clock.time()
            if now >= Log.next:
                Log.next = now + LOG_EVERY_MS
                log('S', last, 'yaw=%d tgt=%d turns=%d x=%d y=%d' % (
                    Heading.yaw // 1000, Heading.target // 1000, S.turn_sum, int(S.x), int(S.y)))
                ev3.screen.clear()
                ev3.screen.print(last)
                ev3.screen.print('wall %d mm' % S.dist)
                ev3.screen.print('floor %d' % S.bright)
            wait(TICK_MS)
    except Exception as e:
        log('ERR', last, repr(e))
        try:
            sys.print_exception(e, Log.f)
        except Exception:
            pass
        raise
    finally:
        stop_drive()
        fan.stop()
        log('E', last, 'END')
        log_close()


main()
