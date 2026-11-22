#!/usr/bin/env pybricks-micropython
"""
Behavior-based Fire Alarm Robot - CSE 4360 Project 2 (Pybricks EV3 MicroPython)

Only basic motor / sensor functions are used (no robotics / DriveBase module).
No map, no localization: purely reactive behaviors with a little local state.

BEHAVIORS (each has an ACTIVATION condition and a CONTROL output)
    Wander      always          -> drive STRAIGHT until the front bumper hits something
    WallFollow  wall <30 cm left-> gentle PD on the left sonar (hold ~12 cm). When the wall
                                   ENDS (doorway / corner): advance, then turn LEFT 90 deg
    Depart      CROSS starts    -> turn 90 deg away from the wall, then cross the room
    Escape      bumper pressed  -> back up ~5 cm, turn EXACTLY 90 deg by the gyro. Direction from
                                   the left sonar + a tiny local memory of blocked directions:
                                   left (if open) > right > back; never two lefts in a row;
                                   a dead end (everything blocked) = turn around 180 deg
    GoalFind    pad is dark     -> stop, re-check to IDENTIFY the fire pad, start alarm
    Extinguish  fire found      -> creep onto the pad, run the fan under the alarm, finish

EXPLORATION (no map): a random timer alternates FOLLOW (15-30 s, wall following) and
CROSS (6-12 s, wall following inhibited: straight line across the room) so the robot does
not circle the same walls forever; 3 quick bumps in a row also force a CROSS.

TURNS use the gyro rate (zero offset measured while stopped, integrated), never the wheels:
the wheels slip on the plastic sheet. A turn that gets stuck backs up and retries.

COORDINATION: SUBSUMPTION (strict priority, highest first)
    Extinguish > GoalFind > Escape > Depart > WallFollow > Wander
Every tick all sensors are read once, then the highest-priority behavior whose
activation condition holds drives the motors. Escape is a small state machine that
never blocks, so a higher-priority behavior (goal finding) can interrupt it at any tick.

Ports (edit to match the robot):
    Motors : A = left wheel, B = right wheel, D = fan (medium)
    Sensors: 4 = color (down), 2 = ultrasonic (LEFT side), 1 = touch (front bumper)

MicroPython memory note: the loop works on small ints only (no floats, no lists, no
string building) so it allocates almost nothing; gc.collect() runs once before it.
"""
import gc
import sys

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch

# ======================= configuration =======================
LEFT_PORT, RIGHT_PORT, FAN_PORT = Port.A, Port.B, Port.D
COLOR_PORT, SONAR_PORT, TOUCH_PORT = Port.S4, Port.S2, Port.S1
GYRO_PORT = Port.S3                     # gyro measures the turn angle (falls back to encoders)
LEFT_DIR = Direction.CLOCKWISE          # flip if a wheel runs backward
RIGHT_DIR = Direction.CLOCKWISE
FAN_DIR = Direction.CLOCKWISE           # flip if the fan blows backward

# --- robot geometry (measured, cm from the AXLE CENTER = midway between the drive wheels) ---
WHEEL_DIA_MM = 54        # 2.125 in wheels
TRACK_MM = 170           # wheel to wheel
FRONT_CM, REAR_CM, HALF_WIDTH_CM = 11, 17, 10
# turning in place, the corners swing out to: front sqrt(11^2+10^2) = 14.9 cm,
# rear sqrt(17^2+10^2) = 19.7 cm  -> a clean in-place turn needs ~40 cm of free space.


def mm_to_deg(mm):
    """Wheel degrees for a straight distance in mm (pi ~ 3.14)."""
    return mm * 36000 // (314 * WHEEL_DIA_MM)


# --- driving (percent motor power; real motors differ, trim the stronger one down) ---
BASE = 35                # cruising power
LEFT_GAIN = 90           # e.g. 95 if the robot curves left
RIGHT_GAIN = 100
SLEW = 10                # max power change per tick: gentle starts (slippery floor)

# --- wall following (ultrasonic on the LEFT side, distances in mm) ---
WALL_TARGET = 120        # distance to hold
WALL_SEE = 300           # closer than this = a wall is beside us
AVOID_MM = 150           # CROSS phase: closer than this on the LEFT -> steer away (right)
TURN_AWAY_MM = 250       # something this close on the LEFT: never turn left toward it
WALL_LOST = 380          # farther than this while following = wall ended (corner/door)
LOST_CONFIRM_MS = 500    # wall must be gone this long before we call it an ending
MIN_FOLLOW_MS = 1500     # only a wall followed this long counts (not a glimpse)
CORNER_ADVANCE_DEG = mm_to_deg(120)  # ~12 cm: drive on so the body clears the wall end
SONAR_CAP = 1000         # clip far/no-echo readings (sensor returns ~2550 for no echo)
KP_PCT = 8               # steer = (KP_PCT*error + KD_PCT*change_in_error) / 100
KD_PCT = 20              # damping against wobble
MAX_STEER = 8             # gentle toward the wall: the robot should stay nearly straight
MAX_STEER_AWAY = 16       # but twice as hard AWAY from the wall when too close

# --- escape: back up, then turn RIGHT in place (wall ends up on the left) ---
# 75 mm wheels: 236 mm per 360 deg. Track ~150 mm: wheel_deg = robot_deg * 150 / 75.
BACKUP_DEG = mm_to_deg(60)   # 6 cm: bumper is 11 cm from the axle, front corners swing
                             # out to 14.9 cm -> back up 4 cm + 2 cm margin before turning
TURN_PER_DEG_X100 = 440  # wheel deg per robot deg x100, encoder FALLBACK only: track/wheel =
                         # 170/54 = 3.15, x1.4 measured slip on the sheet. Gyro turns ignore it.
                         # normally use the gyro). Measured: 198 wheel deg = ~70 deg (wheels slip).
BACK_PWR = 30
MIN_DUTY = 22            # a wheel below this power stalls on the sheet: never command less
TURN_PWR = 35            # turn power (gentle: the plastic sheet slips and folds)
TURN_SLOW_PWR = 22       # last part of a turn / trim: slow so it does not coast past
SLOW_ZONE_DEG = 30       # go slow for the last this-many degrees of a turn
TRIM_TOL_DEG = 1.5       # after a turn, correct if the gyro is off by more than this
TRIM_MAX_MS = 1200
STUCK_MDEG = 4000        # turning less than this many milli-deg per CHECK_MS = stuck on the sheet
CHECK_MS = 200
BOOST_STEP = 10          # add this much power each time the robot is stuck...
MAX_BOOST = 30           # ...up to this much extra. Kept low: pushing a TALL robot hard
                         # against a wall corner tips it over
TURN_RETRIES = 1          # stuck turn: back up and try again this many times
STUCK_TOL_DEG = 15       # turned less than target by this much = stuck
RECOVER_DEG = mm_to_deg(60)  # back up 6 cm before retrying a stuck turn
ROCK_MS = 250            # still stuck at max power: spin the other way this long to free it
HOLD_KP_X10 = 15         # straight driving: 1.5 power per degree off the target heading
HOLD_MAX = 10            # max power difference used to hold the heading
HOLD_KI_X1000 = 30       # slowly removes a constant leftover offset (e.g. one motor stronger)
REALIGN_DEG = 4          # straight driving pushed more than this off the target heading...
REALIGN_TICKS = 6        # ...for this many ticks (~0.2 s) = a corner is dragging us: stop,
REALIGN_BACK_DEG = mm_to_deg(40)  # back up 4 cm, then turn back in place
GOAL_ERR_DEG = 2         # ignore the color sensor while this far off course (robot tipping)
AVOID_COOLDOWN_MS = 1500  # at most one small avoid turn per this many ms
SIGN_CHECK_DEG = 10      # startup: small right turn to learn which way the gyro counts
GYRO_LEAD = 1            # stop the turn this many degrees early: the robot coasts a little
BACK_MAX_MS = 1500       # safety timeouts (stalled wheels never lock the program)
TURN_MAX_MS = 3500
REPEAT_MS = 3000         # bumped again this soon after an escape = stuck: turn 50% more
NEAR_DEG = mm_to_deg(300)  # driven farther than 30 cm = a new place, forget blocks
WEDGE_MM = 45            # sonar this close...
WEDGE_TICKS = 55         # ...for this many ticks (~2 s) = wedged against the wall: escape
STUCK_BUMPS = 3         # this many quick bumps in a row -> force a CROSS phase

# --- exploration: alternate FOLLOW (left wall) and CROSS (leave the wall, new area) ---
FOLLOW_MIN_MS = 15000    # time spent following walls before breaking away
FOLLOW_MAX_MS = 30000
CROSS_MIN_MS = 6000      # time spent crossing open floor toward a different wall
CROSS_MAX_MS = 12000

# --- fire pad (BLACK pad on a lighter floor), reflection 0-100 ---
PAD_MAX = 2              # pad = reading BELOW this. Measured: pad 0-1, floor 13-25, wall shadows 2-5.
                         # If the color sensor height changes, re-measure and set this value.
MIN_FLOOR = 8            # floor must read at least this at startup or detection is off
FIRE_CONFIRM = 8        # consecutive dark ticks before GoalFind looks closer
CREEP_DEG = mm_to_deg(100)  # roll ~10 cm forward so the fan is over the candle (tune)
FAN_MS = 10000

START_DELAY_MS = 3000    # time to set the robot down after pressing run
TICK_MS = 30

# --- run log: CSV file on the brick, download it afterwards and share it ---
LOG_FILE = 'log.csv'     # written in the project folder, overwritten each run
LOG_EVERY_MS = 200       # sensor/motor sample period
LOG_FLUSH_EVERY = 5      # flush to the SD card every N lines
LOG_MAX_LINES = 5000     # safety cap so the card never fills


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
    gyro = None                                    # not connected: turns use the encoders
clock = StopWatch()


# ======================= sensing =======================
class Sense:
    dist = SONAR_CAP     # mm to the wall on the left (smoothed)
    bump = False         # front bumper
    bright = 0           # floor reflection 0-100
    floor = 50           # floor reflection, measured at start and slowly adapted
    thresh = 20          # pad threshold derived from floor
    fire = False         # set by GoalFind once the pad is identified
    expect_wall = False  # set by Escape: a wall is now on our left
    cross = False        # True = CROSS phase (wall following inhibited), False = FOLLOW
    mode_until = 0       # time (ms) when the current phase ends
    depart = False       # a leave-the-wall turn is pending (start of a CROSS phase)
    busy = False         # Escape/Depart is maneuvering (color sensor not trusted)
    hit = False          # a NEW bumper press / wedge happened (consumed by Escape)
    wedge_ticks = 0
    drag_ticks = 0       # ticks the heading has been pushed off target while driving
    avoid_until = 0      # cooldown for the small avoid turn
    last_dir = 0         # previous turn decision: +1 left, -1 right, 0 none yet
    heading = 0          # facing direction in 90-deg steps: 0,1,2,3 (right turn = +1)
    blocked = 0          # bit mask: directions known to be blocked at this place
    detect_ok = True     # False when the floor reading has too little contrast vs the pad


S = Sense()


def sense():
    """Read every sensor once per tick (cheap, keeps behaviors consistent)."""
    d = sonar.distance()
    if d > SONAR_CAP:
        d = SONAR_CAP
    S.dist = (S.dist + d) // 2                 # light smoothing against echo noise
    pressed = touch.pressed()
    if pressed and not S.bump:                     # NEW press only (rising edge): a bumper
        S.hit = True                               # that stays pressed must not retrigger
    S.bump = pressed
    b = color.reflection()
    S.bright = b
    yaw_update()
    # sonar wedged against / blocked by something very close for a long time -> treat as a bump
    if S.dist < WEDGE_MM:
        S.wedge_ticks += 1
        if S.wedge_ticks >= WEDGE_TICKS:
            S.wedge_ticks = 0
            S.hit = True
            log('E', 'WEDGE', 'sonar < %d mm for a long time: treat as a bump' % WEDGE_MM)
    else:
        S.wedge_ticks = 0


def average_reflection(n):
    total = 0
    for _ in range(n):
        total += color.reflection()
        wait(20)
    return total // n


# ======================= actuation =======================
class Cmd:
    l = 0
    r = 0


C = Cmd()


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
    """Open-loop power, slew-limited so the wheels don't spin or jerk on a slick floor."""
    C.l = ramp(C.l, min_duty(l * LEFT_GAIN // 100))
    C.r = ramp(C.r, min_duty(r * RIGHT_GAIN // 100))
    left.dc(C.l)
    right.dc(C.r)


def stop_drive():
    C.l = 0
    C.r = 0
    left.stop(Stop.BRAKE)
    right.stop(Stop.BRAKE)


class Heading:
    """ONE absolute heading for the whole run, integrated from the gyro RATE.
    yaw    : where the robot points now, milli-degrees, RIGHT turn = positive
    target : where it SHOULD point. Straight driving steers back to it; a 90 deg turn moves
             it by exactly 90 deg from the previous TARGET (not from wherever the robot
             happens to point), so every turn also corrects the error of the one before."""
    yaw = 0
    target = 0
    bias = 0        # gyro zero-rate offset, milli-deg/s, re-measured whenever stopped
    last = 0        # time of the last integration step (ms)
    gsign = 1       # +1 if the gyro counts positive for a RIGHT turn (found at startup)
    iacc = 0        # integral of the heading error while driving straight (degree-ticks)


def yaw_update():
    """Integrate the gyro rate (zero offset removed). deg/s * ms = milli-degrees."""
    if gyro is None:
        return
    now = clock.time()
    Heading.yaw += Heading.gsign * (gyro.speed() * 1000 - Heading.bias) * (now - Heading.last) // 1000
    Heading.last = now


def gyro_zero():
    """Robot is STOPPED: re-measure the gyro zero offset (the gyro drifts). ~0.25 s."""
    if gyro is None:
        return
    yaw_update()
    wait(150)                                      # let the brake settle the robot
    total = 0
    for _ in range(10):
        total += gyro.speed()
        wait(10)
    Heading.bias = total * 100                     # mean of 10 samples, milli-deg/s
    Heading.last = clock.time()                    # robot was still: yaw unchanged


def drive_straight(base):
    """Drive forward HOLDING the target heading: any drift (motor difference, slip, a fold
    in the sheet) is measured by the gyro and steered back out."""
    yaw_update()
    if gyro is None:
        drive(base, base)
        return
    err = (Heading.target - Heading.yaw) // 1000   # degrees, + = target is to the right
    lim = HOLD_MAX * 1000 // HOLD_KI_X1000
    Heading.iacc = max(-lim, min(lim, Heading.iacc + err))
    steer = err * HOLD_KP_X10 // 10 + Heading.iacc * HOLD_KI_X1000 // 1000
    if steer > HOLD_MAX:
        steer = HOLD_MAX
    elif steer < -HOLD_MAX:
        steer = -HOLD_MAX
    drive(base + steer, base - steer)              # right of us: left wheel faster


def spin(d, pwr):
    """Turn in place: d=+1 left, d=-1 right."""
    drive(-pwr * d, pwr * d)


def turn_to(goal):
    """Rotate in place until the gyro heading reaches `goal` (milli-deg). Fast, then slow
    near the goal, settle, trim. Overshoot is corrected automatically (the direction comes
    from the sign of the error). If the gyro shows the robot is not rotating (caught in a
    fold of the sheet) add power step by step, rock back if needed. Returns the final
    error (milli-deg)."""
    stop_drive()
    left.reset_angle(0)
    right.reset_angle(0)
    gyro_zero()
    Heading.iacc = 0
    t0 = clock.time()
    boost = 0
    peak = 0
    rocks = 0
    chk_t = t0
    chk_e = abs(goal - Heading.yaw)
    while clock.time() - t0 < TURN_MAX_MS:
        yaw_update()
        err = goal - Heading.yaw
        if abs(err) <= GYRO_LEAD * 1000:
            break
        d = -1 if err > 0 else 1                   # goal to the right -> turn right
        now = clock.time()
        if now - chk_t >= CHECK_MS:                # is the robot actually rotating?
            if chk_e - abs(err) < STUCK_MDEG:
                if boost >= MAX_BOOST:             # max power and still stuck: rock it free
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
            chk_e = abs(goal - Heading.yaw)
        spin(d, (TURN_PWR if abs(err) > SLOW_ZONE_DEG * 1000 else TURN_SLOW_PWR) + boost)
        wait(10)
    stop_drive()
    t1 = clock.time()
    while clock.time() - t1 < 250:                 # settle: keep integrating the coast
        yaw_update()
        wait(10)
    t2 = clock.time()
    trims = 0
    boost = 0
    chk_t = t2
    chk_e = abs(goal - Heading.yaw)
    while clock.time() - t2 < TRIM_MAX_MS:
        yaw_update()
        err = goal - Heading.yaw
        if abs(err) <= TRIM_TOL_DEG * 1000:
            break
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
    t3 = clock.time()
    while clock.time() - t3 < 150:
        yaw_update()
        wait(10)
    err = goal - Heading.yaw
    log('E', 'TURN', 'to %d deg: now %d, error %d, trims %d boost %d rocks %d%s' % (
        goal // 1000, Heading.yaw // 1000, err // 1000, trims, peak, rocks,
        ' STUCK' if abs(err) > STUCK_TOL_DEG * 1000 else ''))
    return err


def enc_turn(d, robot_deg):
    """Fallback without a gyro: turn by wheel encoders (inaccurate on the sheet)."""
    stop_drive()
    left.reset_angle(0)
    right.reset_angle(0)
    target = robot_deg * TURN_PER_DEG_X100 // 100
    t0 = clock.time()
    while clock.time() - t0 < TURN_MAX_MS and (right.angle() - left.angle()) * d // 2 < target:
        spin(d, TURN_PWR)
        wait(10)
    stop_drive()
    log('E', 'TURN', 'enc turn %s %d deg' % ('L' if d > 0 else 'R', robot_deg))


def turn_by(d, robot_deg):
    """Turn 90 (or 180) deg: d=+1 left, -1 right. The new target is the OLD TARGET plus the
    turn, so the error left over from earlier turns and from driving is corrected now.
    If the sheet grabs the robot (stuck), back up to a fresh spot and finish the turn."""
    if gyro is None:
        enc_turn(d, robot_deg)
        return
    Heading.target += -d * robot_deg * 1000        # right turn = + (see Heading)
    err = turn_to(Heading.target)
    tries = 0
    while tries < TURN_RETRIES and abs(err) > STUCK_TOL_DEG * 1000:
        tries += 1
        log('E', 'TURN', 'stuck %d deg short: back up and retry' % (abs(err) // 1000))
        back_up(RECOVER_DEG)
        err = turn_to(Heading.target)


def back_up(deg):
    """Reverse straight a fixed distance (wheel degrees), keeping the heading integrated."""
    left.reset_angle(0)
    right.reset_angle(0)
    t = clock.time()
    while clock.time() - t < 1500 and -(left.angle() + right.angle()) // 2 < deg:
        yaw_update()
        drive(-BACK_PWR, -BACK_PWR)
        wait(10)
    stop_drive()


def gyro_startup():
    """Robot is standing still after the start delay: measure the gyro zero, then make a
    small right turn to learn which way the gyro counts (it depends on how it is mounted),
    and turn back to the start heading."""
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
        spin(-1, TURN_PWR)                         # turn RIGHT
        wait(10)
    stop_drive()
    if abs(raw) < 3000:
        log('E', 'GYRO', 'sign check: gyro saw almost no rotation (%d mdeg) - check it' % raw)
        print('GYRO: no rotation seen in the sign check - check mount/cable')
    Heading.gsign = 1 if raw >= 0 else -1
    Heading.yaw = abs(raw)                         # we turned RIGHT = positive
    Heading.last = clock.time()
    Heading.target = 0
    log('E', 'GYRO', 'sign %d, bias %d mdeg/s; turning back to the start heading' % (
        Heading.gsign, Heading.bias))
    turn_to(0)


# ======================= run log =======================
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
        Log.f = None                               # no log, the robot still runs


def log(tag, beh, note=''):
    """tag: S = periodic sample, E = event, ERR = exception. One CSV line."""
    f = Log.f
    if f is None or Log.n >= LOG_MAX_LINES:
        return
    f.write('%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n' % (
        clock.time(), tag, beh, S.dist, int(S.bump), S.bright, S.floor,
        S.thresh, C.l, C.r, left.angle(), right.angle(), note))
    Log.n += 1
    if tag != 'S' or Log.n % LOG_FLUSH_EVERY == 0:
        f.flush()                                  # survives the back button


def log_close():
    if Log.f is not None:
        try:
            Log.f.close()
        except OSError:
            pass
        Log.f = None


# ======================= behaviors =======================
class Rng:
    seed = 12345


def rand(lo, hi):
    """Small integer LCG (no floats, no allocation). Randomness breaks repeating paths."""
    Rng.seed = (Rng.seed * 1103515245 + 12345) & 0x7FFFFFFF
    return lo + (Rng.seed >> 8) % (hi - lo + 1)


def supervise():
    """Switch between FOLLOW and CROSS phases on a random timer (called every tick).
    Without this, wall following circles the same loop of walls forever."""
    now = clock.time()
    if now < S.mode_until:
        return
    if S.cross:
        S.cross = False
        S.mode_until = now + rand(FOLLOW_MIN_MS, FOLLOW_MAX_MS)
        log('E', 'MODE', 'FOLLOW for %d ms' % (S.mode_until - now))
    else:
        S.cross = True
        S.depart = True
        S.mode_until = now + rand(CROSS_MIN_MS, CROSS_MAX_MS)
        log('E', 'MODE', 'CROSS for %d ms' % (S.mode_until - now))


def choose_turn(front, trav):
    """Decide the next turn from a tiny LOCAL memory (no map): the facing direction in
    90-deg steps and which directions are known to be blocked at this place.
      front : True after a bump (the wall in front is blocked)
      trav  : wheel deg driven since the last bump; a long way = a new place, forget blocks
    Left is known blocked when the LEFT sonar sees a wall. Order of preference:
      normally  LEFT (the one side we can see is free) > RIGHT > BACK
      after a left turn  RIGHT > LEFT > BACK   (two lefts would be a U-turn back where we
                                                 came from)
    Directions known blocked are skipped; everything blocked (dead end) = turn around.
    Returns (d, degrees): d = +1 left, -1 right; BACK is a 180 deg turn to the right."""
    h = S.heading
    if not front or trav >= NEAR_DEG:
        S.blocked = 0
    if front:
        S.blocked |= 1 << h
    lt = (h + 3) % 4
    rt = (h + 1) % 4
    bk = (h + 2) % 4
    if S.dist < TURN_AWAY_MM:
        S.blocked |= 1 << lt
    if S.last_dir == 1:
        first, second = rt, lt
    else:
        first, second = lt, rt
    pick = bk
    if not (S.blocked >> first) & 1:
        pick = first
    elif not (S.blocked >> second) & 1:
        pick = second
    if pick == lt:
        d, deg = 1, 90
    elif pick == rt:
        d, deg = -1, 90
    else:
        d, deg = -1, 180
    S.last_dir = d
    return d, deg


def turned(d, deg):
    """Update the facing direction after a turn of deg degrees (d: +1 left, -1 right)."""
    S.heading = (S.heading - d * ((deg + 45) // 90)) % 4


def advance(deg):
    """Drive straight a fixed distance (wheel degrees). Returns False if the bumper hit."""
    left.reset_angle(0)
    right.reset_angle(0)
    t = clock.time()
    while clock.time() - t < 2500 and (left.angle() + right.angle()) // 2 < deg:
        if touch.pressed():
            stop_drive()
            S.hit = True
            return False
        drive_straight(BASE)
        wait(10)
    stop_drive()
    return True


class Wander:
    """Free space / CROSS phase: drive STRAIGHT until the bumper hits something."""
    name = 'WANDER'

    def active(self):
        return True                                # lowest level: always available

    def act(self):
        now = clock.time()
        if gyro is not None:
            if abs(Heading.target - Heading.yaw) > REALIGN_DEG * 1000:
                S.drag_ticks += 1
            else:
                S.drag_ticks = 0
            if S.drag_ticks >= REALIGN_TICKS:      # something (a corner) is pushing us off
                S.drag_ticks = 0                   # course: stop, back off, turn back, go on
                log('E', 'REALIGN', 'pushed %d deg off course: back up, turn back' % (
                    (Heading.yaw - Heading.target) // 1000))
                stop_drive()
                back_up(REALIGN_BACK_DEG)
                turn_to(Heading.target)
                return
        if S.dist < AVOID_MM and now > S.avoid_until:   # scraping the left wall:
            S.avoid_until = now + AVOID_COOLDOWN_MS        # turn 10 deg right IN PLACE
            Heading.target = Heading.yaw + 10000
            turn_to(Heading.target)
            return
        drive_straight(BASE)


class WallFollow:
    """Follow a wall on the LEFT with gentle steering on the side sonar. When the wall
    ENDS (doorway / outside corner) drive on until the body clears it, then turn LEFT 90
    deg around it (left-hand rule): that is how the robot gets into the next room."""
    name = 'WALLFOLLOW'

    def __init__(self):
        self.following = False
        self.seen_since = 0
        self.lost_since = 0
        self.prev = 0

    def active(self):
        now = clock.time()
        if S.cross:                                # CROSS phase inhibits wall following
            self.following = False
            S.expect_wall = False
            return False
        if S.expect_wall:                          # Escape just turned the wall to our left
            S.expect_wall = False
            if not self.following:
                self.seen_since = now
            self.following = True
        if S.dist < WALL_SEE and not self.following:
            self.following = True
            self.seen_since = now
            self.lost_since = 0
        return self.following

    def act(self):
        d = S.dist
        now = clock.time()
        if d > WALL_LOST:
            if self.lost_since == 0:
                self.lost_since = now
            if now - self.lost_since < LOST_CONFIRM_MS:
                drive_straight(BASE)               # maybe just an echo dropout: keep straight
                return
            followed = now - self.seen_since
            self.following = False
            self.lost_since = 0
            self.prev = 0
            if followed >= MIN_FOLLOW_MS:          # we really were following a wall: it ended
                log('E', self.name, 'wall ended after %d ms: advance, turn left around it' % followed)
                if advance(CORNER_ADVANCE_DEG):
                    turn_by(1, 90)
                    turned(1, 90)
                    S.last_dir = 1                 # a left turn: the next one is right
                    S.blocked = 0
            return
        self.lost_since = 0
        err = d - WALL_TARGET                      # + = too far from the wall
        derr = err - self.prev
        self.prev = err
        steer = (KP_PCT * err + KD_PCT * derr) // 100
        if d < WALL_TARGET // 2:                   # much too close: push away harder
            steer = min(steer, -MAX_STEER)
        if steer > MAX_STEER:
            steer = MAX_STEER
        elif steer < -MAX_STEER_AWAY:
            steer = -MAX_STEER_AWAY
        # wall on the left: too far -> steer left (left wheel slower),
        # too close -> steer right (right wheel slower)
        drive(BASE - steer, BASE + steer)
        Heading.target = Heading.yaw               # the wall defines our heading now
        Heading.iacc = 0


class Escape:
    """Bumper hit: back up, then turn RIGHT a fixed angle (measured with the encoders).
    A state machine: each tick does a little and returns, so it can be interrupted."""
    name = 'ESCAPE'
    IDLE, BACK = 0, 1

    def __init__(self):
        self.phase = Escape.IDLE
        self.t0 = 0
        self.last_end = -100000
        self.robot_deg = 0
        self.dir = 1                               # +1 = turn left, -1 = turn right
        self.streak = 0

    def active(self):
        return self.phase != Escape.IDLE or S.hit

    def act(self):
        now = clock.time()
        if self.phase == Escape.IDLE:              # start
            trav = (left.angle() + right.angle()) // 2   # driven since the last turn
            S.hit = False
            S.wedge_ticks = 0
            stop_drive()
            left.reset_angle(0)
            right.reset_angle(0)
            self.t0 = now
            self.phase = Escape.BACK
            S.depart = False                       # this turn replaces a pending departure
            if now - self.last_end < REPEAT_MS:
                self.streak += 1                   # bumped again right away
            else:
                self.streak = 1
            self.dir, robot_deg = choose_turn(True, trav)   # sonar + local memory
            if self.streak >= STUCK_BUMPS:         # bumping again and again: leave this
                self.streak = 0                    # place, cross open floor for a while
                S.cross = True
                S.mode_until = now + rand(CROSS_MIN_MS, CROSS_MAX_MS)
            self.robot_deg = robot_deg
            log('E', self.name, 'bump: back up, turn %s %d deg cross=%d heading=%d blocked=%d trav=%d' % (
                'L' if self.dir > 0 else 'R', robot_deg, int(S.cross), S.heading, S.blocked, trav))
        if self.phase == Escape.BACK:
            drive(-BACK_PWR, -BACK_PWR)
            back = -(left.angle() + right.angle()) // 2
            if back >= BACKUP_DEG or now - self.t0 > BACK_MAX_MS:
                log('E', self.name, 'back up done: %d wheel deg in %d ms' % (back, now - self.t0))
                turn_by(self.dir, self.robot_deg)      # gyro-controlled turn
                turned(self.dir, self.robot_deg)
                self.phase = Escape.IDLE
                self.last_end = clock.time()
                S.hit = False                          # drop presses that happened meanwhile
                S.wedge_ticks = 0
                if not S.cross:
                    S.expect_wall = True


class Depart:
    """Start of a CROSS phase: turn 90 deg away from the wall (decided by choose_turn) and
    then drive straight across the room to a different wall."""
    name = 'DEPART'

    def active(self):
        return S.depart

    def act(self):
        S.depart = False
        d, deg = choose_turn(False, 0)
        log('E', self.name, 'leave wall: turn %s %d deg' % ('L' if d > 0 else 'R', deg))
        turn_by(d, deg)                            # gyro-controlled turn
        turned(d, deg)


class GoalFind:
    """Detect and IDENTIFY the fire: a dark pad for several ticks, then stop and
    re-sample to rule out a shadow or a tile seam. Confirmed -> start the alarm."""
    name = 'GOALFIND'

    def __init__(self):
        self.hits = 0

    def active(self):
        # ignore the color sensor while backing up / turning against a wall (shadows,
        # tilting bumper) and while the bumper is pressed: only trust it when driving
        off = abs(Heading.target - Heading.yaw) > GOAL_ERR_DEG * 1000
        if not S.detect_ok or S.busy or off or S.bright >= S.thresh:
            self.hits = 0
        else:
            self.hits += 1
        return self.hits >= FIRE_CONFIRM

    def act(self):
        stop_drive()
        avg = average_reflection(8)
        log('E', self.name, 'recheck avg=%d thresh=%d' % (avg, S.thresh))
        if avg < S.thresh:                         # still dark when standing still
            S.fire = True
            ev3.light.on(Color.RED)
            ev3.screen.clear()
            ev3.screen.print('FIRE!')
            print('FIRE found: reading', S.bright, '< threshold', S.thresh)
            ev3.speaker.beep(1000, 300)            # alarm starts
        else:
            self.hits = 0                          # false alarm, keep searching
            log('E', self.name, 'false alarm rejected')


class Extinguish:
    """Fire is found: creep onto the pad, run the fan under the alarm, then finish."""
    name = 'EXTINGUISH'

    def active(self):
        return S.fire

    def act(self):
        log('E', self.name, 'creep %d deg onto the pad' % CREEP_DEG)
        left.run_angle(150, CREEP_DEG, Stop.HOLD, False)
        right.run_angle(150, CREEP_DEG, Stop.HOLD, True)
        fan.dc(100)
        log('E', self.name, 'fan on')
        t = clock.time()
        while clock.time() - t < FAN_MS:           # alarm sounds while the fan runs
            ev3.speaker.beep(1000, 200)
            wait(300)
        fan.stop()
        stop_drive()
        log('E', self.name, 'fan off, mission complete')
        ev3.light.on(Color.GREEN)
        ev3.speaker.beep(1500, 600)
        return True                                # mission complete


# ======================= subsumption arbitration =======================
def main():
    log_open()
    ev3.screen.clear()
    ev3.screen.print('Set robot down')
    wait(START_DELAY_MS)
    S.floor = average_reflection(30)               # floor under the MOUNTED sensor
    S.thresh = PAD_MAX
    print('floor =', S.floor, ' pad threshold =', S.thresh)
    Rng.seed = (Rng.seed + clock.time()) & 0x7FFFFFFF   # different path every run
    S.cross = False                                # start by following the left wall
    S.mode_until = clock.time() + rand(FOLLOW_MIN_MS, FOLLOW_MAX_MS)
    log('E', 'START', 'floor=%d thresh=%d BASE=%d WALL_TARGET=%d BACKUP=%d TURN_X100=%d '
        'PAD_MAX=%d' % (S.floor, S.thresh, BASE, WALL_TARGET, BACKUP_DEG,
                       TURN_PER_DEG_X100, PAD_MAX))
    print('gyro:', 'connected' if gyro is not None else 'NOT found - turns use encoders')
    log('E', 'START', 'gyro=%s' % ('yes' if gyro is not None else 'no'))
    S.detect_ok = S.floor >= MIN_FLOOR             # enough floor-vs-pad contrast to decide
    if not S.detect_ok:
        print('FIRE DETECTION OFF: floor reads', S.floor, '(need >=', MIN_FLOOR, ')')
        print('Color sensor too high / tilted / shadowed: lower it to ~1 cm, point it down.')
        log('E', 'START', 'detection disabled: floor too dark for a reliable pad reading')
        ev3.speaker.beep(200, 800)
    gyro_startup()                                 # gyro zero + direction check
    ev3.speaker.beep(600, 100)

    # priority order: first active behavior wins
    behaviors = [Extinguish(), GoalFind(), Escape(), Depart(), WallFollow(), Wander()]
    last = ''
    gc.collect()
    try:
        while True:
            sense()
            supervise()
            S.busy = behaviors[2].phase != Escape.IDLE
            for b in behaviors:
                if b.active():
                    if b.name != last:
                        last = b.name
                        print(last)
                        ev3.screen.clear()
                        ev3.screen.print(last)
                        log('E', last, 'behavior start')
                    if b.act():
                        return
                    break
            now = clock.time()
            if now >= Log.next:                    # periodic sample for the log
                Log.next = now + LOG_EVERY_MS
                log('S', last, 'yaw=%d tgt=%d' % (Heading.yaw // 1000, Heading.target // 1000))
            wait(TICK_MS)
    except Exception as e:                         # record the crash, then re-raise
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
