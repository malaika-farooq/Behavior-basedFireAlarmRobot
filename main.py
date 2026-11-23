#!/usr/bin/env pybricks-micropython
"""
Behavior-based Fire Alarm Robot - CSE 4360 Project 2 (Pybricks EV3 MicroPython)

Only basic motor / sensor functions are used (no robotics / DriveBase module).
No map, no localization: reactive behaviors plus a gyro heading.

EXPLORATION = the LEFT-HAND RULE (classic maze exploration), on a 90-degree grid:
    left open  -> go left;  else front open -> go straight;  else right;  else turn back.
All straight paths are on the grid lined up with the start heading (gyro), so errors from
slip or a bad turn never carry into the next move.

BEHAVIORS (priority order = subsumption, highest first)
    Extinguish  fire identified   -> stay where it is, fan on, alarm, finish
    Tipped      no floor for 3 s  -> robot fell over / lifted: stop, beep, wait to be set up
    GoalFind    dark pad          -> stop, re-check (identify), start the alarm
    Escape      bumper / stuck    -> back up 6 cm, then a gyro turn:
                                       left open (sonar >= 8 cm) -> LEFT 90
                                       left blocked              -> RIGHT 90
                                       bumped again at once      -> turn around 180
    WallFollow  wall on the left  -> stay on the grid line, lean at most 10 deg to keep the
                                     wall at ~15 cm. Wall ENDS (doorway / corner) ->
                                     drive on until the body clears it, turn LEFT into it.
                                     After a full 360 loop (circling a block or a room) it
                                     turns right once to leave the loop.
    Wander      always            -> drive straight on the grid until something is hit

Ports: motors A = left wheel, B = right wheel, D = fan;
       sensors 1 = touch (bumper), 2 = ultrasonic (LEFT side), 3 = gyro, 4 = color (down)
"""
import gc
import sys

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
# front 11 cm, rear 17 cm, half width 10 cm. Turning in place the corners swing out to
# 14.9 cm (front) and 19.7 cm (rear): keep ~15 cm from the left wall so a RIGHT turn does
# not swing the rear corner into it, and back up 6 cm so the front corners clear a wall.


def mm_to_deg(mm):
    """Wheel degrees for a straight distance in mm (pi ~ 3.14)."""
    return mm * 36000 // (314 * WHEEL_DIA_MM)


# --- driving (percent motor power) ---
BASE = 35                # cruising power
LEFT_GAIN = 90           # measured: left motor ~10% stronger
RIGHT_GAIN = 100
SLEW = 10                # max power change per tick: gentle starts on the slick sheet
MIN_DUTY = 22            # a wheel below this power stalls on the sheet
BACK_PWR = 30
BACKUP_DEG = mm_to_deg(60)      # back up 6 cm before every bump turn

# --- heading (gyro) ---
HOLD_KP_X10 = 15         # straight driving: 1.5 power per degree off the target heading
HOLD_KI_X1000 = 30       # slowly removes a constant offset (one motor stronger)
HOLD_MAX = 10            # max power difference used to hold the heading
SIGN_CHECK_DEG = 10      # startup: small right turn to learn which way the gyro counts

# --- turns (gyro) ---
TURN_PWR = 35
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

# --- left sonar (mm) ---
SONAR_CAP = 1000
LEFT_OPEN_MM = 80        # bump decision: left is open if the wall is 8 cm or farther
WALL_SEE = 300           # a wall closer than this on the left = start following it
WALL_TARGET = 150        # distance to keep from the left wall
WALL_LOST = 380          # farther than this while following = the wall ended (door/corner)
LOST_CONFIRM_MS = 500
MIN_FOLLOW_MS = 1500     # only a wall followed this long counts as "ended"
CORNER_ADVANCE_DEG = mm_to_deg(120)  # drive on ~12 cm so the body clears the wall end
LEAN_MDEG_PER_MM = 100   # wall following: 1 deg of lean per cm of distance error...
LEAN_MAX_DEG = 10        # ...at most 10 deg off the grid line
LOOP_TURNS = 4           # 4 turns the same way (360 deg) = going round in a loop
LOOP_BREAK_MS = 2000     # leave the loop after following the next wall this long

# --- treated as a bump (the bumper did not catch it) ---
REPEAT_MS = 3000         # bumped again this soon after a turn = boxed in
STALL_MS = 1000          # a wheel powered forward but turning less than...
STALL_DEG = 15           # ...this in STALL_MS = stuck
WEDGE_MM = 45            # sonar this close...
WEDGE_TICKS = 55         # ...for ~2 s = wedged against the wall
SIDEHIT_DEG = 15         # knocked this far off the heading while driving...
SIDEHIT_TICKS = 10       # ...for ~0.35 s = a corner hit the side

# --- fire pad (BLACK) and tip-over ---
PAD_MAX = 2              # pad = reading below this (measured: pad 0-1, floor 13-25, shadows 2-5)
MIN_FLOOR = 8            # floor must read at least this at startup or detection is off
FIRE_CONFIRM = 8         # dark ticks in a row before GoalFind stops to check
GOAL_ERR_DEG = 4         # ignore dark readings while this far off course (robot tipping)
TIP_MS = 3000            # dark (no floor) this long without a fire = tipped over / lifted
UPRIGHT_MS = 1500        # floor seen again this long = standing again
FAN_MS = 10000

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
    busy = False         # Escape is backing up (color sensor not trusted)
    wedge_ticks = 0
    side_ticks = 0
    stall_t = 0
    stall_l = 0
    stall_r = 0
    dark_since = 0       # time the floor went dark (0 = not dark)
    turn_sum = 0         # net 90-deg turns, left = +1, right = -1 (loop detection)
    follow_off = False   # loop break: no wall following until the next bump


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
    C.l = ramp(C.l, min_duty(l * LEFT_GAIN // 100))
    C.r = ramp(C.r, min_duty(r * RIGHT_GAIN // 100))
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
    drive(base + steer, base - steer)


def turn_to(goal):
    """Rotate in place to the heading `goal` (milli-deg), the SHORT way. Fast, slow near the
    end, settle, trim. If the robot does not rotate (held by the sheet / a wall) add power
    in steps, rock back if needed. Returns the final error (milli-deg)."""
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
    chk_e = abs(wrap(goal - Heading.yaw))
    while clock.time() - t0 < TURN_MAX_MS:
        yaw_update()
        err = wrap(goal - Heading.yaw)
        if abs(err) <= GYRO_LEAD * 1000:
            break
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
    err = wrap(goal - Heading.yaw)
    log('E', 'TURN', 'to %d: now %d, error %d, trims %d boost %d rocks %d%s' % (
        goal // 1000, Heading.yaw // 1000, err // 1000, trims, peak, rocks,
        ' STUCK' if abs(err) > STUCK_TOL_DEG * 1000 else ''))
    return err


def back_up(deg):
    left.reset_angle(0)
    right.reset_angle(0)
    t = clock.time()
    while clock.time() - t < 1500 and -(left.angle() + right.angle()) // 2 < deg:
        yaw_update()
        drive(-BACK_PWR, -BACK_PWR)
        wait(10)
    stop_drive()


def turn_by(d, robot_deg):
    """Turn 90 or 180 deg (d=+1 left, -1 right) from the GRID line we are on, so any error
    from before is taken out by this turn. Counts the turn for loop detection."""
    S.turn_sum += d * (robot_deg // 90)
    if gyro is None:                               # fallback: wheel encoders
        stop_drive()
        left.reset_angle(0)
        right.reset_angle(0)
        target = robot_deg * 440 // 100
        t0 = clock.time()
        while clock.time() - t0 < TURN_MAX_MS and (right.angle() - left.angle()) * d // 2 < target:
            spin(d, TURN_PWR)
            wait(10)
        stop_drive()
        return
    Heading.target = snap90(Heading.target) - d * robot_deg * 1000
    err = turn_to(Heading.target)
    if abs(err) > STUCK_TOL_DEG * 1000:
        log('E', 'TURN', 'stuck %d deg short: back up and retry' % (abs(err) // 1000))
        back_up(RECOVER_DEG)
        err = turn_to(Heading.target)
    if abs(err) > STUCK_TOL_DEG * 1000:            # could not get there: drive on the grid
        Heading.target = snap90(Heading.yaw)       # line we actually face instead
        log('E', 'TURN', 'gave up: continue on the grid line at %d' % (Heading.target // 1000))


def advance(deg):
    """Straight on the grid for a fixed distance (wheel deg). False if something was hit."""
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
    S.bump = pressed
    S.bright = color.reflection()
    yaw_update()
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


# ======================= behaviors =======================
class Wander:
    """No wall beside us: drive straight on the grid until something is hit."""
    name = 'WANDER'

    def active(self):
        return True

    def act(self):
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

    def active(self):
        if S.follow_off:
            self.following = False
            return False
        if S.dist < WALL_SEE and not self.following:
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
                log('E', self.name, 'wall ended after %d ms: go round it to the left' % followed)
                if advance(CORNER_ADVANCE_DEG):
                    turn_by(1, 90)
            return
        self.lost_since = 0
        if abs(S.turn_sum) >= LOOP_TURNS and now - self.seen_since > LOOP_BREAK_MS:
            log('E', 'LOOP', 'turned 360 deg the same way: leave the loop to the right')
            S.turn_sum = 0
            S.follow_off = True                    # cross the room until the next bump
            self.following = False
            turn_by(-1, 90)
            return
        lean = (WALL_TARGET - d) * LEAN_MDEG_PER_MM    # too close -> lean right (+)
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
            S.hit = False
            S.follow_off = False                   # a bump ends a loop-break crossing
            stop_drive()
            left.reset_angle(0)
            right.reset_angle(0)
            self.t0 = now
            self.phase = Escape.BACK
            if now - self.last_end < REPEAT_MS:
                self.streak += 1
            else:
                self.streak = 1
            left_open = S.dist >= LEFT_OPEN_MM
            if self.streak == 2:                   # hit again right after a turn: boxed in
                self.dir = 1 if left_open else -1
                self.deg = 180
            elif self.streak >= 3:                 # still boxed in: try the right side
                self.dir = -1
                self.deg = 90
                self.streak = 0
            else:
                self.dir = 1 if left_open else -1
                self.deg = 90
            log('E', self.name, 'hit: back up, turn %s %d (left sonar %d mm, hit #%d)' % (
                'L' if self.dir > 0 else 'R', self.deg, S.dist, self.streak))
        if self.phase == Escape.BACK:
            drive(-BACK_PWR, -BACK_PWR)
            back = -(left.angle() + right.angle()) // 2
            if back >= BACKUP_DEG or now - self.t0 > 1500:
                turn_by(self.dir, self.deg)
                self.phase = Escape.IDLE
                self.last_end = clock.time()
                S.hit = False                      # ignore presses during the turn
                S.wedge_ticks = 0
                S.side_ticks = 0


class GoalFind:
    """Dark pad for several ticks while driving level and on course -> stop and re-check
    (identification). Confirmed -> alarm starts."""
    name = 'GOALFIND'

    def __init__(self):
        self.hits = 0

    def active(self):
        off = abs(heading_error()) > GOAL_ERR_DEG * 1000
        if not S.detect_ok or S.busy or off or S.bright >= S.thresh:
            self.hits = 0
        else:
            self.hits += 1
        return self.hits >= FIRE_CONFIRM

    def act(self):
        stop_drive()
        avg = average_reflection(8)
        log('E', self.name, 'recheck avg=%d thresh=%d' % (avg, S.thresh))
        if avg < S.thresh:
            S.fire = True
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
        ev3.light.on(Color.ORANGE)
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
    """Fire identified: do NOT move (driving on knocks the candle over). Fan on right
    where it stopped, alarm beeping, then finish."""
    name = 'EXTINGUISH'

    def active(self):
        return S.fire

    def act(self):
        stop_drive()
        fan.dc(100)
        log('E', self.name, 'fan on')
        t = clock.time()
        while clock.time() - t < FAN_MS:
            ev3.speaker.beep(1000, 200)
            wait(300)
        fan.stop()
        log('E', self.name, 'fan off, mission complete')
        ev3.light.on(Color.GREEN)
        ev3.speaker.beep(1500, 600)
        return True


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
    log('E', 'START', 'floor=%d thresh=%d BASE=%d WALL_TARGET=%d LEFT_OPEN=%d' % (
        S.floor, S.thresh, BASE, WALL_TARGET, LEFT_OPEN_MM))
    print('gyro:', 'connected' if gyro is not None else 'NOT found - turns use encoders')
    if not S.detect_ok:
        print('FIRE DETECTION OFF: floor reads', S.floor)
        log('E', 'START', 'detection disabled: floor too dark')
        ev3.speaker.beep(200, 800)
    gyro_startup()
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
                        ev3.screen.clear()
                        ev3.screen.print(last)
                        log('E', last, 'behavior start')
                    if b.act():
                        return
                    break
            now = clock.time()
            if now >= Log.next:
                Log.next = now + LOG_EVERY_MS
                log('S', last, 'yaw=%d tgt=%d turns=%d' % (
                    Heading.yaw // 1000, Heading.target // 1000, S.turn_sum))
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
