# Group Name: Juliuz Seizure
# Group Member: Kittibhum Penkul 6638018521,
#               Nuntis Lertbunditkul 6638103121
#               Panapon Thienmontree 6638114021
#               Pumipat Thanarapatvanic 6638187921


# coreXYCircle.py
# Hardware: Raspberry Pi Pico2 (RP2350) + 2x DM452 Microstep Driver
# CoreXY kinematics — Motor A = (X - Y), Motor B = (X + Y)
# Motor 0 (GP7/8/9) = Motor A,  Motor 1 (GP10/11/12) = Motor B
#
# PCB pin assignments (from PCB_2026 schematic):
#   GP0  = BUTTON_1 (toggle CW/CCW)
#   GP1  = BUTTON_2 (trigger homing)
#   GP2  = RED   (LED MOSFET gate)
#   GP3  = GREEN (LED MOSFET gate)
#   GP4  = BLUE  (LED MOSFET gate)
#   GP5  = LM_MIN (X limit switch — home end)
#   GP6  = LM_MAX (Y limit switch — home end)
#   GP7  = PUL_0  (X motor step)
#   GP8  = DIR_0  (X motor direction)
#   GP9  = ENA_0  (X motor enable)
#   GP28 = POT    (potentiometer ADC)
#
# Second DM452 (Y axis):
#   GP10 = PUL_1  (Y motor step)
#   GP11 = DIR_1  (Y motor direction)
#   GP12 = ENA_1  (Y motor enable)
#
# DM452 signal polarity (MOSFET open-drain):
#   Pico HIGH → MOSFET ON → signal- pulled LOW → DM452 sees active signal
#   ENA: LOW = motor ENABLED (ENA not active), HIGH = motor DISABLED (ENA active)
#   PUL: rising edge = one step
#   DIR: HIGH = forward, LOW = reverse

import machine
import time
import math

# =============================================================================
# SECTION 1: PINS
# =============================================================================

# --- X axis motor (Motor 0, first DM452) ---
STEP_X = machine.Pin(7,  machine.Pin.OUT)  # GP7  = PUL_0
DIR_X  = machine.Pin(8,  machine.Pin.OUT)  # GP8  = DIR_0
ENA_X  = machine.Pin(9,  machine.Pin.OUT)  # GP9  = ENA_0

# --- Y axis motor (Motor 1, second DM452) ---
STEP_Y = machine.Pin(10, machine.Pin.OUT)  # GP10 = PUL_1
DIR_Y  = machine.Pin(11, machine.Pin.OUT)  # GP11 = DIR_1
ENA_Y  = machine.Pin(12, machine.Pin.OUT)  # GP12 = ENA_1

# --- Limit switches (active LOW — 3V3 pull-up on PCB, switch pulls to GND) ---
LX = machine.Pin(5, machine.Pin.IN, machine.Pin.PULL_UP)  # GP5 = LM_MIN → X home
LY = machine.Pin(6, machine.Pin.IN, machine.Pin.PULL_UP)  # GP6 = LM_MAX → Y home

# --- RGB LED (common-anode, MOSFET open-drain per channel) ---
# Pico HIGH → MOSFET ON → cathode LOW → LED ON
# PWM duty 0–65535: 65535 = full brightness ON
led_r = machine.PWM(machine.Pin(2)); led_r.freq(1000)   # GP2 = RED
led_g = machine.PWM(machine.Pin(3)); led_g.freq(1000)   # GP3 = GREEN
led_b = machine.PWM(machine.Pin(4)); led_b.freq(1000)   # GP4 = BLUE

# --- Buttons (active LOW, internal pull-up) ---
btn1 = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)  # GP0 = BUTTON1 → toggle CW/CCW
btn2 = machine.Pin(1, machine.Pin.IN, machine.Pin.PULL_UP)  # GP1 = BUTTON2 → trigger homing

# --- Potentiometer ---
pot = machine.ADC(28)   # GP28 = POT (ADC2)

# =============================================================================
# SECTION 2: MACHINE PARAMETERS
# =============================================================================

# DM452 pulses/rev — set by SW1/SW2/SW3 dip switches on each driver
# ⚠️ Both drivers must be set to the same value for a correct circle
# Common settings: 400=x2, 800=x4, 1600=x8, 3200=x16
# Check the table printed on your DM452 and match this number
PULSES_PER_REV = 800    # ⚠️ match your DM452 SW setting

PULLEY_TEETH   = 40     # 2GT 40T pulley (assembly manual Part #11)
GT2_PITCH      = 2.0    # mm per tooth

MM_PER_REV     = PULLEY_TEETH * GT2_PITCH       # 80.0 mm / rev
MM_PER_PULSE   = MM_PER_REV / PULSES_PER_REV    # mm per step pulse

MIN_SPEED_MM   = 100    # mm/sec (pot fully left  = slowest)
MAX_SPEED_MM   = 700    # mm/sec (pot fully right = fastest)
HOMING_SPEED   = 450     # mm/sec (fixed slow speed for homing)

# CoreXY motor direction inversion (matching friend's working code)
M0_INVERT = 1   # Motor A direction inverted
M1_INVERT = 1   # Motor B direction inverted

# Circle
CIRCLE_RADIUS = 280      # mm — ⚠️ must fit inside your machine travel area
CIRCLE_STEPS  = 72      # line segments per full circle (more = smoother)

# =============================================================================
# SECTION 3: STATE
# =============================================================================
homed       = False
direction   = 1         # 1 = CW,  -1 = CCW
DEBOUNCE_MS = 50
last_btn_ms = 0

# =============================================================================
# SECTION 4: HELPERS
# =============================================================================

def enableAll():
    """Enable both DM452 drivers (active-low through MOSFET)."""
    ENA_X.value(0)
    ENA_Y.value(0)

def disableAll():
    """Disable both DM452 drivers (motors free to turn by hand)."""
    ENA_X.value(1)
    ENA_Y.value(1)

def set_led(r, g, b, brightness=1.0):
    """
    r, g, b   : 0 = off,  1 = on
    brightness: 0.0 – 1.0
    Common-anode + MOSFET: duty 65535 = full ON
    """
    scale = int(brightness * 65535)
    led_r.duty_u16(r * scale)
    led_g.duty_u16(g * scale)
    led_b.duty_u16(b * scale)

def led_off():
    led_r.duty_u16(0)
    led_g.duty_u16(0)
    led_b.duty_u16(0)

def get_speed():
    """Read potentiometer → speed mm/sec between MIN and MAX."""
    t = pot.read_u16() / 65535
    return MIN_SPEED_MM + t * (MAX_SPEED_MM - MIN_SPEED_MM)

def get_brightness():
    """Read potentiometer → LED brightness 0.1 – 1.0."""
    return 0.1 + 0.9 * (pot.read_u16() / 65535)

def check_buttons():
    """Non-blocking button check. Returns 'toggle', 'home', or None."""
    global last_btn_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, last_btn_ms) < DEBOUNCE_MS:
        return None
    if btn1.value() == 0:
        last_btn_ms = now
        while btn1.value() == 0: time.sleep_ms(10)
        return 'toggle'
    if btn2.value() == 0:
        last_btn_ms = now
        while btn2.value() == 0: time.sleep_ms(10)
        return 'home'
    return None

# =============================================================================
# SECTION 5: MOTION — CoreXY kinematics
# =============================================================================

def _pulse_both(do_a, do_b, delay_us):
    """Fire step pulses on whichever motors are flagged."""
    if do_a: STEP_X.value(1)   # Motor A
    if do_b: STEP_Y.value(1)   # Motor B
    time.sleep_us(delay_us)
    STEP_X.value(0)
    STEP_Y.value(0)
    time.sleep_us(delay_us)

def move_mm(dx, dy,
            speed_mm=None,
            stop_lx=False,
            stop_ly=False):
    """
    Move carriage by dx, dy mm using CoreXY kinematics.
    CoreXY: Motor A = X-Y, Motor B = X+Y

    stop_lx : abort if X limit switch (LX) triggers (value == 1)
    stop_ly : abort if Y limit switch (LY) triggers (value == 1)
    Returns True if completed, False if stopped by a limit switch.
    """
    if speed_mm is None:
        speed_mm = get_speed()

    # CoreXY transformation
    raw_a = (dx - dy) / MM_PER_PULSE   # Motor A = X - Y
    raw_b = (dx + dy) / MM_PER_PULSE   # Motor B = X + Y

    steps_a = round(abs(raw_a))
    steps_b = round(abs(raw_b))

    # Direction with inversion
    dir_a = 1 if raw_a >= 0 else 0
    dir_b = 1 if raw_b >= 0 else 0

    if M0_INVERT: dir_a ^= 1
    if M1_INVERT: dir_b ^= 1

    DIR_X.value(dir_a)   # Motor A
    DIR_Y.value(dir_b)   # Motor B

    # Step half-period timing
    pulse_rate = speed_mm / MM_PER_PULSE
    delay_us   = max(50, int(500000 / pulse_rate))

    total = max(steps_a, steps_b)
    if total == 0:
        return True

    # Bresenham interleave — both motors finish simultaneously
    err_a = total // 2
    err_b = total // 2

    for _ in range(total):
        if stop_lx and LX.value() == 1:   # X limit triggered (NC opens → HIGH)
            return False
        if stop_ly and LY.value() == 1:   # Y limit triggered (NC opens → HIGH)
            return False

        do_a = False
        do_b = False

        err_a += steps_a
        if err_a >= total:
            err_a -= total
            do_a = True

        err_b += steps_b
        if err_b >= total:
            err_b -= total
            do_b = True

        _pulse_both(do_a, do_b, delay_us)

    return True

# =============================================================================
# SECTION 6: HOMING
# =============================================================================

def home_axis(axis, limit_pin, speed_mm=HOMING_SPEED):
    """
    Home a single axis. Moves in the negative direction only.
    Uses one big move with in-step switch monitoring (stop_lx/stop_ly).
    Returns -1 if successful, 0 on failure.
    """
    MAX_TRAVEL = 10000   # mm max distance

    if axis == 'X':
        print("  Homing X (positive direction)...")
        result = move_mm(MAX_TRAVEL, 0, speed_mm=speed_mm, stop_lx=True)
        home_dir = 1
    else:
        print("  Homing Y (negative direction)...")
        result = move_mm(0, -MAX_TRAVEL, speed_mm=speed_mm, stop_ly=True)
        home_dir = -1

    if not result:   # stopped by limit switch (value went HIGH)
        print("  " + axis + " limit switch triggered!")
        return home_dir

    print("  WARNING: " + axis + " switch never triggered!")
    return 0

def home():
    """
    Drive toward the home corner until both limit switches trigger.
    Homes Y axis first, then X axis, only moving in the negative direction.
    LED = RED during homing, BLUE while moving to start position.
    """
    global homed
    print("Homing started...")
    enableAll()
    set_led(1, 0, 0, get_brightness())   # RED

    # Home Y axis FIRST
    print("Homing Y axis...")
    y_dir = home_axis('Y', LY)

    # Home X axis SECOND
    print("Homing X axis...")
    x_dir = home_axis('X', LX)

    print("Home found. Backing off...")
    # Since we move negative to home (-1), backing off uses positive direction
    move_mm(-x_dir * 5, -y_dir * 5, speed_mm=HOMING_SPEED)

    homed = True
    print("Homing complete.")

    # Move to circle centre position
    print("Moving to start position...")
    set_led(0, 0, 1, get_brightness())    # BLUE
    move_mm(-x_dir * 1000, -y_dir * 1000, speed_mm=HOMING_SPEED)
    print("Ready.")

# =============================================================================
# SECTION 7: CIRCLE
# =============================================================================

def draw_circle_step(angle_deg, direction, speed_mm):
    """
    Move one chord segment of the circle.
    angle_deg : current angle position (degrees)
    direction : 1 = CW,  -1 = CCW
    Returns the next angle.
    """
    step_deg   = direction * (360.0 / CIRCLE_STEPS)
    next_angle = angle_deg + step_deg

    cur_x  = CIRCLE_RADIUS * math.cos(math.radians(angle_deg))
    cur_y  = CIRCLE_RADIUS * math.sin(math.radians(angle_deg))
    next_x = CIRCLE_RADIUS * math.cos(math.radians(next_angle))
    next_y = CIRCLE_RADIUS * math.sin(math.radians(next_angle))

    move_mm(next_x - cur_x, next_y - cur_y, speed_mm=speed_mm)
    return next_angle % 360

# =============================================================================
# SECTION 8: MAIN LOOP
# =============================================================================

print("CoreXY Circle Program — 2x DM452 CoreXY drive")
print("  BTN1 (GP0): Toggle CW / CCW")
print("  BTN2 (GP1): Trigger homing")
print("  POT  (GP28): Speed + LED brightness")
print("")

enableAll()
home()

angle = 0.0
print("Drawing circle. Direction = CW")
set_led(0, 1, 0, get_brightness())   # GREEN — circling

try:
    while True:
        btn = check_buttons()

        if btn == 'home':
            print("Re-homing...")
            angle = 0.0
            home()
            set_led(0, 1, 0, get_brightness())

        elif btn == 'toggle':
            direction *= -1
            print("Direction:", "CW" if direction == 1 else "CCW")

        # Read pot every loop — controls both speed and LED brightness
        speed      = get_speed()
        brightness = get_brightness()
        set_led(0, 1, 0, brightness)   # GREEN, brightness tracks pot

        # Move one chord of the circle
        angle = draw_circle_step(angle, direction, speed)

finally:
    disableAll()
    led_off()
    print("Motors deactivated.")
