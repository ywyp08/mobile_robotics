#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog
import math

# --- Setup ---
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
color_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
watch = StopWatch()

# --- Settings ---
FORWARD_SPEED = 100       # mm/s
WAIT_TIME = 100           # ms
TURN_BASE = 10            # deg
GREY_CHECK = False

# --- Data logger ---
# timestamp=False -> readable, constant filename; True -> adds timestamp
data = DataLog('time_ms', 'x_mm', 'y_mm', 'heading_deg', 'r', 'g', 'b', 'reflection', 'color',
               name='path_log', timestamp=False, extension='csv')

# --- Variables ---
x, y, heading = 0.0, 0.0, 0.0
last_distance = 0.0
grey_patch = 0
run = True

# --- Helper functions ---
def sees_green():
    return color_sensor.color() == Color.GREEN

def sees_grey():
    r, g, b = color_sensor.rgb()
    reflection = color_sensor.reflection()
    rgb_range = (7 <= r <= 13) and (10 <= g <= 18) and (18 <= b <= 26)
    reflection_range = (12 <= reflection <= 26)
    return rgb_range and reflection_range

def look_around(turn_angle):
    robot.turn(turn_angle)
    wait(WAIT_TIME)
    if sees_green():
        return
    robot.turn(-turn_angle)
    robot.turn(-turn_angle)
    wait(WAIT_TIME)
    if sees_green():
        return
    robot.turn(turn_angle)

# --- Start ---
robot.reset()
watch.reset()

while run:
    # --- Odometry update ---
    distance = robot.distance()
    delta_d = distance - last_distance
    last_distance = distance

    heading = robot.angle()
    rad = math.radians(heading)
    x += delta_d * math.cos(rad)
    y += delta_d * math.sin(rad)

    # --- Log to CSV ---
    t = watch.time()
    data.log(t, x, y, heading)

    # --- Behavior logic ---
    if sees_green():
        robot.drive(FORWARD_SPEED, 0)
    elif GREY_CHECK and sees_grey():
        ev3.speaker.beep()
        if grey_patch == 0:
            robot.turn(360)
            grey_patch += 1
        else:
            run = False
    else:
        robot.stop()
        wait(WAIT_TIME)
        turn_angle = TURN_BASE
        while not sees_green():
            look_around(turn_angle)
            turn_angle += TURN_BASE

    wait(WAIT_TIME)

robot.stop()
ev3.speaker.beep()
