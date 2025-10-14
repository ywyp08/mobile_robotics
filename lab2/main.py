#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Setup
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
color_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Settings
FORWARD_SPEED = 100
WAIT_TIME = 100
TURN_BASE = 10
GREY_CHECK = False

# Functions
def sees_green():
    return color_sensor.color() == Color.GREEN

def sees_grey():
    r, g, b = color_sensor.rgb()
    reflection = color_sensor.reflection()
    rgb_range = (7 <= r <= 13) and (10 <= g <= 18) and (18 <= b <= 26)
    reflection_range = (12 <= reflection <= 26)
    ev3.screen.clear()
    ev3.screen.print("R:", r, "G:", g, "B:", b)
    ev3.screen.print("Ref:", reflection)
    wait(5000)
    return rgb_range and reflection_range

def look_around(turn_angle):
    robot.turn(turn_angle)
    wait(WAIT_TIME)
    if sees_green():
        return
    robot.turn(-1*turn_angle)
    robot.turn(-1*turn_angle)
    wait(WAIT_TIME)
    if sees_green():
        return
    robot.turn(turn_angle)

# Start
grey_patch = 0
run = True
while run:
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

