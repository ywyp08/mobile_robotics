#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Setup
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
ultra_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Settings
FORWARD_SPEED = 10     # mm/s
TURN_RATE = 30          # deg/s for correcting turns
ULTRA_THRESH = 50        # mm for obstacle

# Functions
def sensor_is_on_black():
    return color_sensor.color() == Color.BLACK

# Start
while True:
    if ultra_sensor.distance() < ULTRA_THRESH:
        robot.stop()
        ev3.speaker.beep()
        while ultra_sensor.distance() < ULTRA_THRESH:
            wait(10)
    if sensor_is_on_black():
        robot.drive(FORWARD_SPEED, +TURN_RATE)
    else:
        robot.drive(FORWARD_SPEED, -TURN_RATE)
    wait(10)
