#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.tools import wait, StopWatch, DataLog
import math

# Setup
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
ultra_sensor = UltrasonicSensor(Port.S2)
color_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
watch = StopWatch()
data = DataLog('time_ms', 'x_mm', 'y_mm', 'heading_deg', 'speed', 'turn', 'end', name='path_log', timestamp=False, extension='csv')

# Settings
FORWARD_SPEED = 15     # mm/s
TURN_RATE = 50          # deg/s for correcting turns
ULTRA_THRESH = 50        # mm for obstacle

# Variables
x, y, heading = 0.0, 0.0, 0.0
last_distance = 0.0

# Functions
def sensor_is_on_black():
    return color_sensor.color() == Color.BLACK

# Start
while True:
    end = 0
    distance = robot.distance()
    delta_d = distance - last_distance
    last_distance = distance

    heading = robot.angle()
    rad = math.radians(heading)
    x += delta_d * math.cos(rad)
    y += delta_d * math.sin(rad)

    if ultra_sensor.distance() < ULTRA_THRESH:
        robot.stop()
        ev3.speaker.beep()
        while ultra_sensor.distance() < ULTRA_THRESH:
            wait(10)
    if sensor_is_on_black():
        speed = FORWARD_SPEED
        turn = +TURN_RATE
    else:
        speed = FORWARD_SPEED
        turn = -TURN_RATE

    robot.drive(speed, turn)

    data.log(watch.time(), x, y, heading, speed, turn, end)
    wait(10)
