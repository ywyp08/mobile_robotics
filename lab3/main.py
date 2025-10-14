#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait, run_coroutine
from pybricks.exceptions import FileExistsError
import time

# Setup
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
color_sensor = ColorSensor(Port.S3)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Settings
FORWARD_SPEED = 100
WAIT_TIME = 50  # Reduced from 100ms to 50ms for real-time logging
TURN_BASE = 10
GREY_CHECK = True
LOG_FILE = "line_track_data.csv"  # Name of data file on EV3

# Initialize CSV data file
try:
    with open(LOG_FILE, "w") as f:
        f.write("timestamp,reflection,r,g,b,robot_state,turn_angle\n")  # Header
except FileExistsError:
    pass  # File exists, we'll overwrite

# Functions
def sees_green():
    return color_sensor.color() == Color.GREEN

def sees_grey():
    """Efficiently check for grey patches without long delays"""
    r, g, b = color_sensor.rgb()
    reflection = color_sensor.reflection()
    # Simplified grey check (no 5-second wait!)
    return (12 <= reflection <= 26) and (7 <= r <= 13) and (10 <= g <= 18) and (18 <= b <= 26)

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

def log_data(timestamp, reflection, r, g, b, robot_state, turn_angle):
    """Write sensor data to CSV file"""
    with open(LOG_FILE, "a") as f:
        f.write(f"{timestamp},{reflection},{r},{g},{b},{robot_state},{turn_angle}\n")

# Start line following with logging
run = True
last_timestamp = time.time()

while run:
    # Get sensor data
    reflection = color_sensor.reflection()
    r, g, b = color_sensor.rgb()
    current_timestamp = time.time()
    
    # Log data
    robot_state = "forward"
    if sees_green():
        robot.drive(FORWARD_SPEED, 0)
    elif GREY_CHECK and sees_grey():
        ev3.speaker.beep()
        robot_state = "grey_patch"
        robot.turn(360)  # Immediate turn to avoid getting stuck
        run = False  # Stop after first grey patch
    else:
        robot_state = "searching"
        robot.stop()
        turn_angle = TURN_BASE
        while not sees_green():
            look_around(turn_angle)
            turn_angle += TURN_BASE
    
    # Log data with timestamp
    log_data(current_timestamp, reflection, r, g, b, robot_state, turn_angle)

    # Wait for next iteration
    wait(WAIT_TIME)

# Final cleanup
print(f"Data saved to {LOG_FILE} on EV3")
ev3.speaker.beep()
