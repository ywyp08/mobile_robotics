#!/usr/bin/python3
# coding=utf8
import time
from armpi_mini_sdk.kinematics_sdk.kinematics.arm_move_ik import ArmIK
from armpi_mini_sdk.common_sdk.common.ros_robot_controller_sdk import Board

AK = ArmIK()
board = Board()

AK.board = board

coordinate = {
    'base'  : (1.5, 13, 10),
    'top'   : (1.5, 13, 6),
    'mid'   : (1.5, 13, 3),
    'bottom': (1.5, 13, 0),
    'red'   : (-11, 15, 0),
    'green' : (-11, 8, 0),
    'blue'  : (-11, 0, 0)
}

def move(pos, t=1.0):
    AK.setPitchRangeMoving(pos, -90, -90, 0, int(t * 1000))
    time.sleep(t)

def gripper(open=True):
    pulse = 2000 if open else 500
    board.pwm_servo_set_position(0.5, [[1, pulse]])
    time.sleep(0.5)

def sort_block(pick_pos, drop_pos):
    gripper(True)
    move(coordinate['base'])
    move(pick_pos)
    gripper(False)
    move(coordinate['base'])
    move(drop_pos)
    gripper(True)

def deconstruct():
    move(coordinate['base'])
    sort_block(coordinate['top'], coordinate['blue'])
    sort_block(coordinate['mid'], coordinate['red'])
    sort_block(coordinate['bottom'], coordinate['green'])
    move(coordinate['base'])

def construct():
    move(coordinate['base'])
    sort_block(coordinate['green'], coordinate['bottom'])
    sort_block(coordinate['red'], coordinate['mid'])
    sort_block(coordinate['blue'], coordinate['top'])
    move(coordinate['base'])

while(1):
    deconstruct()
    construct()

