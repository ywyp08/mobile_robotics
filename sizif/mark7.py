#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np
import common.misc as Misc
import common.yaml_handle as yaml_handle

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# Calibration
TARGET_PICK_Z = -2.5

OFFSET_Y = 1.0  
OFFSET_X = 0.0  

ROBOT_X_RANGE = (-16, 16) 
ROBOT_Y_RANGE = (12, 28)

# Colors
Coordinates_data = yaml_handle.get_yaml_data(yaml_handle.PickingCoordinates_file_path)

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Functions
lab_data = None
def load_config():
    global lab_data
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

    lab_data['blue'] = {
        'min': [30, 40, 0],
        'max': [255, 127, 120] 
    }

__target_color = ('green',) 

def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    return (True, ())

def getAreaMaxContour(contours) :
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None
        for c in contours : 
            contour_area_temp = math.fabs(cv2.contourArea(c)) 
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300: 
                    area_max_contour = c
        return area_max_contour, contour_area_max 

servo1 = 1500

def initMove():
    board.pwm_servo_set_position(0.3, [[1, servo1]])
    AK.setPitchRangeMoving((0, 8, 10), 0,-90, 90,1500)

def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

_stop = False
color_list = []
__isRunning = False
detect_color = 'None'
start_pick_up = False

tower_step = 0  
tower_order = ['green', 'red', 'blue'] 

goal_x = -999 
goal_y = -999
last_seen_time = 0 

def reset():
    global _stop, color_list, detect_color, start_pick_up, __target_color, tower_step
    global goal_x, goal_y, last_seen_time
    _stop = False
    color_list = []
    detect_color = 'None'
    start_pick_up = False
    tower_step = 0
    goal_x = -999
    goal_y = -999
    last_seen_time = time.time()
    __target_color = (tower_order[0],)

def init():
    print("ColorSorting Init")
    load_config()
    initMove()

def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorSorting Start")

def stop():
    global _stop, __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorSorting Stop")

size = (320, 240)
unreachable = False 

def move():
    global _stop, unreachable, __isRunning, detect_color, start_pick_up, tower_step, __target_color
    global goal_x, goal_y, last_seen_time
    
    tower_x = -11
    tower_y = 15
    block_height = 3.0
    
    search_state = 0
    
    while True:
        if __isRunning:
            
            if detect_color == tower_order[tower_step] and start_pick_up and goal_x != -999 and goal_y > 5:
                
                search_state = 0
                last_seen_time = time.time()
                
                set_rgb(detect_color)
                board.set_buzzer(1900, 0.1, 0.9, 1)
                
                print(f"Phase 1: Hovering {goal_x}, {goal_y}")
                board.pwm_servo_set_position(0.5, [[1, 2000]]) 
                AK.setPitchRangeMoving((goal_x + OFFSET_X, goal_y + OFFSET_Y, 12), -90, -90, 0, 1000)
                time.sleep(1.5) 
                
                final_x = goal_x + OFFSET_X
                final_y = goal_y + OFFSET_Y
                print(f"Phase 2: Corrected to {final_x}, {final_y}")

                grab_success = False
                for z_try in np.arange(TARGET_PICK_Z, 1.0, 0.5):
                    result = AK.setPitchRangeMoving((final_x, final_y, z_try), -90, -90, 0, 1000)
                    if result != False:
                        print(f"  -> Grabbing at Z={z_try}")
                        grab_success = True
                        break 
                
                if not grab_success:
                    print("  -> Ground unreachable. Resetting.")
                    start_pick_up = False
                    detect_color = 'None'
                    AK.setPitchRangeMoving((0, 10, 15), -90, -90, 0, 1000)
                    time.sleep(1)
                    continue

                time.sleep(0.8) 
                board.pwm_servo_set_position(0.5, [[1, 1500]])
                time.sleep(0.5)
                AK.setPitchRangeMoving((0, 6, 20), 0,-90, 90, 1500)
                time.sleep(1.5)
                
                print("Phase 3: Gentle Stacking")
                board.pwm_servo_set_position(0.5, [[6, 1900]]) 
                time.sleep(0.5)

                current_z = tower_step * block_height
                
                AK.setPitchRangeMoving((tower_x, tower_y, current_z + 3), -90, -90, 0, 1500)
                time.sleep(1.5)
                
                AK.setPitchRangeMoving((tower_x, tower_y, current_z), -90, -90, 0, 2000) 
                time.sleep(2.0)
                
                board.pwm_servo_set_position(0.5, [[1, 1800]]) 
                time.sleep(0.5)

                AK.setPitchRangeMoving((tower_x, tower_y, current_z + 8), -90, -90, 0, 2000) 
                time.sleep(1.5)
                
                board.pwm_servo_set_position(1.2, [[1, 1500],[3, 515],[4, 2170],[5, 945]])
                time.sleep(1.2)
                board.pwm_servo_set_position(0.5, [[6, 1500]])
                time.sleep(0.5)
                AK.setPitchRangeMoving((0, 8, 10), -90, -90, 0, 1000)
                time.sleep(1)
                
                start_pick_up = False
                detect_color = 'None'
                set_rgb(detect_color)
                
                tower_step += 1 
                if tower_step >= 3:
                    print("Tower Complete! Stopping.")
                    stop()
                else:
                    next_color = tower_order[tower_step]
                    print(f"Next Target: {next_color}")
                    __target_color = (next_color,)
                    last_seen_time = time.time()
            
            # search logic
            else:
                elapsed = time.time() - last_seen_time
                if elapsed > 3.0:
                    if int(elapsed * 10) % 10 == 0:
                        print(f"Searching... Time: {elapsed:.1f}s")
                    
                    if search_state == 0:
                        print("SEARCH ACTION: Lift Head")
                        AK.setPitchRangeMoving((0, 15, 25), -90, -90, 0, 1500)
                        search_state = 1
                        time.sleep(1.5)
                    elif search_state == 1:
                        print("SEARCH ACTION: Scan Left")
                        AK.setPitchRangeMoving((-18, 15, 25), -90, -90, 0, 1500)
                        search_state = 2
                        time.sleep(1.5)
                    elif search_state == 2:
                        print("SEARCH ACTION: Scan Right")
                        AK.setPitchRangeMoving((18, 15, 25), -90, -90, 0, 1500)
                        search_state = 1
                        time.sleep(1.5)
                else:
                    search_state = 0
                    time.sleep(0.01)

        else:
            time.sleep(0.01)
            
th = threading.Thread(target=move)
th.daemon = True
th.start()      

draw_color = range_rgb["black"]

def run(img):
    global unreachable, __isRunning, start_pick_up, detect_color, draw_color, color_list, __target_color
    global goal_x, goal_y, last_seen_time
    
    if not __isRunning:
        return img
    else:
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB) 

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0
        
        for i in lab_data:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2])) 
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) 
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) 
                closed[0:80, :] = 0
                closed[:, 0:120] = 0
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] 
                areaMaxContour, area_max = getAreaMaxContour(contours) 
                
                if areaMaxContour is not None:
                    if area_max > max_area: 
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        
        if max_area > 500: 
            (center_x, center_y), radius = cv2.minEnclosingCircle(areaMaxContour_max) 
            
            raw_x = Misc.map(center_x, 0, 320, ROBOT_X_RANGE[0], ROBOT_X_RANGE[1])
            raw_y = Misc.map(center_y, 0, 240, ROBOT_Y_RANGE[1], ROBOT_Y_RANGE[0])
            goal_x = round(raw_x, 2)
            goal_y = round(raw_y, 2)
            
            if color_area_max == tower_order[tower_step]:
                last_seen_time = time.time()

            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[color_area_max], 2)
            
            if not start_pick_up:
                if color_area_max == tower_order[tower_step]:
                    if color_area_max == 'red': color = 1
                    elif color_area_max == 'green': color = 2
                    elif color_area_max == 'blue': color = 3
                    else: color = 0
                    
                    color_list.append(color)
                    if len(color_list) == 3: 
                        color = int(round(np.mean(np.array(color_list))))
                        color_list = []
                        if color == 1 and tower_order[tower_step] == 'red':
                            detect_color = 'red'
                            draw_color = range_rgb["red"]
                            start_pick_up = True
                        elif color == 2 and tower_order[tower_step] == 'green':
                            detect_color = 'green'
                            draw_color = range_rgb["green"]
                            start_pick_up = True
                        elif color == 3 and tower_order[tower_step] == 'blue':
                            detect_color = 'blue'
                            draw_color = range_rgb["blue"]
                            start_pick_up = True
        else:
            if not start_pick_up:
                detect_color = "None"
                draw_color = (0, 0, 0)
        
        cv2.putText(img, f"Step: {tower_order[tower_step]}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
        
        elapsed = time.time() - last_seen_time
        if elapsed > 1.0:
            cv2.putText(img, f"Wait: {elapsed:.1f}s", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        return img

# Main
if __name__ == '__main__':
    from kinematics.arm_move_ik import *
    from common.ros_robot_controller_sdk import Board
    board = Board()
    AK = ArmIK()
    AK.board = board
    
    init()
    start()
    
    cap = cv2.VideoCapture('http://127.0.0.1:8080?action=stream')
    
    while True:
        ret,img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
