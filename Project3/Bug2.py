import RPi.GPIO as GPIO
import cv2
import numpy as np
import yaml
import utils
import math
from AlphaBot import AlphaBot
import time
import pickle
from TRSensor import TRSensor

Ab = AlphaBot()
Ab.stop()

class point(object):
    def __init__(self):
        point.x = None
        point.z = None

def turn(Ab):
    for i in range(5):
        left(Ab)

def left(Ab):
    Ab.setMotor(-35,-35)
    time.sleep(0.05)
    Ab.stop()

def right(Ab):
    Ab.setMotor(35,35)
    time.sleep(0.05)
    Ab.stop()

def forward(Ab):
    Ab.setMotor(30,-32)
    time.sleep(0.1)
    Ab.stop()
    
def backward(Ab):
    Ab.setMotor(-30,32)
    time.sleep(0.1)
    Ab.stop()

def go_to_goal(Ab,TR,cap,goal_id,goal,hit,deg_eps,dist_eps,last_proportional,angle_to_goal):
    print("Warming up the Line track sensors...")
    time.sleep(0.5)
    for i in range(0,100):
        position = TR.readLine()
        proportional = position - 2000
        last_proportional = proportional
    time.sleep(0.5)
    print("Fininished warmup")
    while cap.isOpened():
        ret, frame = cap.read()
        position = TR.readLine()
        proportional = position - 2000
        derivative = proportional - last_proportional
        last_proportional = proportional
        # When the IR sensor detects the line...
        if abs(derivative) >= 500:
            state = 1
            print("Hit line...")
            hit.x = p_gc[0]
            hit.z = p_gc[2]
            turn(Ab)
            return state, goal, hit, last_proportional, angle_to_goal
        elif ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if (len(corners)!=0):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i,_ in enumerate(rvecs):
                    if ids[i] == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                        p_gc = g_gc[:,3]
                        th = utils.transmtx2twist(g_gc)[2]
                        if angle_to_goal is None:
                            goal.z = 0.1
                            # This is the angle between the robot's initial position to the Goal marker frame's origin
                            # The goal point will be set along this line near to the Goal marker
                            angle_to_goal = math.atan(p_gc[0]/p_gc[2])
                            goal.x = goal.z*math.tan(angle_to_goal)
                            print("Set Goal Point x:{} z:{}".format(goal.x,goal.z))
                        xdiff = p_gc[0]-goal.x
                        zdiff = p_gc[2]-goal.z
                        cur_dist = utils.distance(xdiff,zdiff)
                        if cur_dist <= dist_eps:
                            print("Reached goal point!")
                            cv2.destroyAllWindows()
                            Ab.stop()
                            state = 3
                            return state, goal, hit, last_proportional, angle_to_goal
                        # TODO: fill out the which movements (left, right, forward, etc) should be running
                        #       in each case
                        if zdiff < 0:
                            
                        else:
                            
                    else:
                        
            else:
                if 'th' in locals(): # 
                    if th - angle_to_goal > 0:
                        
                    else:
                        
                else:
                    
                    
            cv2.imshow('aruco',frame)
            key = cv2.waitKey(100) & 0xFF
            # Press q to stop in the middle of the process
            if key == ord('q'):
                cv2.destroyAllWindows()
                Ab.stop()
                state = 3
                return state, goal, hit, last_proportional, angle_to_goal
                
def follow_line(Ab,TR,cap,goal_id,helper1_id,helper2_id,goal,hit,dist_eps,g_gh1,g_gh2,last_proportional,angle_to_goal):
    maximum = 35
    count = 0
    while cap.isOpened():
        Ab.backward()
        position = TR.readLine()
        # The "proportional" term should be 0 when we are on the line.
        proportional = position - 2000
        # Compute the derivative (change) and integral (sum) of the position.
        derivative = proportional - last_proportional
        # Remember the last position.
        last_proportional = proportional
        power_difference = proportional/25 + derivative/100 #+integral/500
        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        if (power_difference < 0):
            Ab.setPWMB(maximum + power_difference)
            Ab.setPWMA(maximum)
        else:
            Ab.setPWMB(maximum)
            Ab.setPWMA(maximum - power_difference)
        time.sleep(0.05)
        Ab.stop()
        ret, frame = cap.read()
        count += 1
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if (len(corners)!=0):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i,_ in enumerate(rvecs):
                    # TODO: Please fill out when the markers are detected.
                    #       The code is very similar to find_leave() function in Bug1.py.
                    if ids[i] == goal_id:
                        
                    elif ids[i] == helper1_id:
                        
                    elif ids[i] == helper2_id:
                        
            cv2.imshow('aruco',frame)
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                Ab.stop()
                state = 3
                return state, last_proportional

# Setups for Alphabot
CS = 5
Clock = 25
Address = 24
DataOut = 23

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Clock,GPIO.OUT)
GPIO.setup(Address,GPIO.OUT)
GPIO.setup(CS,GPIO.OUT)
GPIO.setup(DataOut,GPIO.IN,GPIO.PUD_UP)

last_proportional = 0

with open('ir_calib.pkl', 'rb') as ir_calib_file:
    TR = pickle.load(ir_calib_file)
    Ab.stop()

# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

# Side length of the ArUco marker in meters 
marker_length = 0.05

# initialize the points
goal = point()
hit = point()

angle_to_goal = None 

# Calibration parameters yaml file
with open(r'calib_data_bug1.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])
g_gh1 = np.asarray(calib_data["g_gh1"])
g_gh2 = np.asarray(calib_data["g_gh2"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

#marker ids
goal_id = 2
helper1_id = 1
helper2_id = 0

#epsilons
deg_eps = 0.1
dist_eps = 0.2

# flags
state = 0

try:
    while True:
        if state == 0:
            state, goal, hit, last_proportional, angle_to_goal = go_to_goal(Ab,TR,cap,goal_id,goal,hit,deg_eps,dist_eps,last_proportional,angle_to_goal)
        elif state == 1:
            state, last_proportional = follow_line(Ab,TR,cap,goal_id,helper1_id,helper2_id,goal,hit,dist_eps,g_gh1,g_gh2,last_proportional,angle_to_goal)
        elif state == 2:
            print("Finished Bug 2!")
            break

except KeyboardInterrupt:
    cap.release()
    cv2.destroyAllWindows()
    Ab.stop()