import pigpio
import time
import cv2
import numpy as np
import yaml
import utils
import math

# function that moves the servo motor btw 0~180 degrees
def ServoAngle2PulseWidth(angle):
    return (500.0 + 2000.0*angle/180)

# Setting the Alphabot and the servo motor
# Use the servo motor moving around
SERVO = 27# it could be 22 depending on where you connected
p = pigpio.pi()
p.set_mode(SERVO,pigpio.OUTPUT)
p.set_PWM_frequency(SERVO,50)

# define the initial and the next angle
# amount of rotation (theta) = next angle - initial angle
init_angle = 55
next_angle = 75

# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Side length of the ArUco marker in meters 
marker_length = 0.1
 
# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

# move the camera to the initial angle and start detecting the aruco marker
init_pw = ServoAngle2PulseWidth(init_angle)
p.set_servo_pulsewidth(SERVO,init_pw)
time.sleep(3)

print("Press s to save the initial data or press q to quit...")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if len(corners)!=0: # if aruco marker detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0,255,0))
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
        cv2.imshow("aruco", frame)
        key = cv2.waitKey(2) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            init_rvec = rvec
            init_tvec = tvec
            print("Initial data saved, press m to move the camera or q to quit...")
        elif key == ord('m'):
            next_pw = ServoAngle2PulseWidth(next_angle)
            p.set_servo_pulsewidth(SERVO,next_pw)
            time.sleep(3)
            print("Camera position changed, press r to save current data or q to quit...")
        elif key == ord('r'):
            next_rvec = rvec
            next_tvec = tvec
            print("Current data saved, press q to quit and start the calculation...")

# Turn off the servo and camera
p.set_PWM_dutycycle(SERVO,0)
p.set_PWM_frequency(SERVO,0)
cap.release()
cv2.destroyAllWindows()

if init_rvec.all() and next_rvec.all():
    # g(0)
    g0 = utils.cvdata2transmtx(init_rvec,init_tvec)[0]
    # g(th)
    gth = utils.cvdata2transmtx(next_rvec,next_tvec)[0] 
    # TODO: find exp^(hat(xi)*th) using g(0) and g(th)
    exp_mtx = 
    # The twist coordinate and screw motion of the servo
    v,w,th = utils.transmtx2twist(exp_mtx)
    q,h,u,M = utils.twist2screw(v,w,th)
    print("Estimated rotation angle: {} degrees".format(math.degrees(th)))
    print("Twist Coordinates:\n {}".format(np.vstack((v,w))*th))
    print("Screw motion:\n q:{},\n h:{},\n u:{},\n M:{}".format(q,h,u,M))