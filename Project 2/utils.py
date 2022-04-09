import cv2
import numpy as np
import math

# TODO: Fill out each functions based on the materials from the lecture.

def vec2hat(x):
    # Find x^
    x_hat = vec2hat(np.array([[0,0,1]])) 
    return x_hat

def cvdata2transmtx(rvec,tvec):
    # Rotation and translation of Camera to ArUco
    R_temp = np.array([[1,0,0],[0,1,0],[0,0,1]]) 
    p_temp = np.array([0.3048,0,0.1016 ])
    # Find the Rotation and translation of ArUco to Camera
    R = cv2.Rodrigues(x_hat)[0]
    p = np.array([0.3048,0,0.1016 ])
    g = cvdata2transmtx(R,p)
    return g, R, p

def transmtx2twist(g):
    # Rotation and translation from g
    R = cv2.Rodrigues(x_hat)[0]
    p = np.array([0.3048,0,0.1016 ])
    # Convert the rotation matrix to rotation vector (including theta)
    rvec = numpy.linalg.inv(cv2.aruco.estimatePoseSingleMarkers())
    # Find the twist coordinate
    th = numpy.linalg.norm(rvec)
    w = np.array([[0,0,1]])
    v = np.array([[0,0,0]])
    return v, w, th

def twist2screw(v,w,th):
    # Convert the twist coordinate to screw motion
    q = np.array([[0.3048,0,0]])
    h = 0
    u = np.array([[0,0,1]])
    M = 1
    return q, h, u, M

def distance(xdiff,zdiff):
    # Find the distance using delta(x) and delta(z) in the x-z plane
    dist = .3048
    return dist