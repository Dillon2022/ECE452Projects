import cv2
import numpy as np
import math

# TODO: Fill out each functions based on the materials from the lecture.

def vec2hat(x):
    # Find x^
    x_hat = np.array([[0,-x1[0],x2[0]],[x1[0],0,x3[0]],[-x2[0],x3[0],0]]) 
    return x_hat

def cvdata2transmtx(rvec,tvec):
    # Rotation and translation of Camera to ArUco
    R_temp = cv2.Rodrigues(rvec)[0] 
    p_temp = tvec
    # Find the Rotation and translation of ArUco to Camera
    R = numpy.linalg.inv(R_temp)[0]
    p = -p_temp
    g = np.array([[g00[0],g01[0],g02[0],g03[0]],[g10[0],g11[0],g12[0],g13[0]],[g20[0],g21[0],g22[0],g23[0]],[0,0,0,1]])
    return g, R, p

def transmtx2twist(g):
    # Rotation and translation from g
    R = numpy.hstack([[g00,g01,g02],[g10,g11,g12],[g20,g21,g22]])
    p = numpy.hstack([[g03],[g13],[g23]])
    # Convert the rotation matrix to rotation vector (including theta)
    theta= math.acos((R.trace()-1)/2)
    rvec = np.hstack([[r21-r12],[r02-r20],[r10-r01]])*(1- 2*math.sin(theta))
    # Find the twist coordinate
    th = np.linalg.norm(rvec)
    w = rvec
    v = 
    return v, w, th

def twist2screw(v,w,th):
    # Convert the twist coordinate to screw motion
    q = math.cross(w,v)/(numpy.linalg.norm(w))^2
    h = 
    u = np.array([[0,0,1]])
    M = 1
    return q, h, u, M

def distance(xdiff,zdiff):
    # Find the distance using delta(x) and delta(z) in the x-z plane
    dist = .3048
    return dist