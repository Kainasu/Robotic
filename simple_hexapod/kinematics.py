import numpy as np
from constants import *

def computeDK(alpha, beta, gamma, use_rads=True):
    l0 = constL1
    l1 = constL2
    l2 = constL3

    pt0 = LEG_CENTER_POS[1]

    x1 = np.cos(THETA1_MOTOR_SIGN * alpha) * l0 + pt0[0]
    y1 = np.sin(THETA1_MOTOR_SIGN * alpha) * l0 + pt0[1]
    z1 = pt0[2]
    pt1 = [x1,y1,z1]

    l = np.cos(THETA2_MOTOR_SIGN * beta) * l1
    x2 = np.cos(THETA1_MOTOR_SIGN * alpha) * l + pt1[0]
    y2 = np.sin(THETA1_MOTOR_SIGN * alpha) * l + pt1[1]
    z2 = pt1[2] - np.sin(THETA2_MOTOR_SIGN * beta) * l1
    pt2 = [x2,y2,z2]

    o1 = np.pi/2 - THETA2_MOTOR_SIGN* beta 
    gamma += theta3Correction
    o2 =  THETA3_MOTOR_SIGN* gamma - o1
    l = np.sin(o2) * l2
    x3 = -np.cos(THETA1_MOTOR_SIGN * alpha) * l + pt2[0]
    y3 = -np.sin(THETA1_MOTOR_SIGN * alpha) * l + pt2[1]
    z3 = pt2[2] - np.cos(THETA3_MOTOR_SIGN * o2) * l2
    pt3 = [x3,y3,z3]

    return pt3


def normalise(x):
    if ( x > 1): 
        x = 1
    elif x < -1:
        x = -1 
    return x


def computeIK(x, y, z, verbose=True, use_rads=True):   

    o0 = np.arctan2(y,x)

    l0 = constL1
    l1 = constL2
    l2 = constL3


    A = l0 * np.array([np.cos(o0), np.sin(o0), 0])
    C = np.array([x,y,z])
    
    AC = np.linalg.norm(C - A)

    o2 = np.pi - np.arccos(max(-1.0, min(1.0, (l1*l1 + l2*l2 - AC*AC)/(2*l1*l2))))
    L = np.sqrt((x - A[0])*(x - A[0]) + (y - A[1])*(y - A[1]))

    o1 = np.arccos(max(-1.0, min(1.0, (l1*l1 + AC*AC - l2*l2)/(2*AC*l1)))) + np.arctan2(z, L)

    
    return [THETA1_MOTOR_SIGN * o0, -THETA2_MOTOR_SIGN * (o1 - theta2Correction), THETA3_MOTOR_SIGN * (o2 + theta3Correction)]

def computeDKDetailed(alpha, beta, gamma, use_rads=True):
    points = []
    
    l0 = constL1
    l1 = constL2
    l2 = constL3

    pt0 = LEG_CENTER_POS[1]
    x1 = np.cos(THETA1_MOTOR_SIGN * alpha) * l0 + pt0[0]
    y1 = np.sin(THETA1_MOTOR_SIGN * alpha) * l0 + pt0[1]
    z1 = pt0[2]
    pt1 = [x1,y1,z1]

    beta -= theta2Correction
    l = np.cos(THETA2_MOTOR_SIGN * beta) * l1
    x2 = np.cos(THETA1_MOTOR_SIGN * alpha) * l + pt1[0]
    y2 = np.sin(THETA1_MOTOR_SIGN * alpha) * l + pt1[1]
    z2 = pt1[2] - np.sin(THETA2_MOTOR_SIGN * beta) * l1
    pt2 = [x2,y2,z2]

    o1 = np.pi/2 - THETA2_MOTOR_SIGN* beta 
    gamma += theta3Correction
    o2 =  THETA3_MOTOR_SIGN* gamma - o1
    l = np.sin(o2) * l2
    x3 = -THETA1_MOTOR_SIGN * np.cos(alpha) * l + pt2[0]
    y3 = -THETA1_MOTOR_SIGN * np.sin(alpha) * l + pt2[1]
    z3 = pt2[2] - np.cos(THETA3_MOTOR_SIGN * o2) * l2
    pt3 = [x3,y3,z3]
    
    points.append(pt0)
    points.append(pt1)
    points.append(pt2)
    points.append(pt3)

    return points
    

def computeIKOriented(x, y, z, angle, leg_id, params, verbose=True):
    leg_id-=1
    A = 0.165
    B = 0.020
    C = -0.130
    P = rotaton_2D(x, y, z, LEG_ANGLES[leg_id] + angle)
    P[0] += A
    P[1] += B
    P[2] += C
    
    return np.array([ LEG_ANGLES2[leg_id], 0.0, 0.0]) + computeIK(P[0], P[1], P[2])

def rotation(alpha):
    return np.array([[np.cos(alpha), -np.sin(alpha), 0.],
                     [np.sin(alpha),  np.cos(alpha), 0.],
                     [           0.,             0., 1.]])

def rotaton_2D(x, y, z, ang):
    return rotation(ang) @ np.array([x, y, z])