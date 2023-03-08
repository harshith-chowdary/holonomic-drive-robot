#!/usr/bin/env python3

import math
import numpy as np

PI = math.pi

def inverse_kinematics(x, y, w, theta):

    '''
    front_alpha = 90*(PI/180) + theta
    right_alpha = 210*(PI/180) + theta
    left_alpha = 330*(PI/180) + theta
    
    a11 = math.cos(front_alpha + PI/2)
    a12 = math.cos(right_alpha + PI/2)
    a13 = math.cos(left_alpha + PI/2)

    a21 = math.sin(front_alpha + PI/2)
    a22 = math.sin(right_alpha + PI/2)
    a23 = math.sin(left_alpha + PI/2)
    '''

    front_alpha = 180*(PI/180) + theta
    right_alpha = 300*(PI/180) + theta
    left_alpha = 420*(PI/180) + theta
    
    a11 = math.cos(front_alpha)
    a12 = math.cos(right_alpha)
    a13 = math.cos(left_alpha)

    a21 = math.sin(front_alpha)
    a22 = math.sin(right_alpha)
    a23 = math.sin(left_alpha)

    #arr = np.array([math.cos(front_alpha + PI/2), math.cos(right_alpha + PI/2), math.cos(left_alpha + PI/2)], [math.sin(front_alpha + PI/2), math.sin(right_alpha + PI/2), math.sin(left_alpha + PI/2)], [1, 1, 1])

    arr = np.array([[a11, a12, a13], [a21, a22, a23], [1,1,1]])
    inverse_arr = np.linalg.pinv(arr)

    vel = [x, y, w]

    arr = np.dot(inverse_arr, vel)
    arr[0] = round(arr[0],6)*-100
    arr[1] = round(arr[1],6)*-100
    arr[2] = round(arr[2],6)*-100

    print(-arr[0], -arr[1], -arr[2])

    return arr[0], arr[1], arr[2]

inverse_kinematics(1, 0, 0, 2.223)
