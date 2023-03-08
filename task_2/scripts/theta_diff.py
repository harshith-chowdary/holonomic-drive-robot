#!/usr/bin/env python3

PI = 22/7

theta_diff = -4.67
hola_theta = 2.235

if theta_diff>0:
    if (theta_diff//PI)%2 == 0:
        theta_diff-= 2*PI*(theta_diff//PI)
    else:
        theta_diff = -PI + theta_diff%PI
elif theta_diff<0:
    theta_diff = abs(theta_diff)
    if (theta_diff//PI)%2 == 0:
        theta_diff-= 2*PI*(theta_diff//PI)
    else:
        theta_diff = -PI + theta_diff%PI
    theta_diff*=-1

theta_d = theta_diff

print("theta_diff : ",theta_diff,end= " ")
 
print("theta_d : ",theta_d)

