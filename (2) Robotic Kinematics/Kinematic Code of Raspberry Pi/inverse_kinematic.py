'''
Code for inverse kinematics of designed robot
Copyright Junhao TU
Date 2023.05

Description: This is the code to transform the coordinates 
             of end effector to rotational angles of joints

Input: 3D coordinates of end point
Output: Rotational angles of joints
'''

import math as m

# Define configuration parameters
L1 = 261
L2 = 220
L3 = 223
L4 = 158

# Define the math functions
asin = m.asin
acos = m.acos
atan2 = m.atan2
sqrt = m.sqrt
cos = m.cos
sin = m.sin

# Define the fixed angle of end effector
fixed_angle = 0

# Function for inverse kinematics
def inverse(position):
    # Define an error to indicate if the input 
    # position outside the workspace
    error = 0

    # Input coordinates of end point
    x_end = position[0]
    y_end = position[1]
    z_end = position[2]
    
    # Caculate the coordinates of joint 5
    xj5 = x_end - L4*cos(m.radians(fixed_angle))*(x_end / sqrt(x_end**2 + y_end**2))
    yj5 = y_end - L4*cos(m.radians(fixed_angle))*(y_end / sqrt(x_end**2 + y_end**2))
    zj5 = z_end + L4*sin(m.radians(fixed_angle))

    # Determine if the input coordinates ouside the workspace
    P1 = (L2**2 + (xj5**2 + yj5**2 + (L1 - zj5)**2) - L3**2) / (2*L2*sqrt(xj5**2 + yj5**2 + (L1 - zj5)**2))
    if P1 > 1 or P1 < -1:
        print("outside workspace & can not reach")
        joint2_angle = 1000
        error = 1
    elif error == 0:
        # Calculate the joint 2 angle
        joint2_angle = 180 * (m.pi - atan2(sqrt(xj5**2 + yj5**2), (L1 - zj5)) - acos(P1)) / m.pi

    # Determine if the input coordinates ouside the workspace
    P2 = (L2**2 + L3**2 - (xj5**2 + yj5**2 + (L1 - zj5)**2)) / (2*L2*L3)
    if P2 > 1 or P2 < -1:
        print("outside workspace & can not reach")
        joint3_angle = 1000
        error = 2
    elif error == 0:
        # Calculate the joint 3 angle
        joint3_angle = 180 * (m.pi/2 - acos(P2)) / m.pi
    
    # If the input point is in the workspace
    if error == 0:
        # Calculate the joint 1 angle
        joint1_angle = 180 * (atan2(yj5, xj5)) / m.pi
    else:
        joint1_angle = 1000
    
    # Angles buffer for joints
    # joint5_angle = fixed_angle - 1 * (joint2_angle + joint3_angle)
    angles = [joint1_angle, joint2_angle, joint3_angle, 0, \
               (fixed_angle - 1 * (joint2_angle + joint3_angle))]
    
    return angles

