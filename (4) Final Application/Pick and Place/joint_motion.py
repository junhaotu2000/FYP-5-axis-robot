'''
Code for simultaneous control of 5 joints
Copyright Junhao TU
Date 2023.05

Description: This is the code to control the 5 joints to rotate 
             to the desired angles simultaneously

Input: Rotational angles of joints
'''

import RPi.GPIO as GPIO
import numpy as np
import time
import multiprocessing

# Direction pin from controller
DIR = [21, [7, 19], 15, 13, 11]
# Step pin from controller
PUL = [22, 18, 16, 12, 10]

# Angle resolution of stepper motors
RESm = [3200, 6400, 1600, 3200, 3200]

# Angle limits for 5 joints
angle_limit = [[-160, 160], [-140, 140], [-230, 50], [-140, 140], [-140, 140]]

# Angle buffere to save current absolute rotational angles
angle_buff = [0, 0, -90, 0, 0]

# 0/1 used to signify clockwise or counterclockwise.
CW = 1
CCW = 0

# Calcuate the angle resolution of 5 joints
RESj = [0, 0, 0, 0, 0]
RESj[0] = (5.5*RESm[0])/360
RESj[1] = (5.5*RESm[1])/360
RESj[2] = ((61/14)*5*RESm[2])/360
RESj[3] = RESm[3]/360
RESj[4] = (4.5*RESm[4])/360

# Define [vmin, vmax] for stepper motors of 5 joints (< 20 degrees)
v_thre_small_deg = [[10, 80], [10, 150], [45, 500], [2, 20], [10, 90]]  # unit: rpm

# Define [vmin, vmax] for stepper motors of 5 joints (> 20 degrees)
v_thre_large_deg = [[16, 100], [16, 550], [65, 2000], [3, 50], [13, 250]] # unit:rpm

# Define a micro delay function
usleep = lambda x: time.sleep(x/1000000.0)

# Define the function to transform the speed to signal period
def v_to_T(v_array):
	T_array = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]

	for m in range(5):
		T_array[m][0] = (60 / (RESm[m] * v_array[m][1]))*1000000 # unit: us
		T_array[m][1] = (60 / (RESm[m] * v_array[m][0]))*1000000 # unit: us
	
	return T_array

# Calculate the signal periods
T_thre_small_deg = v_to_T(v_thre_small_deg)
T_thre_large_deg = v_to_T(v_thre_large_deg)

# Setup pin layout on Pi
GPIO.setmode(GPIO.BOARD)

# Setup the pin mode
GPIO.setwarnings(False)
for n in range(5):
	if n == 1:
		GPIO.setup(DIR[n][0], GPIO.OUT)
		GPIO.setup(DIR[n][1], GPIO.OUT)
		GPIO.setup(PUL[n], GPIO.OUT)
	else:
		GPIO.setup(DIR[n], GPIO.OUT)
		GPIO.setup(PUL[n], GPIO.OUT)

# Transformation function to synchronize the motion of joints
def T_min_Trans (angles):
	global T_thre_small_deg, T_thre_large_deg

	# Array to save total number of output pulses to 5 joints
	PUL_num = [0, 0, 0, 0, 0]
	
	# Array to save the rotation times of 5 joints
	run_time = [0, 0, 0, 0, 0]

	# Calcualte the rotation times of 5 joints
	for n in range(5):
		PUL_num[n] = int(abs(angles[n]) * RESj[n])

		if abs(angles[n]) < 20:
			T_min = T_thre_small_deg[n][0]
			T_max = T_thre_small_deg[n][1]
		else:
			T_min = T_thre_large_deg[n][0]
			T_max = T_thre_large_deg[n][1]
		
		run_time[n] = 0.1 * PUL_num[n] * (2 * T_max + 8 * T_min)
	
	# Determine the maximum time among all rotation times
	max_time = np.max(run_time)

	# Recalculate Tmin to synchronize rotation of all joints
	for m in range(5):
		if PUL_num[m] != 0:
			if abs(angles[m]) < 20:
				T_max = T_thre_small_deg[m][1]
				T_thre_small_deg[m][0] = (((10 * max_time) / PUL_num[m])
				 - 2 * T_max) / 8 
			else:
				T_max = T_thre_large_deg[m][1]
				T_thre_large_deg[m][0] = (((10 * max_time) / PUL_num[m])
				 - 2 * T_max) / 8
		

# Control function to rotate one joint with desired angle
# Input: joint number, desired angle
def joint_rotation (joint, angle):
	
	if joint > 5 or joint < 0:
		print("error axis")

	# Define the direction of each joint
	if angle >= 0:
		if joint == 1:
			GPIO.output(DIR[joint - 1],CCW)
		elif joint == 2:
			GPIO.output(DIR[joint - 1][0],CW)
			GPIO.output(DIR[joint - 1][1],CCW)
		else:
			GPIO.output(DIR[joint - 1],CW)
	else:
		if joint == 1:
			GPIO.output(DIR[joint - 1],CW)
		elif joint == 2:
			GPIO.output(DIR[joint - 1][0],CCW)
			GPIO.output(DIR[joint - 1][1],CW)
		else:
			GPIO.output(DIR[joint - 1],CCW)

	# Determine the Tmin and Tmax
	if abs(angle) < 20:
		T_min = T_thre_small_deg[joint - 1][0]
		T_max = T_thre_small_deg[joint - 1][1]
	else:
		T_min = T_thre_large_deg[joint - 1][0]
		T_max = T_thre_large_deg[joint - 1][1]

	# Caculate the number of output pulse
	pulNum = int(abs(angle) * RESj[joint - 1])

	# Send the pulse signal to joint with variable period
	for n in range(pulNum):
		T_change_v = (T_max - T_min) / (pulNum * 0.2)
		
		t_delay = (T_max - T_change_v * n) / 2

		if t_delay < (T_min / 2):
			t_delay = T_min / 2

		if n >= 0.8 * pulNum:
			t_delay = (T_min + T_change_v * (n - 0.8 * pulNum)) / 2
		
		GPIO.output(PUL[joint - 1], GPIO.HIGH)
		usleep (t_delay)
		GPIO.output(PUL[joint - 1], GPIO.LOW)
		usleep (t_delay)


# Control function to rotate 5 joints simutaneously
# Input: Rotational angles of 5 joints
def all_rotation (angles):
	global angle_buff, T_thre_large_deg, T_thre_small_deg
	
	# buffer to save current [Tmin, Tmax]
	T_buff_small = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
	T_buff_large = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
	
	for row in range(5):
		for column in range(2):
			T_buff_small[row][column] = T_thre_small_deg[row][column]
			T_buff_large[row][column] = T_thre_large_deg[row][column]

	# Arrays for relative angle 
	rotate_angle = [0, 0, 0, 0, 0]

	# Arrays to save rotation functions of 5 joints
	joints_rotation = [ 0, 0, 0, 0, 0]

	# Check if the input angles exceed the limits
	for n in range(5):
		if angles[n] > angle_limit[n][1] or angles[n] < angle_limit[n][0]:
			print("joint " + str(n+1) + " angle error")
			for num in range(5):
				angles[num] = angle_buff[num]
			break
	
	# Calculate the relative angles of joints
	# Save current absolute angles of joints
	for m in range(5):
		rotate_angle[m] = angles[m] - angle_buff[m]
		angle_buff[m] = angles[m]

	# Transform Tmin to synchronize motions of joints
	T_min_Trans(rotate_angle)

	# Setup multithreading for 5 joints rotation
	for m in range(5):
		joints_rotation[m] = multiprocessing.Process(target=joint_rotation, args= (m+1, rotate_angle[m]))
		joints_rotation[m].start()

	# Enable multithreading for 5 joints rotation
	for k in range(5):
		joints_rotation[k].join()
		
	for row in range(5):
		for column in range(2):
			T_thre_small_deg[row][column] = T_buff_small[row][column]
			T_thre_large_deg[row][column] = T_buff_large[row][column]
