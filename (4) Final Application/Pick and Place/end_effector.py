'''
Code to control end effector
Copyright Junhao TU
Date 2023.05

Description: This is the code to switch ON/OFF end effector 

Input: Switch number
'''
import RPi.GPIO as GPIO
import time

# Define the pin for end effector
endpin = 8


# Function to transform the angle to duty cycle
def servo_angle(angle): 
    duty_cycle = int(2.5 + (10 / 180) * angle)
    return duty_cycle


# Set the pin mode
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(endpin, GPIO.OUT)


# Setup the basic wave
basic_wave = GPIO.PWM(endpin,50) #50HZ


# Function to control end effector
def effector(switch):
	if switch == 0:
		# Initialize the end effector
		basic_wave.start(servo_angle(0)) 
		time.sleep(0.5)
		basic_wave.ChangeDutyCycle(0) 
		time.sleep(0.01)
	elif switch == 1:
		# Switch ON the end effector
		basic_wave.ChangeDutyCycle(servo_angle(140))
		time.sleep(1)
	else:
		# Switch OFF the end effector
		basic_wave.ChangeDutyCycle(servo_angle(0))
		time.sleep(1)
