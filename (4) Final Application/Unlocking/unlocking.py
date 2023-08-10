'''
Code for unlocking and bowing to specific individuals
Copyright Junhao TU
Date 2023.05

Description: This is the code to recognize the face and bow to 
             sspecific human
'''
from tracemalloc import start
from joint_motion import all_rotation 
from joint_motion import angle_buff
from Facial_Recognition.facial_recognition import facial_reco
from Facial_Recognition.facial_recognition import start_capture
from Facial_Recognition.facial_recognition import stop_capture
from time import sleep
import math as m
    


if __name__ == '__main__':
    # Move to home position
    all_rotation([0, 0, -90, 0, 90])

    start_capture()
    # Facial recoginition
    name = facial_reco(1)
    stop_capture()

    # Bow to specific person
    all_rotation([0, 50, 30, 0, 60])
    sleep(1)
    print(["Hello " + name])

    all_rotation([0, 0, 0, 0, 0])
    print("\n\nUnlocked Now\n")

