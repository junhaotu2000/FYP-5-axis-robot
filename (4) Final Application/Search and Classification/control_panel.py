'''
Code to search and classify the objects around robot
Copyright Junhao TU
Date 2023.05

Description: This is the code to search the yellow, deep blue and blue cubes
             and then pick & place them on different platforms
'''
from tracemalloc import start
from end_effector import effector 
from joint_motion import all_rotation 
from joint_motion import angle_buff
from inverse_kinematic import inverse
from forward_kinematic import forward
from colour_detect import obj_position
from colour_detect import start_capture
from colour_detect import stop_capture
from time import sleep
import math as m

colour_num = [0, 0, 0] # detected number of yellow, deep_blue, blue


# Function to return to home position
def home():all_rotation([0, 0, 0, 0, 0])


# Function to search and pick objects with different colour
def pick(obj_location, area):

    # Error for detecting if the object outside the workspace
    error = 0

    # Detection positions for searching
    camera_detection_pos = [[0, 55, 35, 0, -90], [50, 55, 35, 50, -90], \
    [50, 55, 35, -40, -90]]

    # Calcualte the rotational angles to reach the object
    angles = inverse(obj_location)

    # If the object is outside the workspace
    if angles[0] != 1000 and angles[1] != 1000:
        # Calculate the position of detection position
        end_position = forward(camera_detection_pos[area])

        # Lift up the end effector to avoid obstacles
        end_position[2] += 200
        midpoint = inverse(end_position)
        sleep(0.4)
        all_rotation(midpoint)

        # Reach and pick the object
        all_rotation(angles)
        sleep(0.5)
        effector(1)

        # Lift up to avoid obstacles
        obj_location[2] += 65
        angles = inverse(obj_location)
        all_rotation(angles)
        return error
    else:
        error = 1
        return error


# Function to place the objects to different platforms
# 'switch' is to enable/disable the place function
def place(colour, switch):
    if switch == 0:
        # Place the objects into different platforms
        if colour == "yellow":
            angles = inverse([400 - 30 * colour_num[0], -175 - 40 * colour_num[0], 350])
            all_rotation(angles)
            angles = inverse([400 - 30 * colour_num[0], -175 - 40 * colour_num[0], 65])
            all_rotation(angles)
            sleep(0.5)
            effector(2)
            angles = inverse([400 - 30 * colour_num[0], -175 - 40 * colour_num[0], 350])
            all_rotation(angles)
            colour_num[0] += 1
        elif colour == "deep_blue":
            angles = inverse([120 + 50 * colour_num[1], -380, 350])
            all_rotation(angles)
            angles = inverse([120 + 50 * colour_num[1], -380, 60])
            all_rotation(angles)
            sleep(0.5)
            effector(2)
            angles = inverse([120 + 50 * colour_num[1], -380, 350])
            all_rotation(angles)
            colour_num[1] += 1
        elif colour == "blue":
            angles = inverse([140 + 40 * colour_num[1], -190, 350])
            all_rotation(angles)
            angles = inverse([140 + 40 * colour_num[1], -190, 20])
            all_rotation(angles)
            sleep(0.5)
            effector(2)
            angles = inverse([140 + 40 * colour_num[1], -190, 350])
            all_rotation(angles)
            colour_num[2] += 1

    
# Function to search and classfy objects
def detect_colour(area):
    # Calculate location of camera using forward kinematic
    location= forward(angle_buff)

    # Arrays to save the offset of camera
    camera_offset = [[-52, 52, 80], [-56, -92, 68], [-35, 70, 78]]

    # Coordinates of different objects with different colours
    location_Y = [0, 0, 0]
    location_D = [0, 0, 0]
    location_B = [0, 0, 0]

    # Detected number of different objects
    detected_Y = 1
    detected_D = 1
    detected_B = 1
    
    # Start capture frame
    start_capture()

    # Coordinates of different objects
    relative_location_Y, relative_location_D, relative_location_B = obj_position()
    
    # Stop capture frame
    stop_capture()

    # Calculate the coordinates of yellow object
    if relative_location_Y != [0, 0, 0]:
        if area == 2:
            location_Y[0] = location[0] + camera_offset[area][0] - relative_location_Y[1]
            location_Y[1] = location[1] + camera_offset[area][1] + relative_location_Y[0]
            location_Y[2] = location[2] + camera_offset[area][2] + relative_location_Y[2]
        else:
            location_Y[0] = location[0] + camera_offset[area][0] + relative_location_Y[0]
            location_Y[1] = location[1] + camera_offset[area][1] + relative_location_Y[1]
            location_Y[2] = location[2] + camera_offset[area][2] + relative_location_Y[2]
    elif relative_location_Y == [0, 0, 0]:
        # if there is no yellow objects detected
        detected_Y = 0

    # Calculate the coordinates of deep blue object
    if relative_location_D != [0, 0, 0]:
        if area == 2:
            location_D[0] = location[0] + camera_offset[area][0] - relative_location_D[1]
            location_D[1] = location[1] + camera_offset[area][1] + relative_location_D[0]
            location_D[2] = location[2] + camera_offset[area][2] + relative_location_D[2]
        else:
            location_D[0] = location[0] + camera_offset[area][0] + relative_location_D[0]
            location_D[1] = location[1] + camera_offset[area][1] + relative_location_D[1]
            location_D[2] = location[2] + camera_offset[area][2] + relative_location_D[2]
    elif relative_location_D == [0, 0, 0]:
         # if there is no deep blue objects detected
        detected_D = 0
    
    # Calculate the coordinates of blue object
    if relative_location_B != [0, 0, 0]:
        if area == 2:
            location_B[0] = location[0] + camera_offset[area][0] - relative_location_B[1] 
            location_B[1] = location[1] + camera_offset[area][1] + relative_location_B[0] 
            location_B[2] = location[2] + camera_offset[area][2] + relative_location_B[2]
        else:
            location_B[0] = location[0] + camera_offset[area][0] + relative_location_B[0] 
            location_B[1] = location[1] + camera_offset[area][1] + relative_location_B[1] 
            location_B[2] = location[2] + camera_offset[area][2] + relative_location_B[2]
    elif relative_location_B == [0, 0, 0]:
         # if there is no blue objects detected
        detected_B = 0

    # If the objects are detected, pick and place the objects    
    if detected_Y == 1:
        error = pick(location_Y, area)
        place("yellow", error)
    
    if detected_D == 1:
        error = pick(location_D, area)
        place("deep_blue", error)
        
    if detected_B == 1:
        error = pick(location_B, area)
        place("blue", error)

    # If there is no objects detected
    if detected_Y == 0 and detected_D == 0 and detected_B == 0:
        return 0
    else:
        return 1
    

# Function to search and classify the objects until there is no
# objects detected
if __name__ == '__main__':
    
    # Switch to check if there is a object detected
    detected_switch_1 = 1
    detected_switch_2 = 1
    detected_switch_3 = 1
    
    while True:
        # Initialize the end effector
        effector(0) 

        # If objects are detected
        if detected_switch_1 == 1:
            all_rotation([0, 55, 35, 0, -90])
            sleep(0.5)
            detected_switch_1 = detect_colour(0)
        if detected_switch_2 == 1:
            all_rotation([50, 55, 35, 50, -90])
            sleep(0.5)
            detected_switch_2 = detect_colour(1)
        if detected_switch_3 == 1:
            all_rotation([50, 55, 35, -40, -90])
            sleep(0.5)
            detected_switch_3 = detect_colour(2)
        
        # If there is no objects detected
        if detected_switch_1 == 0 and detected_switch_2 == 0 and detected_switch_3 == 0:
            home()
            print("\n\n\n\n\n\n\n\n\n\n\nNo detected colour any more\n")
            print("Number of Yellow Objects: " + str(colour_num[0]) + "\n")
            print("Number of Deep Blue Objects: " + str(colour_num[1]) + "\n")
            print("Number of Blue Objects: " + str(colour_num[2]) + "\n")
            sleep(0.5)
            break
