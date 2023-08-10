'''
Code to pick and place different colour objects
Copyright Junhao TU
Date 2023.05

Description: This is the code to recognize the yellow, deep blue and blue cubes
             and then pick & place them on different platforms
'''
# Import functions from other python files
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

color_num = [0, 0, 0] # detected number of yellow, deep_blue, blue

# Function to return to home position
def home():all_rotation([0, 0, 0, 0, 0])

# Function to detect the objcets with specific colour
# and return its coordinates
def detect_colour():
    # Detected colours
    colour = ["yellow", "deep_blue", "blue"]

    # Rotate to detection place
    all_rotation([0, 55, 35, 0, -90])
    # Calculate the location with forward kinematics
    location = forward(angle_buff)
    
    # Start capture frames
    start_capture()

    # Detect the 3 colours until one has been detected
    for num in range(3):
        # Coordinates of object relative to the camera
        relative_location = obj_position(colour[num])

        # If there is no detected colour
        if relative_location[0] == 0 and relative_location[1] == 0 \
        and relative_location[2] == 0:
            if num == 2:
                stop_capture()
                return "no detcted color any more", relative_location
        else:
            # Calculate the coordinates of the object according to base coordinate system
            location[0] = location[0] + relative_location[0] - 52
            location[1] = location[1] + relative_location[1] + 50
            location[2] = location[2] + relative_location[2] + 80

            stop_capture()

            return colour[num], location
    

# Function to pick th eobject with movable camera
def pick_movable_camera(obj_location):

    # Error for detecting if the object outside the workspace
    error = 0
    # Calculate rotational angles using inverse kinematics
    angles = inverse(obj_location)

    # If the object within workspace
    if angles[0] != 1000 and angles[1] != 1000:
        # Lift up to a midpoint to avoid obstacles
        midpoint = inverse([320, 0, 360])
        all_rotation(midpoint)

        # Reach and pick the object
        all_rotation(angles)
        effector(1)

        # Lift up
        obj_location[2] += 65
        angles = inverse(obj_location)
        all_rotation(angles)

        return error
    else:
        error = 1
        return error
    

# Function to place the picked object according to different colours
def place(color, switch):
    if switch == 0:
        if color == "yellow":
            angles = inverse([400 - 30 * color_num[0], -150 - 40 * color_num[0], 350])
            all_rotation(angles)
            angles = inverse([400 - 30 * color_num[0], -150 - 40 * color_num[0], 106])
            all_rotation(angles)
            sleep(0.4)
            effector(2)
            angles = inverse([400 - 30 * color_num[0], -150 - 40 * color_num[0], 350])
            all_rotation(angles)
            color_num[0] += 1
        elif color == "deep_blue":
            angles = inverse([120 + 40 * color_num[1], 360, 350])
            all_rotation(angles)
            angles = inverse([120 + 40 * color_num[1], 360, 80])
            all_rotation(angles)
            sleep(0.4)
            effector(2)
            angles = inverse([120 + 40 * color_num[1], 360, 350])
            all_rotation(angles)
            color_num[1] += 1
        elif color == "blue":
            angles = inverse([400, 210 + 40 * color_num[2], 350])
            all_rotation(angles)
            angles = inverse([400, 210 + 40 * color_num[2], 112])
            all_rotation(angles)
            sleep(0.4)
            effector(2)
            angles = inverse([400, 210 + 40 * color_num[2], 350])
            all_rotation(angles)
            color_num[2] += 1
        

# Function to pick and place the objects based on their colours
if __name__ == '__main__':
    
    while True:
        # Initilize the end effector
        effector(0) 
        detected_color, obj_location = detect_colour()

        # If there is no detected colour
        if obj_location[0] == 0 and obj_location[1] == 0  \
            and obj_location[2] == 0:
            print(detected_color)
            print("Number of Yellow Objects: " + str(color_num[0]) + "\n")
            print("Number of Deep Blue Objects: " + str(color_num[1]) + "\n")
            print("Number of Blue Objects: " + str(color_num[2]) + "\n")
            home()
            break
        
        error = pick_movable_camera(obj_location)
        place(detected_color, error)
        home()
