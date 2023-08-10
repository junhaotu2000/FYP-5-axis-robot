'''
Code to detect coordinates of objects with different colours
Copyright Junhao TU
Date 2023.05

Description: This is the code to recognize the yellow, deep blue and blue objects
             and calcaulte their coordinates 

Output: Coordinates of yellow, deep blue and blue objects
'''
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time
import random
import math as m

# Arrays to save HSV limits for 3 colours
colour_array = {
              'yellow': {'Lower': np.array([10, 150, 130]), 'Upper': np.array([50, 255, 255])},
              'blue': {'Lower': np.array([90, 140, 100]), 'Upper': np.array([100, 255, 210])},
              'deep_blue': {'Lower': np.array([100, 210, 40]), 'Upper': np.array([110, 255, 200])},
              }


# Function to detect colour
def object_colour_detect(colour):
    global depth_frame, colour_image
    hsvFrame = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
    
    # HSV for detected colour
    colour_lower = np.array(colour_array[colour]["Lower"], np.uint8) 
    colour_upper = np.array(colour_array[colour]["Upper"], np.uint8) 
    colour_mask = cv2.inRange(hsvFrame, colour_lower, colour_upper) 
	
    colour_mask = cv2.medianBlur(colour_mask, 9)  # Median Filtering
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5)) 
    colour_mask = cv2.dilate(colour_mask, kernel)  # Dilation

	# Drawing contour to track red colour 
    contours, hierarchy = cv2.findContours(colour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    try:
        # The minimum rectangle contour 
        c = max(contours, key=cv2.contourArea)
        left_x, left_y, width, height = cv2.boundingRect(c)

        # Save the pixels of detected colour into a list
        bound_rect = np.array([[[left_x, left_y]], [[left_x + width, left_y]],
                               [[left_x + width, left_y + height]], [[left_x, left_y+height]]])
        colour_list = bound_rect.tolist()
        
    except ValueError:
        colour_list = []
    
    return colour_list


# Function to get coordinates of one pixel
def get_pixel_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]

    # Calculate the depth of one pixel
    dis = aligned_depth_frame.get_distance(x, y) 
    # Compute the coordinates of one point
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    return camera_coordinate


# Function to get coordinates of a object
def object_coordinates_measure(colour_list, aligned_depth_frame, depth_intrin):
    global depth_frame, colour_image 
    
    if colour_list != []:
        # Locations of four corners of detected area
        left = colour_list[0][0][0]
        right = colour_list[1][0][0]
        top = colour_list[1][0][1]
        bottom = colour_list[3][0][1]
        width = right - left
        height = bottom - top
        
        # The area for depth measurement
        roi_lx = int(left + width/4)
        roi_rx = int(right - width/4)
        roi_ty = int(top + height/4)
        roi_by = int(bottom - height/4)

        # Arrays to save coordinates of pixels in the detected area
        depth_points_x = []
        depth_points_y = []
        depth_points_z = []
        
        # Coordinates of object are calcualted as averages of 
        # coordinates of random 60 pixels in the detected area
        for j in range(60):
            rand_x = random.randint(roi_lx, roi_rx)
            rand_y = random.randint(roi_ty, roi_by)
            
            depth_pixel = [rand_x, rand_y]  
            object_coordinate = get_pixel_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)
            
            depth_point_x = round(object_coordinate[2]*1000, 2)
            depth_point_y = round(object_coordinate[0]*(-1000), 2)
            depth_point_z = round(object_coordinate[1]*(-1000), 2)

            if depth_point_x != 0 and m.isnan(depth_point_x) == False:
                depth_points_x.append(depth_point_x)

            if depth_point_y != 0 and m.isnan(depth_point_y) == False:
                depth_points_y.append(depth_point_y)

            if depth_point_z != 0 and m.isnan(depth_point_z) == False:
                depth_points_z.append(depth_point_z)
                
        # dist_mean = round(np.mean(distances),1)
        object_x = round(np.mean(depth_points_x),1)
        object_y = round(np.mean(depth_points_y),1)
        object_z = round(np.mean(depth_points_z),1)

        return [object_x, object_y, object_z]
    

# Function to start capture frames    
def start_capture():
    global align, pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 480, 270, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 60)
    # Start streaming
    pipeline.start(config)
    align = rs.align(rs.stream.color)


# Function to stop capture frames
def stop_capture():
    pipeline.stop()

          
# Function to detect yellow, deep blue and blue objects and return their coordinates 
def obj_position():
    global depth_frame, colour_image

    # Start to record time
    start = time.time()

    # Record the number of detected test for different colours
    detected_Num_Y = 0 # Yellow
    detected_Num_D = 0 # Deep blue
    detected_Num_B = 0 # Blue

    # Save the detected coordinates of objects with different colours
    x_position_Y = [0, 0, 0, 0]
    y_position_Y = [0, 0, 0, 0]
    z_position_Y = [0, 0, 0, 0]

    x_position_D = [0, 0, 0, 0]
    y_position_D = [0, 0, 0, 0]
    z_position_D = [0, 0, 0, 0]

    x_position_B = [0, 0, 0, 0]
    y_position_B = [0, 0, 0, 0]
    z_position_B = [0, 0, 0, 0]

    # Array to save the coorinates of detected objects
    yellow_coord = [0, 0, 0]  
    deep_blue_coord = [0, 0, 0] 
    blue_coord = [0, 0, 0] 

    while True:
        # Wait for a coherent pair of frames: depth and colour
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        colour_frame = frames.get_color_frame()
        if not depth_frame or not colour_frame:
             continue

        # Convert images to numpy arrays
        colour_image = np.asanyarray(colour_frame.get_data())
        
        # task program
        yellow_list = object_colour_detect("yellow")
        deep_blue_list = object_colour_detect("deep_blue")
        blue_list = object_colour_detect("blue")

        aligned_depth_frame = aligned_frames.get_depth_frame() 
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # Detected coordinates of yellow, deep blue and blue objects
        # coordinates are average values of that detected from first four detecting tests
        if yellow_list != [] and detected_Num_Y < 4:
            position_Y = object_coordinates_measure(yellow_list, aligned_depth_frame, depth_intrin)
            x_Y = position_Y[0]
            y_Y = position_Y[1]
            z_Y = position_Y[2]
            if m.isnan(x_Y) == False and m.isnan(y_Y) == False and m.isnan(z_Y) == False:
                if x_Y < 500 and y_Y < 300 and y_Y > -300 and z_Y < 300 and z_Y > -300:
                    x_position_Y[detected_Num_Y] = x_Y
                    y_position_Y[detected_Num_Y] = y_Y
                    z_position_Y[detected_Num_Y] = z_Y
                    detected_Num_Y += 1
                    if detected_Num_Y == 4:
                        yellow_coord [0] = np.mean(x_position_Y)
                        yellow_coord [1] = np.mean(y_position_Y)
                        yellow_coord [2] = np.mean(z_position_Y)
                    
        if deep_blue_list != [] and detected_Num_D < 4:
            position_D = object_coordinates_measure(deep_blue_list, aligned_depth_frame, depth_intrin)
            x_D = position_D[0]
            y_D = position_D[1]
            z_D = position_D[2]
            if m.isnan(x_D) == False and m.isnan(y_D) == False and m.isnan(z_D) == False:
                if x_D < 500 and y_D < 300 and y_D > -300 and z_D < 300 and z_D > -300:
                    x_position_D[detected_Num_D] = x_D
                    y_position_D[detected_Num_D] = y_D
                    z_position_D[detected_Num_D] = z_D
                    detected_Num_D += 1
                    if detected_Num_D == 4:
                        deep_blue_coord [0] = np.mean(x_position_D)
                        deep_blue_coord [1] = np.mean(y_position_D)
                        deep_blue_coord [2] = np.mean(z_position_D)
                    
        if blue_list != [] and detected_Num_B < 4:
            position_B = object_coordinates_measure(blue_list, aligned_depth_frame, depth_intrin)
            x_B = position_B[0]
            y_B = position_B[1]
            z_B = position_B[2]
            if m.isnan(x_B) == False and m.isnan(y_B) == False and m.isnan(z_B) == False:
                if x_B < 500 and y_B < 300 and y_B > -300 and z_B < 300 and z_B > -300:
                    x_position_B[detected_Num_B] = x_B
                    y_position_B[detected_Num_B] = y_B
                    z_position_B[detected_Num_B] = z_B
                    detected_Num_B += 1
                    if detected_Num_B == 4:
                        blue_coord [0] = np.mean(x_position_B)
                        blue_coord [1] = np.mean(y_position_B)
                        blue_coord [2] = np.mean(z_position_B)
        
        # If the detecting time larger than 1.6s
        if (time.time() - start) > 1.6:
            # Break the loop and return coordinates
            return yellow_coord, deep_blue_coord, blue_coord
