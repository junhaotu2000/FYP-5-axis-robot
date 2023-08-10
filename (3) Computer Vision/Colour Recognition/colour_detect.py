'''
Code to detect coordinates of objects with different colours
Copyright Junhao TU
Date 2023.05

Description: This is the code to recognize the yellow, deep blue and blue objects
             and calcaulte their coordinates 
'''
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import time
import random

# Arrays to save HSV limits for 3 colours
colour_array = {
              'yellow': {'Lower': np.array([10, 150, 130]), 'Upper': np.array([50, 255, 255])},
              'blue': {'Lower': np.array([90, 190, 100]), 'Upper': np.array([100, 255, 210])},
              'deep blue': {'Lower': np.array([100, 210, 40]), 'Upper': np.array([110, 255, 200])},
              }


# Function to show colourized depth image             
def show_colourizer_depth_img(): 
    global depth_frame, colour_image
    colourizer = rs.colorizer()
    hole_filling = rs.hole_filling_filter()
    filled_depth = hole_filling.process(depth_frame)
    colourized_depth = np.asanyarray(colourizer.colorize(filled_depth).get_data())
    cv2.imshow('filled depth',colourized_depth)

# Function to detect the colour of image
def object_colour_detect(colour, Num):
    global depth_frame, colour_image
    hsvFrame = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
    
    # HSV of detected colour
    colour_lower = np.array(colour_array[colour]["Lower"], np.uint8) 
    colour_upper = np.array(colour_array[colour]["Upper"], np.uint8) 
    colour_mask = cv2.inRange(hsvFrame, colour_lower, colour_upper) 
	
    colour_mask = cv2.medianBlur(colour_mask, 9)  # Median Filtering

    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))
    colour_mask = cv2.dilate(colour_mask, kernel)  # Dilation
	
    # Find the colour area in the image
    area = cv2.bitwise_and(colour_image, colour_image, mask = colour_mask) 
    cv2.imshow("Colour Detection area in Real-Time", area)
    
	# Draw contour to track specific colour 
    contours, hierarchy = cv2.findContours(colour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    try:

        # The minimum rectangle contour 
        c = max(contours, key=cv2.contourArea)
        left_x, left_y, width, height = cv2.boundingRect(c)

        # Save the points of pixels with specific colours into list
        bound_rect = np.array([[[left_x, left_y]], [[left_x + width, left_y]],
                               [[left_x + width, left_y + height]], [[left_x, left_y+height]]])
        colour_list = bound_rect.tolist()
        cv2.drawContours(colour_image, [bound_rect], -1, Num, 2)

    except ValueError:
        colour_list = []
    
    return colour_list


# Function to get the coordinate of one pixel
def get_pixel_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    # Read one pixel
    x = depth_pixel[0]
    y = depth_pixel[1]

    # Calculate the depth of one pixel
    dis = aligned_depth_frame.get_distance(x, y)  

    # Calculate the coordinates of one pixel
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)

    return dis, camera_coordinate


# Function to get the coordinates of a object
def object_coordinates_measure(colour, colour_list, Num):
    global depth_frame, colour_image, aligned_depth_frame, depth_intrin

    # Define the coulour of displayed fonts
    font_colour = [[0, 255, 255], [255, 0, 0], [255, 255, 0]]

    # If colour is detected
    if colour_list != []:
        # Locations of four corners of detected area
        left = colour_list[0][0][0]
        right = colour_list[1][0][0]
        top = colour_list[1][0][1]
        bottom = colour_list[3][0][1]

        # Width and height of detected area
        width = right - left
        height = bottom - top

        # The area for depth measurement
        roi_lx = int(left + width/4)
        roi_rx = int(right - width/4)
        roi_ty = int(top + height/4)
        roi_by = int(bottom - height/4)
        
        # Center of area for depth measurement
        center_x = int(colour_list[0][0][0] + width/2)
        center_y = int(colour_list[0][0][1] + height/2)
        cv2.circle(colour_image, (center_x, center_y), 5, (0,0,255), 0)

        # Arrays to save coordinates of pixels in the detected area
        depth_points_x = []
        depth_points_y = []
        depth_points_z = []
        
        # Measure the depth of object in the detected area
        # Coordinates of object are calcualted as averages of 
        # coordinates of random 60 pixels in the detected area
        for j in range(60):
            rand_x = random.randint(roi_lx, roi_rx)
            rand_y = random.randint(roi_ty, roi_by)
            
            depth_pixel = [rand_x, rand_y]  
            dist, camera_coordinate = get_pixel_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)
            
            depth_point_x = round(camera_coordinate[0]*1000, 2)
            depth_point_y = round(camera_coordinate[1]*1000, 2)
            depth_point_z = round(camera_coordinate[2]*1000, 2)


            if depth_point_x != 0:
                depth_points_x.append(depth_point_x)

            if depth_point_y != 0:
                depth_points_y.append(depth_point_y)

            if depth_point_z != 0:
                depth_points_z.append(depth_point_z)
                
        depth_object_x = round(np.mean(depth_points_x),1)
        depth_object_y = round(np.mean(depth_points_y),1)
        depth_object_z = round(np.mean(depth_points_z),1)
        
        # Transform from orignial coordinate system to robotic coordinate system
        x = depth_object_z
        y = -1 * depth_object_x
        z = -1 * depth_object_y

        # Put detected colour and coordinates in the image
        cv2.putText(colour_image, str(colour), (center_x - 70, top - 25), cv2.FONT_HERSHEY_TRIPLEX, 1.3,
                    font_colour[Num])
        cv2.putText(colour_image, "(" + str(x) + ", " + str(y) + ", " + str(z) + ")", (center_x - 160, bottom - 60), cv2.FONT_HERSHEY_TRIPLEX, 1,
                    font_colour[Num])
        

# Program to compute coordinates of yellow, deep blue and blue objects
if __name__ == "__main__":
    global depth_frame, coluor_image
    
    # Configure depth and colour streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 6)

    # Start streaming
    pipeline.start(config)
    align = rs.align(rs.stream.color)
 
    try:
        while True:
            start = time.time()
            # Wait for a coherent pair of frames: depth and colour
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames) 
             
            depth_frame = frames.get_depth_frame()
            colour_frame = frames.get_color_frame()
            if not depth_frame or not colour_frame:
                continue

            # align colour and depth frame
            aligned_depth_frame = aligned_frames.get_depth_frame() 
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

            # Convert images to numpy arrays
            colour_image = np.asanyarray(colour_frame.get_data())

            # Program to detect different colours
            list_Y = object_colour_detect("yellow", [0, 255, 255])
            object_coordinates_measure("yellow", list_Y, 0)
            list_D = object_colour_detect("deep blue", [255, 0, 0])
            object_coordinates_measure("deep_blue", list_D, 1)
            list_B = object_colour_detect("blue", [255, 255, 0])
            object_coordinates_measure("blue", list_B, 2)

            # show image
            show_colourizer_depth_img()
            cv2.imshow("colour_image", colour_image)
            #cv2.imwrite("colour.jpg", colour_image)

            # Calculate frames per second
            print("FPS:", 1/(time.time()-start), "/s")

            # Press esc or 'q' to close the image window
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()
