'''
Code to recognize specific face and calculate its coordinates
Date: 2023.05

Description: This is the code to recognize the face in database
             and calculate the coordinates 
'''
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import random
import math as m
import os 

# Setup the face recognition files
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('trainer/trainer.yml')
cascadePath = "haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascadePath)

font = cv2.FONT_HERSHEY_SIMPLEX # Font type

# Minimum heigh and width of detected window
minW = 64
minH = 48

id = 0 # ID number

# Names according to different IDs
names = ['None', 'Yihan Liu', 'Name 2', 'Name 3', 'Name 4', 'Name 5'] 


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


# Function to get the coordinates of a face
def object_coordinates_measure(face_list, aligned_depth_frame, depth_intrin):
    global depth_frame, colour_image

    # If face is detected
    if face_list != []:
        # Locations of four corners of detected area
        left = face_list[0][0][0]
        right = face_list[1][0][0]
        top = face_list[1][0][1]
        bottom = face_list[3][0][1]

        # Width and height of detected area
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
        
        # Measure the depth of object in the detected area
        # Coordinates of object are calcualted as averages of 
        # coordinates of random 60 pixels in the detected area
        for j in range(60):
            rand_x = random.randint(roi_lx, roi_rx)
            rand_y = random.randint(roi_ty, roi_by)
            
            depth_pixel = [rand_x, rand_y]  
            dist, camera_coordinate = get_pixel_coordinate(depth_pixel, aligned_depth_frame, depth_intrin)
            
            depth_point_x = round(camera_coordinate[2]*1000, 2)
            depth_point_y = round(camera_coordinate[0]*(-1000), 2)
            depth_point_z = round(camera_coordinate[1]*(-1000), 2)


            if depth_point_x != 0:
                depth_points_x.append(depth_point_x)

            if depth_point_y != 0:
                depth_points_y.append(depth_point_y)

            if depth_point_z != 0:
                depth_points_z.append(depth_point_z)
                
        object_x = round(np.mean(depth_points_x),1)
        object_y = round(np.mean(depth_points_y),1)
        object_z = round(np.mean(depth_points_z),1)
        
        return [object_x, object_y, object_z]


# Start capture frames
def start_capture():
    global align,pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    # Setup height and weight of frames
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    # Start streaming
    pipeline.start(config)
    align = rs.align(rs.stream.color)

# Stop capture frames
def stop_capture():
    pipeline.stop()

# facial recognition code
def facial_reco():
    global depth_frame, colour_image

    start_capture()

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        colour_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not depth_frame or not colour_frame:
             continue
    
        # Convert images to numpy arrays
        colour_image = np.asanyarray(colour_frame.get_data())
        
        aligned_depth_frame = aligned_frames.get_depth_frame() 
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

        # Convert image to be gray
        gray = cv2.cvtColor(colour_image, cv2.COLOR_BGR2GRAY)
        
        # Setup detection window
        faces = faceCascade.detectMultiScale(
            gray,     
            scaleFactor=1.2,
            minNeighbors=5,   
            minSize = (minW, minH), 
        )

        # Compare the captured frame and training file to recognize the faces
        for(x,y,w,h) in faces:
            cv2.rectangle(colour_image, (x,y), (x+w,y+h), (0,255,0), 2)
            id, confidence = recognizer.predict(gray[y:y+h,x:x+w])

            # Get the area of face
            bound_rect = np.array([[[x, y]], [[x + w, y]],
                               [[x + w, y + h]], [[x, y+h]]])
            face_list = bound_rect.tolist()
           
            # Calculate the facial coordinates
            face_location = object_coordinates_measure(face_list, aligned_depth_frame, depth_intrin)

            if (confidence < 100):
                id = names[id]
                confidence = "  {0}%".format(round(100 - confidence))
            else:
                id = "unknown"
                confidence = "  {0}%".format(round(100 - confidence))
            
            cv2.putText(colour_image, str(id), (x+5,y-5), font, 1, (255,255,255), 2)
            cv2.putText(colour_image, str(confidence), (x+5,y+h-5), font, 1, (255,255,0), 1)
            cv2.putText(colour_image, str(face_location), (x-4,y+h+20), font, 0.6, (255,255,255), 2)  


        cv2.imshow('frame', colour_image)
        
        k = cv2.waitKey(30) & 0xff
        
        if k == 27: 
            break

    print("\nExiting Program")
    stop_capture()
    cv2.destroyAllWindows()

facial_reco()

