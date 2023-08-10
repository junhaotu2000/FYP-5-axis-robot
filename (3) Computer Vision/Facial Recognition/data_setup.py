'''
Code to capture and establish the facial database
Copyright Junhao TU
Date: 2023.05

Description: This is the code to recognize the face in the
             captured frames and then save the facial image
             into a database saved in 'database' folder
'''
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

# File to recognize the face in the image
face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# Start capture frames
def start_capture():
    global pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Setup width and height of frames
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)


# Stop capture frames
def stop_capture():
    pipeline.stop()


# Function to recognize and save facial images
def face_setup():
    
    start_capture()

    # Enter face ID (5 IDs maximum)
    face_id = input('\n enter user id  ')
    count = 0

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        colour_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        colour_image = np.asanyarray(colour_frame.get_data())

        # Convert images to be gray
        gray = cv2.cvtColor(colour_image, cv2.COLOR_BGR2GRAY)

        # Compute area of detected face
        faces = face_detector.detectMultiScale(gray, 1.3, 5)
        
        # Save the gray facial images
        for (x,y,w,h) in faces:
            cv2.rectangle(colour_image, (x,y), (x+w,y+h), (255,0,0), 2)     
            count += 1
            
            cv2.imwrite("database/User." + str(face_id) + '.' + str(count) + ".jpg", gray[y:y+h,x:x+w])
            cv2.imshow('image', colour_image)

        k = cv2.waitKey(100) & 0xff 

        if k == 27:
            break
        elif count >= 30: 
            break
    
    stop_capture()
    cv2.destroyAllWindows()

face_setup()
