'''
Code to train the camera according to database
Copyright Junhao TU
Date: 2023.05

Description: This is the code to read the saved facial images 
             in the database and then train the camera to generate
             a recognition file

Output: A .yml recognition file
'''
import cv2
import numpy as np
from PIL import Image
import os

path = 'database' # database path

# Set the file to recognize faces in the frames
recognizer = cv2.face.LBPHFaceRecognizer_create()
detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")

# Get images in database and train the camera
def getImagesAndLabels(path):
    imagePaths = [os.path.join(path,f) for f in os.listdir(path)]     
    faceSamples=[]

    # Train the camera with different IDs
    ids = []

    for imagePath in imagePaths:
        PIL_img = Image.open(imagePath).convert('L') 
        img_numpy = np.array(PIL_img,'uint8')
        id = int(os.path.split(imagePath)[-1].split(".")[1])
        faces = detector.detectMultiScale(img_numpy)
        for (x,y,w,h) in faces:
            faceSamples.append(img_numpy[y:y+h,x:x+w])
            ids.append(id)
    return faceSamples,ids


print ("\n Training faces")

# Train the camera accoridng to database
faces,ids = getImagesAndLabels(path)
recognizer.train(faces, np.array(ids))

# Write the yml recognition file
recognizer.write('trainer/trainer.yml') 

print("\n {0} faces trained".format(len(np.unique(ids))))
