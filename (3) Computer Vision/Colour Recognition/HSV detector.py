'''
Code to detect HSV of colour of one pixel in the image using mouse
Copyright Junhao TU
Date 2023.05

Description: This is the code to read one image and transform it to HSV colour
             Then, the HSV values of colour of one pixel in the image could be
             measured using mouse
'''
import cv2

# Read the image 
img = cv2.imread('colour.jpg')

# Change the height and width of image
height, width = img.shape[:2]
size = (int(width * 2), int(height * 2))

# Transform the image to HSV colour
img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
     
# Get the HSV colour of one pixel
def getHSV(event, x, y, flags, param):
     if event==cv2.EVENT_LBUTTONDOWN:
      print("HSV is", HSV[y, x])

# Show the HSV of clicked pixel
cv2.imshow("imageHSV", HSV)
cv2.setMouseCallback("imageHSV", getHSV)
cv2.waitKey(0)
